import os
import time
import math
import numpy as np
import subprocess
import rospy
import argparse
import rospkg

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid  
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist, Quaternion, Vector3, TransformStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from map_msgs.msg import OccupancyGridUpdate
import actionlib
from nav_msgs.msg import Path
import laser_geometry.laser_geometry as LaserGeometry
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from std_srvs.srv import Empty
from os.path import join
import tf

from gazebo_simulation import GazeboSimulation

GOAL_REACHED_DIST = 0.3
COLLISION_DIST = 0.35
TIME_DELTA = 0.1


def compute_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


class GazeboEnv:
    def __init__(self, world_idx, gui=True, environment_dim = 100):
        """환경 초기화 및 Gazebo 실행"""
        self.world_idx = world_idx
        self.gui = gui

        # 초기 변수 설정
        self.environment_dim = environment_dim
        self.last_odom = None
        self.lp = LaserGeometry.LaserProjection()
        self.pre_distance = 1000.0
        self.min_distance = 1000.0
        self.global_plan = None
        self.timestep = 0
        self.last_distance = None
        self.left_path = False
        self.last_path_idx = None
        self.sensor_data = np.ones(self.environment_dim) * 10

        self.gaps = [[-np.pi / 2 - 0.03, -np.pi / 2 + np.pi / self.environment_dim]]
        for m in range(self.environment_dim - 1):
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim]
            )
        self.gaps[-1][-1] += 0.03

        # Gazebo 환경 로드
        self.init_gazebo()

        # set ROS publisher and subscriber
        self.init_ros()

        # publish fake odom
        self.init_odom_publisher()

        self.init_global_guide()

    def init_gazebo(self):
        os.environ["JACKAL_LASER"] = "1"
        os.environ["JACKAL_LASER_MODEL"] = "ust10"
        os.environ["JACKAL_LASER_OFFSET"] = "-0.065 0 0.01"

        # ROS 패키지 경로 설정
        rospack = rospkg.RosPack()
        base_path = rospack.get_path('leebots_local_planner')
        os.environ['GAZEBO_PLUGIN_PATH'] = os.path.join(base_path, "plugins")

        # World 설정
        if self.world_idx < 300:
            self.world_name = f"BARN/world_{self.world_idx}.world"
            self.init_position = [-2.25, 3, 1.57]
            self.goal_position = [0, 10]
        elif self.world_idx < 360:
            self.world_name = f"DynaBARN/world_{self.world_idx - 300}.world"
            self.init_position = [11, 0, 3.14]
            self.goal_position = [-20, 0]
        else:
            raise ValueError(f"World index {self.world_idx} does not exist")
        
        self.pre_distance =  np.linalg.norm(
            [self.init_position[0] - self.goal_position[0], self.init_position[1] - self.goal_position[1]]
        )

        launch_file = os.path.join(base_path, 'launch', 'gazebo_launch.launch')
        world_file = os.path.join(base_path, "worlds", self.world_name)

        # Gazebo 실행
        self.gazebo_process = subprocess.Popen([
            'roslaunch',
            launch_file,
            f'world_name:={world_file}',
            f'gui:={"true" if self.gui else "false"}'
        ])
        time.sleep(5)  # Gazebo 로딩 대기
        try:
            node_name = f'gym_{self.world_idx}'
            rospy.init_node(node_name)#, anonymous=True)#, log_level=rospy.FATAL)
        except rospy.exceptions.ROSException as e:
            print("Failed to initialize node:", str(e))

        rospy.set_param('/use_sim_time', True)
    
        self.gazebo_sim = GazeboSimulation(init_position=self.init_position)
        # 초기 위치 확인 및 리셋
        init_coor = (self.init_position[0], self.init_position[1])
        goal_coor = (self.init_position[0] + self.goal_position[0], self.init_position[1] + self.goal_position[1])

        pos = self.gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        collided = True
        while compute_distance(init_coor, curr_coor) > 0.1 or collided:
            self.gazebo_sim.reset()
            pos = self.gazebo_sim.get_model_state().pose.position
            curr_coor = (pos.x, pos.y)
            collided = self.gazebo_sim.get_hard_collision()
            time.sleep(1)

    def init_ros(self):
        self.vel_pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size=10)
        self.odom = rospy.Subscriber("/jackal_velocity_controller/odom", Odometry, self.odom_callback, queue_size=10)
        self.sensor = rospy.Subscriber("/front/scan", LaserScan, self.lidar_callback, queue_size=10)
        self.global_plan_sub = rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.global_plan_callback, queue_size=10)
        self.set_state = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=10)
        self.plan_pub = rospy.Publisher("/td3_global_plan", Path, queue_size=10)

    def init_odom_publisher(self):
        os.system('python3 fake_odom_publisher.py &')

    def init_global_guide(self):
         # ROS 패키지 경로 설정
        rospack = rospkg.RosPack()
        base_path = rospack.get_path('leebots_local_planner')

        launch_file = os.path.join(base_path, 'launch', 'move_base_DWA.launch')

        # Global path 실행
        self.globalpath_process = subprocess.Popen([
            'roslaunch',
            launch_file
        ])

    def odom_callback(self, od_data):
        self.last_odom = od_data

    def lidar_callback(self, msg):
        self.sensor_data = np.ones(self.environment_dim) * 10
        pc2_msg = self.lp.projectLaser(msg)
        #self.sensor_data = np.array(pc2.read_points_list(pc2_msg))
        lidar_data = pc2.read_points_list(pc2_msg, skip_nans=False, field_names=("x", "y", "z"))
        for i in range(len(lidar_data)):
            if lidar_data[i][2] > -0.2:
                dot = lidar_data[i][0] * 1 + lidar_data[i][1] * 0
                mag1 = math.sqrt(math.pow(lidar_data[i][0], 2) + math.pow(lidar_data[i][1], 2))
                mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
                beta = math.acos(dot / (mag1 * mag2)) * np.sign(lidar_data[i][1])
                dist = math.sqrt(lidar_data[i][0] ** 2 + lidar_data[i][1] ** 2 + lidar_data[i][2] ** 2)

                for j in range(len(self.gaps)):
                    if self.gaps[j][0] <= beta < self.gaps[j][1]:
                        self.sensor_data[j] = min(self.sensor_data[j], dist)
                        break

    def global_plan_callback(self, msg):
        plan_points = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            plan_points.append([x, y])

        global_plan = np.array(plan_points)

        #rospy.loginfo(f"Received global plan with {len(global_plan)} points")

    def get_global_plan(self, start_pos, goal_pos, tolerance=0.5):
        rospy.wait_for_service('/move_base/make_plan')
        try:
            make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

            start = PoseStamped()
            start.header.frame_id = "odom"
            start.header.stamp = rospy.Time.now()
            start.pose.position.x = start_pos[0]
            start.pose.position.y = start_pos[1]
            start.pose.orientation.w = 1.0  # 기본 방향

            goal = PoseStamped()
            goal.header.frame_id = "odom"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = goal_pos[0]
            goal.pose.position.y = goal_pos[1]
            goal.pose.orientation.w = 1.0

            resp = make_plan(start=start, goal=goal, tolerance=tolerance)
            path = resp.plan

            plan_points = []
            for pose in path.poses:
                plan_points.append([pose.pose.position.x, pose.pose.position.y])

            #rospy.loginfo(f"Global plan created with {len(plan_points)} points.")

            return np.array(plan_points)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return None    


    def publish_global_plan(self, plan_array):
        """
        plan_array: np.array of shape (N, 2) representing x, y path points
        """
        if plan_array is None or len(plan_array) == 0:
            return

        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "odom"

        for pt in plan_array:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.orientation.w = 1.0  # 방향 설정 없음
            path_msg.poses.append(pose)

        self.plan_pub.publish(path_msg)
        
    def step(self, action):
        #self.nav_as.send_goal(self.mb_goal)
        target = False
        if self.timestep % 10 == 0:  # 매 10스텝마다
            start = [self.init_position[0], self.init_position[1]]
            goal = [self.init_position[0] + self.goal_position[0], self.init_position[1] + self.goal_position[1]]
            self.global_plan = self.get_global_plan(start, goal)
            self.publish_global_plan(self.global_plan)

        # Publish the robot action
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.linear.y = action[1]
        vel_cmd.angular.z = action[2]
        self.vel_pub.publish(vel_cmd) #publish vel_cmd
        #print(f"ACTION X: {action[0]:.2f}, Y: {action[1]:.2f}, Z: {action[2]:.2f} ")

        self.gazebo_sim.unpause()
        time.sleep(TIME_DELTA)  # 액션이 적용될 시간 동안 대기
        self.gazebo_sim.pause()

        done, collision = self.observe_collision()
        l_state = []
        l_state = self.sensor_data[:]
        lidar_state = [l_state]

        # Calculate robot heading from odometry data
        #self.odom_x = self.last_odom.pose.pose.position.x
        #self.odom_y = self.last_odom.pose.pose.position.y

        pos = self.gazebo_sim.get_model_state().pose.position
        orientation = self.gazebo_sim.get_model_state().pose.orientation
        
        #print(f"POS X: {pos.x:.2f}, Y: {pos.y:.2f} : ODM X: {self.odom_x:.2f}, Y: {self.odom_y:.2f}")

        distance = np.linalg.norm(
            [pos.x - self.goal_position[0], pos.y - self.goal_position[1]]
        )

        # Calculate the relative angle between the robots heading and heading toward the goal
        quaternion = (
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        angle = round(euler[2], 4)
        
        skew_x = self.goal_position[0] - pos.x
        skew_y = self.goal_position[1] - pos.y
        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle
        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        # Detect if the goal has been reached and give a large positive reward
        if distance < GOAL_REACHED_DIST:
            target = True
            done = True

        global_plan = self.global_plan
        
        robot_state = [distance, theta, action[0], action[1], action[2]]
        state = np.append(robot_state, lidar_state)
        reward = self.get_reward(target, collision, distance, action, global_plan)
        self.pre_distance = distance
        self.timestep += 1

        return state, reward, done, target

    def reset(self):
        self.gazebo_sim.reset_init_model_state(self.init_position)
        self.gazebo_sim.reset()  # Gazebo에서 로봇을 해당 위치로 이동

        self.gazebo_sim.unpause()
        time.sleep(TIME_DELTA)
        self.gazebo_sim.pause()
        
        pos = self.gazebo_sim.get_model_state().pose.position
        distance = np.linalg.norm(
            [pos.x - self.goal_position[0], pos.y - self.goal_position[1]]
        )

        # Calculate the relative angle between the robots heading and heading toward the goal
        orientation = self.gazebo_sim.get_model_state().pose.orientation
        quaternion = (
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        angle = round(euler[2], 4)
        
        skew_x = self.goal_position[0] - pos.x
        skew_y = self.goal_position[1] - pos.y
        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle
        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        l_state = []
        l_state = self.sensor_data[:]
        lidar_state = [l_state]

        robot_state = [distance, theta, 0.0, 0.0, 0.0]
        state = np.append(robot_state, lidar_state)
        self.timestep = 0
        self.last_distance = None
        self.left_path = False
        self.last_path_idx = None
        
        return state

    def terminate(self):
        """Gazebo 환경 종료"""
        kill_cmd = f"rosnode kill /odom_tf_broadcaster"
        os.system(kill_cmd)
        time.sleep(2)  
        self.gazebo_process.terminate()
        time.sleep(2)  
        os.system("killall -9 gzserver roslaunch rosout gzclient")
        time.sleep(2)
        

    def observe_collision(self):

        if np.min(self.sensor_data) < 0.05:
            rospy.logwarn("Collision detected: Sensor based")
            self.min_distance = 1000.0
            return True, True
        
        if self.gazebo_sim.get_hard_collision():
            rospy.logwarn("Collision detected: Gazebo based")
            self.min_distance = 1000.0
            return True, True

        # 충돌이 없을 경우
        return False, False

    def get_reward(self, target, collision, distance, action, global_plan):
        reward = 0.0
        r1 = 1.0
        r2 = 100.0
        r3 = 5.0
        curr_idx = None
        
        # --- robot position ---
        pos = self.gazebo_sim.get_model_state().pose.position
        robot_pos = np.array([pos.x, pos.y])

        #rospy.loginfo(f"[Robot Position] x: {pos.x:.2f}, y: {pos.y:.2f}")


        # --- 1. goal reach ---

        if target: # reach goal
            reward += 100.0

        # --- 2. collision detect ---

        if collision:
            rospy.loginfo("Collision!")
            return -r1 - r2

        # --- 3. following global plan --- 
        on_path = False
        dist_threshold = 0.05 

        if global_plan is not None and len(global_plan) > 0:
            dists = np.linalg.norm(global_plan - robot_pos, axis=1)
            min_dist = np.min(dists)
            curr_idx = int(np.argmin(dists))
            if min_dist < dist_threshold:
                on_path = True

        # --- path 상태에 따른 reward ---
        # if on_path:
        #     if hasattr(self, "left_path") and self.left_path:
        #         # 복귀했을 때 last index에서 얼마나 이동했는지
        #         if hasattr(self, "last_path_idx") and self.last_path_idx is not None:
        #             Ne = self.last_path_idx - curr_idx  # index 차이
        #             reward += -r1 + Ne * r3
        #             rospy.loginfo(f"[REJOIN PATH] From idx {self.last_path_idx} → {curr_idx}, Ne={Ne}, reward += {-r1 + Ne * r3}")
        #         else: 
        #             reward += 0 # fallback
        #         self.left_path = False
        #     else:# 원래부터 path 위에 있었던 경우
        #         reward += r1  # 정상적으로 path 위에 있음
        #     self.last_path_idx = curr_idx
        # else:
        #     reward += r1
        #     if not hasattr(self, "left_path") or not self.left_path:
        #         self.left_path = True
        #         self.last_path_idx = curr_idx  # path 벗어나기 전 index 저장

        # --- time penalty ---   
        reward -= self.timestep

        # --- global plan length reward ---
        if global_plan is not None and len(global_plan) > 0:
            if global_plan.ndim == 1:
                global_plan = global_plan.reshape(-1, 2)

            # 가장 가까운 점 인덱스
            dists = np.linalg.norm(global_plan - robot_pos, axis=1)
            closest_idx = np.argmin(dists)

            # 남은 경로 길이 계산
            distance_to_goal = len(global_plan[closest_idx:])

        else:
            # using euclidian distance
            distance_to_goal = np.linalg.norm([pos.x - self.goal_position[0], pos.y - self.goal_position[1]])

        if not hasattr(self, "last_distance") or self.last_distance is None:
            self.last_distance = distance_to_goal

        if self.last_distance > distance_to_goal:
            reward += (self.last_distance - distance_to_goal) * 100
            #rospy.loginfo(f"[Reward] LastDist: {self.last_distance:.2f} , CurrDistance {distance_to_goal:.2f}")
            
        else:
            if self.last_distance - distance_to_goal == 0:
                reward -= distance_to_goal * 0.01
            else:
                reward += (self.last_distance - distance_to_goal) * 100
            #rospy.loginfo(f"[Reward] LastDist: {self.last_distance:.2f} , CurrDistance {distance_to_goal:.2f}")

        self.last_distance = distance_to_goal

        # --- penalty of following global plan ---
        penalty = dists[closest_idx] * r1
        reward -= penalty

        # # distance to goal 
        # alpha = 10   # 최대 보상/페널티 크기
        # k = 10.0         # 전환의 급격함 (k값이 클수록 전환이 더 급격해짐)
        # threshold = 2.5  # 전환 임계 거리

        # # tanh 함수: threshold 보다 크면 음수, 작으면 양수가 나오도록 함.
        # reward += alpha * np.tanh(k * (threshold - distance))
        # rospy.loginfo(f"distance to goal : {distance}")

        # # translation than rotation
        # if np.min(self.sensor_data) > 0.5:
        #     reward += (action[0]**2 + action[1]**2)**0.5 / 2 - abs(action[2]) / 2
        

        return reward
