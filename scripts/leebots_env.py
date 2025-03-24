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

        self.gaps = [[-np.pi / 2 - 0.03, -np.pi / 2 + np.pi / self.environment_dim]]
        for m in range(self.environment_dim - 1):
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim]
            )
        self.gaps[-1][-1] += 0.03

        # Gazebo 환경 로드
        self.init_gazebo()

        # # ROS 퍼블리셔 및 서브스크라이버 설정
        self.init_ros()

        #self.publish_odom_tf()

        self.init_global_guide()

        # self.initial_costmap_saved = False
        # self.costmap_file_path = os.path.expanduser("~/barn_ws/jackal_ws/src/global_planner/data/costmap0005.txt")
        # self.costmap_info_path = os.path.expanduser("~/barn_ws/jackal_ws/src/global_planner/data/cminfo0005.txt")
        import actionlib
        from geometry_msgs.msg import Quaternion
        from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
        self.nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.mb_goal = MoveBaseGoal()
        self.mb_goal.target_pose.header.frame_id = 'odom'
        self.mb_goal.target_pose.pose.position.x = self.goal_position[0] + self.init_position[0]
        self.mb_goal.target_pose.pose.position.y = self.goal_position[1] + self.init_position[1]
        self.mb_goal.target_pose.pose.position.z = 0
        self.mb_goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)

        self.nav_as.wait_for_server()
        

    def init_gazebo(self):
        os.environ["JACKAL_LASER"] = "1"
        os.environ["JACKAL_LASER_MODEL"] = "ust10"
        os.environ["JACKAL_LASER_OFFSET"] = "-0.065 0 0.01"
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

        print(f"Loading Gazebo Simulation with {self.world_name}")

        # ROS 패키지 경로 설정
        rospack = rospkg.RosPack()
        base_path = rospack.get_path('leebots_local_planner')
        os.environ['GAZEBO_PLUGIN_PATH'] = os.path.join(base_path, "plugins")

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

        rospy.init_node('gym', anonymous=True) #, log_level=rospy.FATAL)
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
        self.vel_pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size=1)
        self.odom = rospy.Subscriber("/jackal_velocity_controller/odom", Odometry, self.odom_callback, queue_size=1)
        self.sensor = rospy.Subscriber("/front/scan", LaserScan, self.lidar_callback, queue_size=1)
        self.global_plan_sub = rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.global_plan_callback, queue_size=10)
        self.set_state = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=10)
        self.odom_pub = rospy.Publisher("fake/odom", Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

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

    # def initial_costmap_callback(self, msg):
    #     if self.initial_costmap_saved:
    #         return
    #     self.initial_costmap_saved = True

    #     costmap_array = np.array(msg.data).reshape(msg.info.height, msg.info.width)
    #     np.savetxt(self.costmap_file_path, costmap_array, fmt="%d")
    #     rospy.loginfo("Initial costmap saved to file.")

    #     self.robot_xw = (self.init_position[0] - msg.info.origin.position.x) / msg.info.resolution
    #     self.robot_xy = (self.init_position[1] - msg.info.origin.position.y) / msg.info.resolution

    #     with open(self.costmap_info_path, "w") as f:
    #         f.write(
    #             f"{msg.info.width} {msg.info.height} "
    #             f"{msg.info.origin.position.x} {msg.info.origin.position.y} "
    #             f"{msg.info.resolution} f{self.robot_xw} f{self.robot_xy}\n"
    #         )
    #     rospy.loginfo("Costmap metadata saved to file.")
        
    #     self.costmap_data = costmap_array
    #     self.costmap_width = msg.info.width
    #     self.costmap_height = msg.info.height
    #     self.costmap_resolution = msg.info.resolution
    #     self.costmap_origin_x = msg.info.origin.position.x
    #     self.costmap_origin_y = msg.info.origin.position.y

    # def costmap_update_callback(self, msg):
    #     if not hasattr(self, 'costmap_data'):
    #         return  # 아직 초기 costmap을 못 받은 경우

    #     for y in range(msg.height):
    #         for x in range(msg.width):
    #             index = y * msg.width + x
    #             value = msg.data[index]
    #             global_x = msg.x + x
    #             global_y = msg.y + y

    #             if 0 <= global_y < self.costmap_data.shape[0] and 0 <= global_x < self.costmap_data.shape[1]:
    #                 self.costmap_data[global_y][global_x] = value

    #     # 업데이트된 costmap 저장
    #     np.savetxt(self.costmap_file_path, self.costmap_data, fmt="%d")
    #     rospy.loginfo("Costmap updated and saved.")

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

        rospy.loginfo(f"Received global plan with {len(global_plan)} points")

    # def costmap_callback(self, msg):
    #     #rospy.loginfo("Received Costmap data!")
    #     # Costmap 데이터를 numpy 배열로 변환
    #     self.costmap_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
    #     self.costmap_resolution = msg.info.resolution
    #     self.costmap_origin_x = msg.info.origin.position.x
    #     self.costmap_origin_y = msg.info.origin.position.y
    #     self.costmap_width = msg.info.width
    #     self.costmap_height = msg.info.height

    #     # 디버깅용 출력
    #     rospy.loginfo(f"Costmap received: {self.costmap_width}x{self.costmap_height}")

    # def load_plan(self):
    #     planner_dir = os.path.expanduser("~/barn_ws/jackal_ws/src/global_planner/bin")
    #     config_file = "config.txt"
    #     output_file = os.path.join(planner_dir, "outplan.txt")

    #     cmd = f"cd {planner_dir} && ./single_plan_run {config_file}"

    #     process = subprocess.run(cmd, shell=True, capture_output=True, text=True)

    #     if process.returncode != 0:
    #         rospy.logwarn("Global planner 실행 실패:")
    #         rospy.logwarn(process.stderr)
    #         return None

    #     try:
    #         global_plan = np.loadtxt(output_file)
    #         rospy.loginfo("Global plan loaded")
    #         return global_plan
    #     except Exception as e:
    #         rospy.logwarn(f"Global plan 파일 읽기 실패: {e}")
    #         return None

    # def convert_plan(self, grid_plan, resolution, origin_x, origin_y):

    #     # grid index to world coordinate
    #     world_plan = np.zeros_like(grid_plan, dtype=np.float32)
    #     world_plan[:, 0] = origin_x + grid_plan[:, 1] * resolution  # x = origin_x + j * res
    #     world_plan[:, 1] = origin_y + grid_plan[:, 0] * resolution  # y = origin_y + i * res
    #     return world_plan
    
    # Perform an action and read a new state
        
    def publish_odom_tf(self):
        current_time = rospy.get_rostime()
        pos = self.gazebo_sim.get_model_state().pose.position
        orientation = self.gazebo_sim.get_model_state().pose.orientation
        twist_l = self.gazebo_sim.get_model_state().twist.linear
        twist_a = self.gazebo_sim.get_model_state().twist.angular

        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"

        odom_trans.transform.translation.x = pos.x
        odom_trans.transform.translation.y = pos.y
        odom_trans.transform.translation.z = pos.z
        odom_trans.transform.rotation.x = orientation.x
        odom_trans.transform.rotation.y = orientation.y
        odom_trans.transform.rotation.z = orientation.z
        odom_trans.transform.rotation.w = orientation.w

        self.odom_broadcaster.sendTransform((pos.x,pos.y,pos.z), (orientation.x, orientation.y, orientation.z, orientation.w), current_time, "base_link", "odom")
        
        cur_odom = Odometry()
        cur_odom.header.frame_id = "odom"
        cur_odom.header.stamp = current_time

        cur_odom.pose.pose.position.x = pos.x
        cur_odom.pose.pose.position.y = pos.y
        cur_odom.pose.pose.position.z = pos.z
        cur_odom.pose.pose.orientation.x = orientation.x
        cur_odom.pose.pose.orientation.y = orientation.y
        cur_odom.pose.pose.orientation.z = orientation.z
        cur_odom.pose.pose.orientation.w = orientation.w

        cur_odom.child_frame_id = "base_link"
        cur_odom.twist.twist.linear.x = twist_l.x
        cur_odom.twist.twist.linear.x = twist_l.y
        cur_odom.twist.twist.angular.z = twist_a.z

        self.odom_pub.publish(cur_odom)

        
    def step(self, action):
        self.nav_as.send_goal(self.mb_goal)
        target = False

        # Publish the robot action
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.linear.y = action[1]
        vel_cmd.angular.z = action[2]
        self.vel_pub.publish(vel_cmd) #publish vel_cmd

        self.publish_odom_tf()

        self.gazebo_sim.unpause()
        time.sleep(TIME_DELTA)  # 액션이 적용될 시간 동안 대기
        self.gazebo_sim.pause()

        done, collision = self.observe_collision()
        l_state = []
        l_state = self.sensor_data[:]
        lidar_state = [l_state]

        # Calculate robot heading from odometry data
        self.odom_x = self.last_odom.pose.pose.position.x
        self.odom_y = self.last_odom.pose.pose.position.y

        pos = self.gazebo_sim.get_model_state().pose.position
        orientation = self.gazebo_sim.get_model_state().pose.orientation
        
        print(f"POS X: {pos.x:.2f}, Y: {pos.y:.2f} : ODM X: {self.odom_x:.2f}, Y: {self.odom_y:.2f}")

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
        
        # global_plan = self.load_plan()
        # global_plan = self.convert_plan(global_plan, self.costmap_resolution, self.costmap_origin_x, self.costmap_origin_y)

        global_plan = self.global_plan
        
        robot_state = [distance, theta, action[0], action[1], action[2]]
        state = np.append(robot_state, lidar_state)
        reward = self.get_reward(target, collision, distance, action, global_plan)
        self.pre_distance = distance

        return state, reward, done, target

    def reset(self):
        self.gazebo_sim.reset_init_model_state(self.init_position)
        self.gazebo_sim.reset()  # Gazebo에서 로봇을 해당 위치로 이동

        model_state = self.gazebo_sim.get_model_state()
        #self.odom_x = model_state.pose.position.x
        #self.odom_y = model_state.pose.position.y

        self.gazebo_sim.unpause()
        time.sleep(TIME_DELTA)
        self.gazebo_sim.pause()

        #distance = np.linalg.norm([self.odom_x - self.goal_position[0], self.odom_y - self.goal_position[1]])
        
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
        
        return state

    def terminate(self):
        """Gazebo 환경 종료"""
        self.gazebo_process.terminate()

    def observe_collision(self):
        # 현재 로봇 위치 가져오기
        #robot_x = self.odom_x  # 현재 x 좌표
        #robot_y = self.odom_y  # 현재 y 좌표

        # Costmap 정보 가져오기
        #costmap = self.costmap_data
        #costmap_resolution = self.costmap_resolution  # m/cell (해상도)
        #costmap_origin_x = self.costmap_origin_x  # costmap의 원점 x 좌표
        #costmap_origin_y = self.costmap_origin_y  # costmap의 원점 y 좌표
        #costmap_width = self.costmap_width  # costmap의 가로 크기 (셀 단위)
        #costmap_height = self.costmap_height  # costmap의 세로 크기 (셀 단위)

        # 로봇 위치를 Costmap 좌표계로 변환
        #costmap_x = int((robot_x - costmap_origin_x) / costmap_resolution)
        #costmap_y = int((robot_y - costmap_origin_y) / costmap_resolution)

        # Costmap 범위 안에 있는지 확인
        #if costmap_x < 0 or costmap_x >= costmap_width or costmap_y < 0 or costmap_y >= costmap_height:
        #    rospy.logwarn("Robot is out of costmap bounds!")
        #    return False, False

        # 로봇이 위치한 grid의 값 확인 (장애물 여부)
        #obstacle_threshold = 50  # 장애물로 간주하는 점유 확률 값 (0~100 중에서)
        #robot_cell_value = costmap[costmap_y, costmap_x]  # costmap은 (y, x) 순서로 접근

        # 로봇이 장애물 위에 있는 경우 → 충돌로 판단
        #if robot_cell_value >= obstacle_threshold:
        #    rospy.logwarn("Collision detected: Robot is inside an obstacle!")
        #    return True, True

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
        min_dist_to_path = None
        # --- robot position ---
        pos = self.gazebo_sim.get_model_state().pose.position
        robot_pos = np.array([pos.x, pos.y])

        rospy.loginfo(f"[Robot Position] x: {pos.x:.2f}, y: {pos.y:.2f}")

        # --- global plan exists ---
        if global_plan is not None and len(global_plan) > 0:
            if global_plan.ndim == 1:
                global_plan = global_plan.reshape(-1, 2)

            # 가장 가까운 점 인덱스
            dists = np.linalg.norm(global_plan - robot_pos, axis=1)
            closest_idx = np.argmin(dists)

            # 남은 경로 길이 계산
            remaining_plan = global_plan[closest_idx:]
            if len(remaining_plan) >= 1:
                diffs = np.diff(remaining_plan, axis=0)
                segment_lengths = np.linalg.norm(diffs, axis=1)
                distance_to_goal = np.sum(segment_lengths)
            else:
                distance_to_goal = 0.0
        else:
            # using euclidian distance
            distance_to_goal = np.linalg.norm([pos.x - self.goal_position[0], pos.y - self.goal_position[1]])

        # --- reward shaping ---
        if self.min_distance > distance_to_goal:
            reward += 30 * (1 / (self.min_distance - distance_to_goal))
            rospy.loginfo(f"[Reward] LastMinDist: {self.min_distance:.2f} , NewMinDistance {distance_to_goal:.2f}")
            self.min_distance = distance_to_goal
        else:
            reward += self.min_distance - distance_to_goal
            rospy.loginfo(f"[Reward] MinDist: {self.min_distance:.2f} , DistancetoGoal : {distance_to_goal:.2f}")

        # # --- global plan proximity reward ---
        # if np.min(dists) is not None:
        #     min_dist_to_path = np.min(dists)
        #     if min_dist_to_path < 0.1:
        #         reward += 10.0
        #     else:
        #         reward -= 2.0
        #     rospy.loginfo(f"[Reward] MinDisttoPath: {min_dist_to_path:.2f}")
        # #print(f"MIN_DIST: {self.min_distance:.2f}, CUR_DIST: {distance:.2f}; {self.min_distance > distance}")
        # if self.min_distance > distance:
        #     reward += 10 #(self.min_distance - distance)**2
        #     self.min_distance = distance
        # else:
        #     reward += self.min_distance - distance
        
        if target:
            reward += 100.0
        elif collision:
            reward -= 50.0

        # translation than rotation
        if np.min(self.sensor_data) > 0.5:
            reward += (action[0]**2 + action[1]**2)**0.5 / 2 - abs(action[2]) / 2
        
        # if action[0] > 0 and action[1] > 0:
        #     reward += 5
            
        #print(f"REWARD: {reward:.2f}")
        return reward
