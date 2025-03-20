import os
import time
import math
import numpy as np
import subprocess
import rospy
import argparse
import rospkg

from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid  
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist, Quaternion
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import actionlib
import laser_geometry.laser_geometry as LaserGeometry
import sensor_msgs.point_cloud2 as pc2

from std_srvs.srv import Empty
from os.path import join

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

        self.init_global_guide()

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
        self.vel_pub = rospy.Publisher("jackal_velocity_controller/cmd_vel", Twist, queue_size=1)
        self.odom = rospy.Subscriber("/jackal_velocity_controller/odom", Odometry, self.odom_callback, queue_size=1)
        #self.costmap = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.costmap_callback, queue_size=1)
        self.sensor = rospy.Subscriber("/front/scan", LaserScan, self.lidar_callback, queue_size=1)
        self.sensor = rospy.Subscriber("/front/scan", LaserScan, self.lidar_callback, queue_size=1)

    def init_global_guide(self):
         # ROS 패키지 경로 설정
        rospack = rospkg.RosPack()
        base_path = rospack.get_path('leebots_local_planner')

        launch_file = os.path.join(base_path, 'launch', 'move_base_global.launch')

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

    def costmap_callback(self, msg):
        #rospy.loginfo("Received Costmap data!")
        # Costmap 데이터를 numpy 배열로 변환
        self.costmap_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.costmap_resolution = msg.info.resolution
        self.costmap_origin_x = msg.info.origin.position.x
        self.costmap_origin_y = msg.info.origin.position.y
        self.costmap_width = msg.info.width
        self.costmap_height = msg.info.height

        # 디버깅용 출력
        rospy.loginfo(f"Costmap received: {self.costmap_width}x{self.costmap_height}")

    # Perform an action and read a new state
    def step(self, action):
        target = False

        # Publish the robot action
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.linear.y = action[1]
        vel_cmd.angular.z = action[2]
        self.vel_pub.publish(vel_cmd) #publish vel_cmd

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
        quaternion = Quaternion(
            self.last_odom.pose.pose.orientation.w,
            self.last_odom.pose.pose.orientation.x,
            self.last_odom.pose.pose.orientation.y,
            self.last_odom.pose.pose.orientation.z,
        )

        # Calculate distance to the goal from the robot
        distance = np.linalg.norm(
            [self.odom_x - self.goal_position[0], self.odom_y - self.goal_position[1]]
        )

        # Detect if the goal has been reached and give a large positive reward
        if distance < GOAL_REACHED_DIST:
            target = True
            done = True

        robot_state = [distance, action[0], action[1], action[2]]
        state = np.append(robot_state, lidar_state)
        reward = self.get_reward(target, collision, action)
        return state, reward, done, target

    def reset(self):
        self.gazebo_sim.reset_init_model_state(self.init_position)
        self.gazebo_sim.reset()  # Gazebo에서 로봇을 해당 위치로 이동

        model_state = self.gazebo_sim.get_model_state()
        self.odom_x = model_state.pose.position.x
        self.odom_y = model_state.pose.position.y

        self.gazebo_sim.unpause()
        time.sleep(TIME_DELTA)
        self.gazebo_sim.pause()

        distance = np.linalg.norm([self.odom_x - self.goal_position[0], self.odom_y - self.goal_position[1]])
        
        l_state = []
        l_state = self.sensor_data[:]
        lidar_state = [l_state]

        robot_state = [distance, 0.0, 0.0, 0.0]
        state = np.append(robot_state, lidar_state)
        
        return state

    def terminate(self):
        """Gazebo 환경 종료"""
        self.gazebo_process.terminate()

    def observe_collision(self):
        # 현재 로봇 위치 가져오기
        robot_x = self.odom_x  # 현재 x 좌표
        robot_y = self.odom_y  # 현재 y 좌표

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

        #if np.min(self.sensor_data) < 0.2:
        #    rospy.logwarn("Collision detected: Robot is inside an obstacle!")
        #    return True, True
        
        if self.gazebo_sim.get_hard_collision():
            rospy.logwarn("Collision detected: Robot is inside an obstacle!")
            return True, True

        # 충돌이 없을 경우
        return False, False

    def get_reward(self, target, collision, action):
        if target:
            return 100.0
        elif collision:
            return -100.0
        else: #have to change 
            r3 = lambda x: 1 - x if x < 1 else 0.0
            return action[0] / 2 - abs(action[1]) / 2 