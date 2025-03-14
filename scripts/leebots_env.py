import time
import numpy as np
import rospy
import actionlib
import subprocess
import os
from os.path import join

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


GOAL_REACHED_DIST = 0.1
COLLISION_DIST = 0.1
TIME_DELTA = 0.1

class GazeboEnv:

    def __init__(self, world_idx=0):
        self.world_idx = world_idx
        self.init_position, self.goal_position, self.world_name = self._get_world_info(world_idx)


        # Run Gazebo simulation
        self._launch_gazebo(self.world_name)

        # init ROS 
        rospy.init_node("gazebo_leebots_env", anonymous=True)
        rospy.set_param("/use_sim_time", True)
        
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)       
