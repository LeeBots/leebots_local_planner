import rospy
import sys
import numpy as np

import tf2_ros
from gazebo_simulation import GazeboSimulation
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

def publish_odom_tf(gazebo, odom_broad, odom_pub):
    current_time = rospy.get_rostime()
    pos = gazebo.get_model_state().pose.position
    orientation = gazebo.get_model_state().pose.orientation
    twist_l = gazebo.get_model_state().twist.linear
    twist_a = gazebo.get_model_state().twist.angular

    print(f"CUR: {pos}, TIME: {current_time}")

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

    odom_broad.sendTransform(odom_trans)
    
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

    odom_pub.publish(cur_odom)

if __name__ == '__main__':
    rospy.init_node('odom_tf_broadcaster')
    
    gazebo_sim = GazeboSimulation()
    odom_pub = rospy.Publisher("fake/odom", Odometry, queue_size=10)
    odom_broadcaster = tf2_ros.TransformBroadcaster()
    
    # start_pose = [0, 0]
    # if sys.argv[1] == "dynamic":
    #     start_pose = [11, 0, 3.14]
    # else:
    #     start_pose = [-2.25, 3, 1.57]
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        publish_odom_tf(gazebo_sim, odom_broadcaster, odom_pub)
        r.sleep()

        
