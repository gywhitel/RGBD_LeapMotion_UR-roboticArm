#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

if __name__=='__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('add_box1', anonymous=True)

    scene = moveit_commander.PlanningSceneInterface()

    box_pose = geometry_msgs.msg.PoseStamped() 
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.0
    box_pose.pose.position.y = 0.75
    box_pose.pose.position.z = 0.536943

    box_name = "box1"
    scene.add_box(box_name, box_pose, size=(2, 0.01, 2))
    rospy.sleep(10)