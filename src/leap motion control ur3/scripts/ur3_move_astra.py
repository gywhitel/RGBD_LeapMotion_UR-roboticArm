#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import math  
from geometry_msgs.msg import Vector3

pose_target = geometry_msgs.msg.Pose()


def callback3(hand_direction):
    direction_x = hand_direction.x/50.0
    direction_y = hand_direction.z/50.0
    direction_z = hand_direction.y/50.0
    rospy.loginfo('Move x,y,z: %.2f %.2f %.2f',direction_x,direction_y,direction_z)

    start_pose = group.get_current_pose(end_effector_link ="ee_link").pose
    pose_target_2 = geometry_msgs.msg.Pose()
    pose_target_2.position.x = start_pose.position.x + direction_x
    pose_target_2.position.y = start_pose.position.y + direction_y
    pose_target_2.position.z = start_pose.position.z + direction_z
    pose_target_2.orientation.w = 0.707113427592
    pose_target_2.orientation.x = 6.99357969762e-06
    pose_target_2.orientation.y = 0.707100134595
    pose_target_2.orientation.z = -1.12406020568e-05
    
    waypoints = []
    waypoints.append(start_pose)
    waypoints.append(copy.deepcopy(pose_target_2))
    (plan,fraction) = group.compute_cartesian_path(waypoints,0.01,0.0,True,0)
    group.execute(plan)
    group.clear_pose_targets()




if __name__=='__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur3_move_astra',anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = -pi/2.0
    joint_goal[1] = -pi/2.0
    joint_goal[2] = pi/3.0
    joint_goal[3] = -pi/3.0
    joint_goal[4] = -pi/2.0
    joint_goal[5] = 0
    group.set_joint_value_target(joint_goal)
    #plan = group.plan()
    #group.execute(plan)
    group.go(joint_goal, wait=True)
    group.clear_pose_targets()
    
    print "start"
    # Move robot postion small (1CM)
    rospy.Subscriber('RightHand_direction',Vector3, callback3,queue_size=1)    

    rospy.spin()
    moveit_commander.roscpp_shutdown()
    print "============ STOPPING"
