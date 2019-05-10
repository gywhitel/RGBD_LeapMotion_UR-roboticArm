#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

from std_msgs.msg import String

    
    

if __name__=='__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur3_move',
                  anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")

    
    
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = pi
    joint_goal[1] = -pi/2.0
    joint_goal[2] = pi/3
    joint_goal[3] = -pi/3
    joint_goal[4] = -pi/2
    joint_goal[5] = 0
    group.go(joint_goal, wait=True)
    group.clear_pose_targets()
    print("OUT joint")

    end_link=group.get_end_effector_link()
    end_pose=group.get_current_pose(end_effector_link =end_link)
    print"====="
    print end_pose.pose.position
    print"====="
    print group.get_current_pose()
    
    rospy.sleep(2)

    '''
    for i in range(11):
        joint_goal[0]=-i*pi/20
        joint_goal[1]=-pi/2+i*pi/50
        joint_goal[2]=pi/3+i*pi/30
        joint_goal[3]=-pi/3-i*pi/20
        group.go(joint_goal, wait=True)
        group.clear_pose_targets()
        #rospy.sleep(1)
    '''
    group.clear_pose_targets()
    pose_target_1 = geometry_msgs.msg.Pose()
    pose_target_1.position.x=-0.26
    pose_target_1.position.y=0.011
    pose_target_1.position.z=0.42
    pose_target_1.orientation.w = 0.707  
    pose_target_1.orientation.x = 0.0
    pose_target_1.orientation.y = 0.707
    pose_target_1.orientation.z = 0.0

    group.set_pose_target(pose_target_1)
    group.go(pose_target_1,wait=True)
    rospy.sleep(2)
    group.clear_pose_targets()

    pose_target_2 = geometry_msgs.msg.Pose()
    pose_target_2.position.x=-0.26
    pose_target_2.position.y=-0.021
    pose_target_2.position.z=0.32
    pose_target_2.orientation.w=0.707
    pose_target_2.orientation.x=0.0
    pose_target_2.orientation.y=0.707
    pose_target_2.orientation.z=0.0

    group.set_pose_target(pose_target_2)
    group.go(pose_target_2,wait=True)
    rospy.sleep(2)
    group.clear_pose_targets()
    '''
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w=1.0
    for i in range(4):        
        pose_target.position.x=pose_target_1.position.x+i*(pose_target_2.position.x-pose_target_1.position.x)/4
        pose_target.position.y=pose_target_1.position.y+i*(pose_target_2.position.y-pose_target_1.position.y)/4
        pose_target.position.z=pose_target_1.position.z+i*(pose_target_2.position.z-pose_target_1.position.z)/4
        group.set_pose_target(pose_target)
        group.go(pose_target,wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(2)

    rospy.loginfo('Arrive goal')
    group.set_pose_target(pose_target_2)
    group.go(pose_target_2,wait=True)
    '''

    moveit_commander.roscpp_shutdown()
    print "============ STOPPING"
