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

def callback(direction):
    Lstrength   = direction.data[0]
    pitch       = direction.data[1]
    yaw         = direction.data[2]  
    rospy.loginfo(' S,P,Y,[%f],[%f],[%f] ',direction.data[0],direction.data[1],direction.data[2])

    joint_goal = group.get_current_joint_values()
    '''
    Left  1 Right [] 
    pich    joint 4 
    ywa     joint 5
    topic 1 angle 2
    '''
    if(pitch > 0.3): 
        joint_goal[4] += pi/180     
        group.go(joint_goal , wait=True)
        group.clear_pose_targets()

    if(pitch < -0.15): 
        joint_goal[4] -= pi/180      
        group.go(joint_goal , wait=True)
        group.clear_pose_targets()
    
    if(yaw > 0.2):
        joint_goal[5] += pi/180    
        group.go(joint_goal , wait=True)
        group.clear_pose_targets()

    if(yaw < -0.3):
        joint_goal[5] -= pi/180      
        group.go(joint_goal , wait=True)
        group.clear_pose_targets()    

    


if __name__=='__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur3_move_direction',
                  anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")

    print"start"
    rospy.sleep(1)
    rospy.init_node('ur3_move_direction', anonymous=True)
    rospy.Subscriber('hand_direction', Float32MultiArray, callback,queue_size=1)
    rospy.spin()

    moveit_commander.roscpp_shutdown()
    print "============ STOPPING"
