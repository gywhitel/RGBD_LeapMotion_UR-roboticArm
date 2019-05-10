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

pose_target = geometry_msgs.msg.Pose()
def callback(pose):
    flag=1
    y=-pose.data[2]
    x=pose.data[0]
    z=pose.data[1]
    if x==0 and y==0 and z==0:
        flag=0
    if z<80:
        if x>-60 or x<60:
            if y>-60 or y<60:
                flag=0

    if flag==1:
        x=1.0*x/1000.0+0.05
        y=1.0*y/1000.0-0.05
        z=1.0*z/1000.0+0.18
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z
        
        pose_target.orientation.w=0.707113427592
        pose_target.orientation.x=6.99357969762e-06
        pose_target.orientation.y=0.707100134595
        pose_target.orientation.z=-1.12406020568e-05

    #Get current ee_link Pose
    start_pose = group.get_current_pose(end_effector_link ="ee_link").pose
    if pose_target.position == start_pose:
        flag=0

    if flag==1:
        rospy.loginfo(' x,y,z,[%f],[%f],[%f]',pose_target.position.x,pose_target.position.y,pose_target.position.z)
        waypoints = []
        waypoints.append(start_pose)
        #Append target Pose
        waypoints.append(copy.deepcopy(pose_target))
        #Plan the point And go
        (plan,fraction) = group.compute_cartesian_path(waypoints,0.01,0.0,True,0)
        rospy.loginfo('Visualing (%.2f % % acheived)',fraction * 100.0)
        group.execute(plan)
        group.clear_pose_targets()


def callback2(finger):
    Finger = finger.data[0]
    state = "OPEN" if Finger >= 1 else "CLOSE"
    rospy.loginfo(' Gripper State is :  %s  The value is %.2f ',state,Finger)
    rospy.sleep(1)


if __name__=='__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur3_move',
                  anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = pi/2.0
    joint_goal[1] = -pi/2
    joint_goal[2] = pi/3
    joint_goal[3] = -pi/3
    joint_goal[4] = -pi/2
    joint_goal[5] = 0
    group.go(joint_goal, wait=True)
    group.clear_pose_targets()
    
    print"start"
    rospy.sleep(1)
    rospy.init_node('ur3_move', anonymous=True)
    rospy.Subscriber('hand_postion', Float32MultiArray, callback,queue_size=1)
    rospy.Subscriber('finger_direction', Float32MultiArray, callback2,queue_size=1)
    
    rospy.spin()

    moveit_commander.roscpp_shutdown()
    print "============ STOPPING"
