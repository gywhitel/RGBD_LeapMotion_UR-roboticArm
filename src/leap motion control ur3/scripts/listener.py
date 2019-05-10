#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray

def callback(pose):
    rospy.loginfo('I heard [%f]',pose.data[0])



if __name__ == '__main__':
    print"start"
    rospy.init_node('listeer', anonymous=True)
    rospy.Subscriber('finger_direction', Float32MultiArray, callback)
    rospy.spin()
