
#ifndef _LEAPROS_H
#define _LEAPROS_H

#include <ros/ros.h>
#include <Leap.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

class RosListener:public Leap::Listener{
  public:
    
    //RosListener();
    virtual void onInit(const Leap::Controller& );
    virtual void onConnect(const Leap::Controller& );
    virtual void onDisconnect(const Leap::Controller& );
    virtual void onExit(const Leap::Controller& );
    virtual void onFrame(const Leap::Controller& );

    //ros::NodeHandle nh_;
    //ros::Publisher vel_pub; //topic
    geometry_msgs::Vector3 teleop;   //设置消息//[0,1,2]=[Lx,Ly,Lz]
    geometry_msgs::Vector3  teleop_1; //设置消息
    std_msgs::Float32 TeleopJaw; //设置消息
    unsigned int FlagLeft;
    unsigned int FlagRight;
    double FingerAngel;
};







#endif