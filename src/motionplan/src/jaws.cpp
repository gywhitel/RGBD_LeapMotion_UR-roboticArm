#include <ur_msgs/SetIO.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
//控制夹爪夹紧
class Gripper_control_high
{
public:
  	Gripper_control_high(ros::NodeHandle& nh)
  	{
    		client = nh.serviceClient<ur_msgs::SetIO>("ur_driver/set_io");
  	}
  	void start()
  	{
    		ur_msgs::SetIO srv;
    		srv.request.fun = 1;
    		srv.request.pin = 5;
    		srv.request.state = 24.0;
    		if (client.call(srv))
    		{
      			ROS_INFO("success ?  %s", srv.response.success ? "True" : "False");
    		}
    		else
    		{
      			ROS_ERROR("Failed to call service ur_driver/set_io");
      			return;
    		}
  	}
private:
  	ros::ServiceClient client;
};

//控制夹爪松开
class Gripper_control_low
{
public:
  	Gripper_control_low(ros::NodeHandle& nh)
  	{
    		client = nh.serviceClient<ur_msgs::SetIO>("ur_driver/set_io");
  	}
  	void start()
  	{
    		ur_msgs::SetIO srv;
    		srv.request.fun = 1;
    		srv.request.pin = 5;
    		srv.request.state = 0.0;
    		if (client.call(srv))
    		{
      			ROS_INFO("success ?  %s", srv.response.success ? "True" : "False");
    		}
    		else
    		{
      			ROS_ERROR("Failed to call service ur_driver/set_io");
      			return;
    		}
  	}
private:
  	ros::ServiceClient client;
};


int jaws;
 
 void chatterCallback(const std_msgs::Float32ConstPtr& teleop)
 {
    /* float x, y,z;
     x=teleop->x;
     y=teleop->y;
     z=teleop->z;*/
     ros::NodeHandle nh;
     Gripper_control_high jaws_high(nh);
     Gripper_control_low  jaws_low(nh);

     jaws=teleop->data;
     std::cout<<"jaws,state ["<<jaws<<"]"<<std::endl;
    
    if(jaws == 1)
     {
         jaws_high.start();
         ROS_INFO("high");
     }else
     {
         jaws_low.start();
         ROS_INFO("low");
     }  

 }
 
 int main(int argc, char **argv)
 {
     ros::init(argc,argv,"ur3_jaws");
     ros::NodeHandle n;
     
     ROS_INFO("Start");
     ros::Subscriber sub = n.subscribe("Lfinger_direction",1,chatterCallback);

     ROS_INFO("end");
     ros::spin();
 
     return 0;
 }