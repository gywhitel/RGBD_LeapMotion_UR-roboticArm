#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
 
 void chatterCallback(const std_msgs::Float32MultiArrayConstPtr& teleop)
 {
    /* float x, y,z;
     x=teleop->x;
     y=teleop->y;
     z=teleop->z;*/

     std::cout<<"x,y,z,state ["<<teleop->data[0]<<"],["<<teleop->data[1]<<"],["<<teleop->data[2]<<"],["<<teleop->data[3]<<"]"<<std::endl;
 }
 
 int main(int argc, char **argv)
 {
     ros::init(argc,argv,"ur3_listener");
     ros::NodeHandle n;
     ROS_INFO("Start subscribe");
     ros::Subscriber sub = n.subscribe("hand_postion",1,chatterCallback);
     ros::spin();
 
     return 0;
 }