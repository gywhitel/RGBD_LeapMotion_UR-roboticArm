 #include "ros/ros.h"
 #include"Leapros.cpp"
 #include"Leap.h"
#include <std_msgs/Float32MultiArray.h>
 int main(int argc, char **argv){

    ros::init(argc, argv, "hand_postion_talk");//定义节点hand_postion_talk
    ros::NodeHandle n;
    ros::Publisher vel_pub;//设置发布位置
    
    ros::Publisher vel1_pub;//设置发布方向
    
    vel_pub= n.advertise<std_msgs::Float32MultiArray>("hand_postion",0);//设置topic为hand_postion，发布信息为geometry_msgs::Point
    
    vel1_pub= n.advertise<std_msgs::Float32MultiArray>("hand_direction",1);
    
    RosListener listener;  //创建监听对象
    Leap::Controller controller;//创建对象
    controller.addListener(listener);//Listener对象注册到Controller对象
 
    std_msgs::Float32MultiArray point;
    std_msgs::Float32MultiArray direction;

    std::cout <<"start publish "<<std::endl;
    point=listener.teleop;
    direction=listener.teleop_1;
    
    ros::Rate loop_rate(1); //设置发布频率１HZ
    while (ros::ok())
    {
        direction=listener.teleop_1;
        if(direction.data[0] == 1){
            vel1_pub.publish(direction);//发布方向
            listener.teleop_1.data[0] = 0;
            sleep(1);
            continue;
        }
        if(listener.flag != 1){
            continue;
        }         
        point=listener.teleop;//接受leap motion 的信息
        vel_pub.publish(point);//发布信息
        ros::spinOnce();//回调函数
        loop_rate.sleep();
    }
    
    std::cout <<"Stop ..."<<std::endl;
    controller.removeListener(listener);
    
}
