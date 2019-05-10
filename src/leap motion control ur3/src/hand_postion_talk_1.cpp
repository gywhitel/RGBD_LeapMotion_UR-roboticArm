 #include "ros/ros.h"
 #include"Leapros.cpp"
 #include"Leap.h"
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
 int main(int argc, char **argv){

    ros::init(argc, argv, "hand_postion_talk");//定义节点hand_postion_talk
    ros::NodeHandle n;
    ros::Publisher vel_pub;//设置发布位置
    ros::Publisher vel1_pub;//设置发布夹角
    ros::Publisher vel2_pub;//设置发布方向
    vel_pub= n.advertise<geometry_msgs::Vector3>("hand_postion",1);//设置topic为hand_postion，发布信息为geometry_msgs::Point    
    vel1_pub= n.advertise<std_msgs::Float32>("Lfinger_direction",1);
    vel2_pub= n.advertise<geometry_msgs::Vector3>("Rfinger_direction",1);
    
    RosListener listener;  //创建监听对象
    Leap::Controller controller;//创建对象
    controller.addListener(listener);//Listener对象注册到Controller对象

    geometry_msgs::Vector3 point;
    std_msgs::Float32 Lfinger_direction;
    geometry_msgs::Vector3 Rfinger_direction;

    std::cout <<"start publish "<<std::endl;   
    ros::Rate loop_rate(2); //设置发布频率１HZ
    while (ros::ok())
    {
        if(listener.FlagLeft == 0 && listener.TeleopJaw.data != 0){
            Lfinger_direction=listener.TeleopJaw;
            vel1_pub.publish(Lfinger_direction);//发布指间夹角
            std::cout <<"  FingerAngel:  "<< Lfinger_direction.data<<std::endl;
            listener.TeleopJaw.data = 0;
            continue;
        }
        if(listener.FlagRight == 1){
            Rfinger_direction=listener.teleop_1;
            vel2_pub.publish(Rfinger_direction);//发布方向
            listener.FlagRight = 0;

        }
        if(listener.FlagLeft == 1){
            point = listener.teleop;//接受leap motion 的信息
            point.x += 0.0;
            point.y += 0.1;
            point.z += 0.2;
            vel_pub.publish(point);//发布信息
            listener.FlagLeft = 0;
        } 

        ros::spinOnce();//回调函数
        loop_rate.sleep();
    }
    
    std::cout <<"Stop ..."<<std::endl;
    controller.removeListener(listener);
    
}
