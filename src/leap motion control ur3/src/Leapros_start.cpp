#include<iostream>
#include"Leapros.h"
#include <ros/ros.h>
#include <math.h>

void RosListener::onInit(const Leap::Controller& controller){
    std::cout << "Initialized" <<std::endl;
    //vel_pub = nh_.advertise<geometry_msgs::Point>("hand_postion",10);
    //data_sz=4;
    teleop.data.resize(4);
    teleop.data[0] = 0;
    teleop.data[1] = 0;
    teleop.data[2] = 0;
    teleop.data[3] = 0;
}

void RosListener::onConnect(const Leap::Controller& controller){
    std::cout<< "Connected" <<std::endl;
    controller.enableGesture(Leap::Gesture::TYPE_CIRCLE);	//启用画圆手势
}

void RosListener::onDisconnect(const Leap::Controller& controller){
    std::cout<< "Disconnected" <<std::endl;
}

void RosListener::onExit(const Leap::Controller& controller){
    std::cout<< "Exited" <<std::endl;
}

void RosListener::onFrame(const Leap::Controller& controller){	//获取数据
    const Leap::Frame frame = controller.frame();

/*
std::cout << "Frame id: " << frame.id()  //对象ID
            << ", timestamp: " << frame.timestamp() //时间戳
            << ", hands: " << frame.hands().count() //手的数量
            << ", extended fingers: " << frame.fingers().extended().count()<<std::endl; //手指对象的个数
*/
  Leap::HandList hands = frame.hands();   //获取手对象的列表
  for (Leap::HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
    // 获得Hand对象第一个
    const Leap::Hand hand = *hl;
    std::string handType = hand.isLeft() ? "Left hand" : "Right hand";  //判断手的种类

    if(hand.isLeft()){
        teleop.data[0] = floor(hand.palmPosition().x);
        teleop.data[1] = floor(hand.palmPosition().y);
        teleop.data[2] = floor(hand.palmPosition().z);
        float strength = round(hand.grabStrength());   
        teleop.data[3] = strength;//>0.9 ? 1 : 0;//左手握拳为１，否则为０
        //发布手位置
        std::cout << std::string(2, ' ') << handType << ",position: " <<1.2*hand.palmPosition().z/1000<<" "
                                                                 <<1.2*hand.palmPosition().x/1000<<" "
                                                                 <<1.4*hand.palmPosition().y/1000
                                                                 << std::endl;
        std::cout<<"state: "<<teleop.data[3]<<std::endl;
    }
    if(hand.isRight()){
        flag=hand.grabStrength()>0.9 ? 1 : 0;//右手握拳区分
        std::cout<<std::string(2, ' ')<< handType <<" flag: "<<flag<<std::endl;
    }    
  }
/*
    std::cout<<"tools: "<<frame.tools().count()<<std::endl;
    const Leap::ToolList tools= frame.tools();
    const Leap::Tool tool = *tools.begin();
    
    if (tool.id() != -1){
        std::cout<<"Tool, id: "<< tool.id() <<std::endl;
        std::cout<<"Tool position: "<<tool.tipPosition()<<std::endl;
             //<<", direction: "<<tool.direction()[0]<<"------"<<tool.direction()[1]<< std::endl;
        //float x_speed = tool.direction()[1];
        //float z_rspeed = tool.direction()[0];
        teleop.data[0] = tool.tipPosition().x;
        teleop.data[1] = tool.tipPosition().y;
        teleop.data[2] = tool.tipPosition().z;
        output = "screen" launch-prefix="gnome-terminal -e"
        
    }*/
}