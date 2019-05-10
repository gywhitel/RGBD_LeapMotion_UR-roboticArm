#include<iostream>
#include"Leapros.h"
#include <ros/ros.h>
#include <math.h>
#include <iomanip>

void RosListener::onInit(const Leap::Controller& controller){
    std::cout << "Initialized" <<std::endl;
    //vel_pub = nh_.advertise<geometry_msgs::Point>("hand_postion",10);
    //data_sz=4;
    teleop.x = 0;
    teleop.y = 0;
    teleop.z = 0;
    
    teleop_1.x = 0;
    teleop_1.y = 0;
    teleop_1.z = 0;
    
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


  Leap::HandList hands = frame.hands();   //获取手对象的列表
  for (Leap::HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
    // 获得Hand对象第一个
    const Leap::Hand hand = *hl;
    std::string handType = hand.isLeft() ? "Left hand" : "Right hand";  //判断手的种类

    if(hand.isLeft()){
        teleop.x = floor(hand.palmPosition().x)/1000;
        teleop.y = floor(hand.palmPosition().y)/1000;
        teleop.z = -floor(hand.palmPosition().z)/1000;
        FlagLeft = hand.grabStrength()>0.7 ? 1 : 0;//右手握拳区分      
        std::cout << std::string(2, ' ') << FlagLeft <<std::endl;
        //发布手位置
        std::cout << std::string(2, ' ') << handType << ",position: " <<teleop.x<<" "
                                                                 <<teleop.y<<" "
                                                                 <<teleop.z
                                                                << std::endl;
        // Get fingers
        const Leap::FingerList fingers = hand.fingers();    
        double OneDirection,TwoDirection;
        for (Leap::FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
            const Leap::Finger finger = *fl;
            //取1 2指的指向
            if(finger.type() == 0){
                Leap::Bone::Type boneType = static_cast<Leap::Bone::Type>(3);
                Leap::Bone bone = finger.bone(boneType);
                OneDirection = bone.direction()[2];           
            }
            if(finger.type() == 1){
                Leap::Bone::Type boneType = static_cast<Leap::Bone::Type>(3);
                Leap::Bone bone = finger.bone(boneType);
                TwoDirection = bone.direction()[2];
                break ;                    
            }
        }
        //计算两指间夹角
        FingerAngel = std::fabs(OneDirection - TwoDirection);
        TeleopJaw.data = FingerAngel <= 0.2 ? 1 : FingerAngel>=0.80 ? 0.1 : 0 ;
    }    

    if(hand.isRight()){
        std::cout<<std::string(2, ' ')<< handType <<" Right Strength: "<<hand.grabStrength()<<std::endl;
        // Get fingers
        const Leap::FingerList fingers = hand.fingers();       
        double Direction_x,Direction_y,Direction_z;
        for (Leap::FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
            const Leap::Finger finger = *fl;
            //取1 2指的指向
            if(finger.type() == 0){
                Leap::Bone::Type boneType = static_cast<Leap::Bone::Type>(3);
                Leap::Bone bone = finger.bone(boneType);
                Direction_y = bone.direction()[2];
            } 
            if(finger.type() == 1){
                Leap::Bone::Type boneType = static_cast<Leap::Bone::Type>(3);
                Leap::Bone bone = finger.bone(boneType);
                Direction_x = -bone.direction()[0];
                Direction_z = -bone.direction()[1];
                break;                    
            }
        }
        if(hand.grabStrength() >= 0.9)
            Direction_x = Direction_z =0.0;
        if(hand.grabStrength()<=0.1)
            Direction_y = 0.0;
        teleop_1.x = round(Direction_x);
        teleop_1.y = round(Direction_y);
        teleop_1.z = round(Direction_z);
        if(teleop_1.x!=0.0 || teleop_1.y!=0.0 || teleop_1.z!=0.0)
            FlagRight=1;
        std::cout<<"  x,y,z"<<std::fixed<<std::setprecision(2)<<teleop_1.x<<"  "<<teleop_1.y<<"  "<< teleop_1.z<<std::endl;
    }    
  }
}