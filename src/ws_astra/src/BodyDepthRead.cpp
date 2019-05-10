#include "ros/ros.h"
#include <astra/astra.hpp>
#include <iostream>
#include <string.h>
#include <cstring>
#include <math.h>
#include <geometry_msgs/Vector3.h>

#include "key_handler.h"


class BodyVisualizer : public astra::FrameListener
{
public:

    BodyVisualizer(){
        j=0;
        std::cout<<"Start Tell"<<std::endl;
        vel_pub= nh.advertise<geometry_msgs::Vector3>("hand_postion",1);
        vel1_pub= nh.advertise<geometry_msgs::Vector3>("RightHand_direction",1);
    }
    //监听消息
    virtual void on_frame_ready(astra::StreamReader& reader,
                                astra::Frame& frame) override
    {
        //processDepth(frame);
        processBodies(frame);
        Tell();
        check_fps();
    }

    void check_fps()
    {
        double fpsFactor = 0.02;

        std::clock_t newTimepoint= std::clock();
        long double frameDuration = (newTimepoint - lastTimepoint_) / static_cast<long double>(CLOCKS_PER_SEC);

        frameDuration_ = frameDuration * fpsFactor + frameDuration_ * (1 - fpsFactor);
        lastTimepoint_ = newTimepoint;
        double fps = 1.0 / frameDuration_;

        printf("FPS: %3.1f (%3.4Lf ms)\n", fps, frameDuration_ * 1000);
    }

    void processBodies(astra::Frame& frame)
    {
        astra::BodyFrame bodyFrame = frame.get<astra::BodyFrame>();

        jointPositions_.clear();

        const float jointScale = bodyFrame.info().width() / 120.f;

        const auto& bodies = bodyFrame.bodies();

        for (auto& body : bodies)
        {
            printf("Processing frame #%d body %d left hand: %u\n",
                bodyFrame.frame_index(), body.id(), unsigned(body.hand_poses().left_hand()));
            astra_vector3f_t L_Position_0,L_Position_1,R_Position_0,R_Position_1,MidSpine,RightShoulder;
            for(auto& joint : body.joints())
            {   
                //aprocessBodies(frame);stra::JointType jointType;
                jointPositions_.push_back(joint.depth_position());
                int jointType=(int)joint.type();
                std::string jointname=getJointName(jointType);
                if(jointname == "RightElbow")
                    R_Position_0 = joint.world_position();
                if(jointname == "RightHand")
                    R_Position_1 = joint.world_position();
                if(jointname == "LeftElbow")
                    L_Position_0 = joint.world_position();
                if(jointname == "LeftHand")
                    L_Position_1 = joint.world_position();
                if(jointname == "MidSpine")
                    MidSpine= joint.world_position();
                if(jointname == "RightShoulder")
                    RightShoulder = joint.world_position();                                  
                std::cout<<(int)joint.type()<<std::setw(15)<<" joint name: "<<jointname<<"    x,y,z: "<<joint.world_position().x<<" "
                                                                            <<joint.world_position().y<<" "
                                                                            <<joint.world_position().z<<std::endl;
            }
            //获取手肘到手的单位向量
            float length = getLengthTwo(R_Position_0,R_Position_1);
            R_Elbow_Hand_direction = getDirection(R_Position_0,R_Position_1,length);
                  length = getLengthTwo(L_Position_0,L_Position_1);
            L_Elbow_Hand_direction = getDirection(L_Position_0,L_Position_1,length);
            //更新相对坐标空间长度
            if(L_Elbow_Hand_direction.y <= -0.9)
                updateURLength(RightShoulder,L_Position_1);
            //获取Mid到手的方向向量
            Mid_R_Hand_direction  = getDirection(MidSpine , R_Position_1,URLength);

            std::cout<<"Right Elbow -> Hand direction: "<<"x,y,z: "<<std::setprecision(3)<<R_Elbow_Hand_direction.x<<" "
                                                                            <<R_Elbow_Hand_direction.y<<" "
                                                                            <<R_Elbow_Hand_direction.z<<std::endl;
            std::cout<<"Left Elbow -> Hand direction: "<<"x,y,z: "<<std::setprecision(3)<<L_Elbow_Hand_direction.x<<" "
                                                                            <<L_Elbow_Hand_direction.y<<" "
                                                                            <<L_Elbow_Hand_direction.z<<std::endl;
            std::cout<<"Mid Spine -> Rihght Hand direction: "<<"x,y,z: "<<std::setprecision(3)<<Mid_R_Hand_direction.x<<" "
                                                                            <<Mid_R_Hand_direction.y<<" "
                                                                            <<Mid_R_Hand_direction.z<<std::endl;
        }std::cout<<std::endl;

        const auto& floor = bodyFrame.floor_info(); //floor
        if (floor.floor_detected())
        {
            const auto& p = floor.floor_plane();
            std::cout << "Floor plane: ["
                << p.a() << ", " << p.b() << ", " << p.c() << ", " << p.d()
                << "]" << std::endl;
        }
    }
    //计算位置获返回关节名字
    std::string getJointName(int num){

        std::string JointName[]={"Head","ShoulderSpine","LeftShoulder","LeftElbow","LeftHand",  
                             "RightShoulder","RightElbow","RightHand","MidSpine","BaseSpine",
                             "LeftHip","LeftKnee","LeftFoot","RightHip","RightKnee",
                             "RightFoot","LeftWrist","RightWrist","Neck","Unknown"};
        return JointName[num];
    }
    //两点计算空间长度
    float getLengthTwo(astra_vector3f_t Position_0,astra_vector3f_t Position_1)
    {
        float x,y,z,length;
        x = Position_1.x - Position_0.x;
        y = Position_1.y - Position_0.y;
        z = Position_1.z - Position_0.z;
        length=sqrt(pow(x,2)+pow(y,2)+pow(z,2));
        return length;
    }
    //两点计算空间方向向量0->1
    astra_vector3f_t getDirection(astra_vector3f_t Position_0,astra_vector3f_t Position_1,float length)
    {
        astra_vector3f_t direction;
        float x,y,z;
        x = Position_1.x - Position_0.x;
        y = Position_1.y - Position_0.y;
        z = Position_1.z - Position_0.z;
        //length=sqrt(pow(x,2)+pow(y,2)+pow(z,2));
        direction.x = x/length;
        direction.y = y/length;
        direction.z = z/length;

        return direction;   
    }
    //根据向量相对于人坐标系坐标
    astra_vector3f_t getRelativePosition(astra_vector3f_t RightHandDirection,float length )
    {
        astra_vector3f_t RelativePosition;
        RelativePosition.x = RightHandDirection.x * length;
        RelativePosition.y = RightHandDirection.y * length;
        RelativePosition.z = RightHandDirection.z * length;
        return RelativePosition;   
    }
    void updateURLength(astra_vector3f_t Position_0,astra_vector3f_t Position_1)
    {
        URLength = getLengthTwo(Position_0 , Position_1);
    }
    
    void Tell()
    {
        if(std::abs(L_Elbow_Hand_direction.z) >= 0.75)
        {  

            astra_vector3f_t Position = getRelativePosition(Mid_R_Hand_direction , UR3);
            teleop.x = Position.x;
            teleop.y = Position.z;
            teleop.z = Position.y;
            vel_pub.publish(teleop);
            std::cout<<"Publish Hand Position Success ! : "<<std::setprecision(3)<<teleop.x<<" "
                                                                            <<teleop.y<<" "
                                                                            <<teleop.z<<std::endl;
        }
        if(L_Elbow_Hand_direction.y >= 0.75)
        {
            astra_vector3f_t Position = R_Elbow_Hand_direction;
            teleop.x = Position.x;
            teleop.y = Position.z;
            teleop.z = Position.y;
            vel1_pub.publish(teleop);
            std::cout<<"Publish Elbow Hand Direction Success ! : "<<std::setprecision(3)<<teleop.x<<" "
                                                                            <<teleop.y<<" "
                                                                            <<teleop.z<<std::endl;
        }
    }

private:
    long double frameDuration_{ 0 };
    std::clock_t lastTimepoint_ { 0 };

    std::vector<astra::Vector2f> jointPositions_;
    astra_vector3f_t R_Elbow_Hand_direction,L_Elbow_Hand_direction,Mid_R_Hand_direction;
    //设置UR机械臂总长度为0.6米
    float URLength;
    float UR3 = 0.55;
    
    //设置发布消息
    ros::Publisher vel_pub;
    ros::Publisher  vel1_pub;    
    ros::NodeHandle nh; 
    geometry_msgs::Vector3  teleop;
    int j;
    
};

astra::DepthStream configure_depth(astra::StreamReader& reader)
{
    auto depthStream = reader.stream<astra::DepthStream>();

    //We don't have to set the mode to start the stream, but if you want to here is how:
    astra::ImageStreamMode depthMode;

    depthMode.set_width(640);
    depthMode.set_height(480);
    depthMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
    depthMode.set_fps(30);

    depthStream.set_mode(depthMode);

    return depthStream;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "BodyDepthRead");
    ros::NodeHandle n;    
    
    astra::initialize();
    const char* licenseString = "<INSERT LICENSE KEY HERE>";
    orbbec_body_tracking_set_license(licenseString);

    astra::StreamSet sensor;
    astra::StreamReader reader = sensor.create_reader();
    BodyVisualizer listener;

    auto depthStream = configure_depth(reader);
    depthStream.start();
    auto bodyStream = reader.stream<astra::BodyStream>();
    bodyStream.start();

    reader.add_listener(listener);
    
    // HandPoses includes Joints and Segmentation
    astra::BodyTrackingFeatureFlags features = astra::BodyTrackingFeatureFlags::HandPoses;
    
    
    ros::Rate loop_rate(5);
    int i=0;
    while (ros::ok())
    {
        astra_update();
        //调试运行方式
        //std::cout<<"main def: i= "<<i++<<std::endl;
        loop_rate.sleep();
    } 
    

    astra::terminate();
}