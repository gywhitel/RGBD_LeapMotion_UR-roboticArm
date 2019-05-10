#include "ros/ros.h"
#include <sstream>
#include <iomanip>
#include <deque>
#include <unordered_map>
#include <chrono>
#include <astra_core/astra_core.hpp>
#include <astra/astra.hpp>
#include "LitDepthVisualizer.hpp"
#include "key_handler.h"


class HandFrameListener : public astra::FrameListener
{

public:
    using PointList = std::deque<astra::Vector2i>;
    using PointMap = std::unordered_map<int, PointList>;


    void check_fps()
    {
        const float frameWeight = .2f;

        const ClockType::time_point now = ClockType::now();
        const float elapsedMillis = std::chrono::duration_cast<DurationType>(now - prev_).count();

        elapsedMillis_ = elapsedMillis * frameWeight + elapsedMillis_ * (1.f - frameWeight);
        prev_ = now;

        const float fps = 1000.f / elapsedMillis;
        const auto precision = std::cout.precision();

        std::cout << std::fixed
                  << std::setprecision(1)
                  << fps << " fps ("
                  << std::setprecision(1)
                  << elapsedMillis_ << " ms)"
                  << std::setprecision(precision)
                  << std::endl;
    }  

    void shorten_hand_traces()
    {
        auto it = pointMap_.begin();

        while (it != pointMap_.end())
        {
            PointList& list = it->second;
            if (list.size() > 1)
            {
                list.pop_front();
                ++it;
            }
            else
            {
                it = pointMap_.erase(it);
            }
        }
    }

    void process_hand_frame(astra::Frame& frame)
    {
        const astra::HandFrame handFrame = frame.get<astra::HandFrame>();

        handPoints_ = handFrame.handpoints();

        shorten_hand_traces();
        
        for (const auto& handPoint : handPoints_)
        {
            std::cout<<"arrive "<<std::endl;
            //if (handPoint.status() == HAND_STATUS_TRACKING)            {
                const auto worldPosition = handPoint.world_position();    
                std::cout<<"tracking_id: "<<handPoint.tracking_id()<<"       handPoint status: "<<handPoint.status()<<std::endl;
                std::cout<<"   depth_position: "<<worldPosition.x<<"  "
                                                <<worldPosition.y<<"  "
                                                <<worldPosition.z<<"  "
                                                <<std::endl;
                //update_hand_trace(handPoint.tracking_id(), handPoint.depth_position());
            //}
        }
    }

    virtual void on_frame_ready(astra::StreamReader& reader,
                                astra::Frame& frame) override
    {
        //process_depth(frame);
        process_hand_frame(frame);

        check_fps();
    }


private:
    samples::common::LitDepthVisualizer visualizer_;

    using DurationType = std::chrono::milliseconds;
    using ClockType = std::chrono::high_resolution_clock;

    ClockType::time_point prev_;
    float elapsedMillis_{.0f};


    std::vector<astra::HandPoint> handPoints_;
    PointMap pointMap_;
};



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "HandDepthRead");
    ros::NodeHandle n;

    astra::initialize();
    set_key_handler();

    astra::StreamSet streamSet;
    astra::StreamReader reader = streamSet.create_reader();
    
    reader.stream<astra::PointStream>().start();
    reader.stream<astra::HandStream>().start();

    HandFrameListener listener;
    reader.add_listener(listener);
    int i=0;
    ros::Rate loop_rate(5);
    while (shouldContinue)
    {
        astra_update();
        loop_rate.sleep();
    }


    //结束
    astra::terminate();
    return 0;
}