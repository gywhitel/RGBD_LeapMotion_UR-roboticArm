#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
 
int main(int argc, char **argv)
{
   ros::init(argc, argv, "move_test");
   ros::NodeHandle node_handle; 
   ros::AsyncSpinner spinner(1);
   spinner.start();
   moveit::planning_interface::MoveGroupInterface group("manipulator");
   moveit::planning_interface::MoveItErrorCode success;
   moveit::planning_interface::MoveGroupInterface::Plan plan;

    //设置目标位置所使用的参考坐标系
    std::string pose_reference_frame="base_link";
    group.setPoseReferenceFrame(pose_reference_frame);
    
    //获取终端link的名称
    std::string end_effector_link=group.getEndEffectorLink();
    geometry_msgs::Pose start_pose=group.getCurrentPose (end_effector_link).pose;
   //设置初始位置

   geometry_msgs::Pose target_pose;

   target_pose.position.x = -0.24;
   target_pose.position.y = -0.21;
   target_pose.position.z = 0.32; 

   target_pose.orientation.w = 0;   //四元素
   target_pose.orientation.x = 1;
   target_pose.orientation.y = 0;
   target_pose.orientation.z = 0;

   group.clearPoseTargets();

   group.setStartState(*group.getCurrentState());
   std::vector<geometry_msgs::Pose> waypoints;
   geometry_msgs::Pose target_pose1 = start_pose;
   waypoints.push_back(target_pose1);

   target_pose1 = target_pose;
   waypoints.push_back(target_pose1);
    

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group.computeCartesianPath(waypoints,0.01,0.0,trajectory,true,NULL);
    ROS_INFO("Visualing paln 3 (%.2f % % acheived)",fraction * 100.0);
    //输出运动
    plan.trajectory_ = trajectory;
    group.execute(plan);
    group.clearPathConstraints();
    
 
   ros::shutdown(); 
   return 0;
}
