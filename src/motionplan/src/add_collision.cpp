#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
/*
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
*/
#include <boost/scoped_ptr.hpp>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "add_collision");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");
  //==================
  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  moveit::planning_interface::PlanningSceneInterface current_scene;
  std::vector<moveit_msgs::CollisionObject> collision_objects;

  sleep(1.0);
  moveit_msgs::CollisionObject cylinder;
  cylinder.id = "floor_cylinder";
  cylinder.header.frame_id="world"; 
  shape_msgs::SolidPrimitive primitve;
  primitve.type = primitve.BOX;
  primitve.dimensions.resize(3);
  primitve.dimensions[0]=1.2;
  primitve.dimensions[1]=-0.4;
  primitve.dimensions[1]=1.2;

  geometry_msgs::Pose pose;
  pose.orientation.w =1.0;
  pose.position.x =0.0;
  pose.position.y =0.0;
  pose.position.z =-0.01;

  cylinder.primitives.push_back(primitve);
  cylinder.primitive_poses.push_back(pose);
  cylinder.operation=cylinder.ADD;

  collision_objects.push_back(cylinder);
  //current_scene.addCollisionObjects(collision_objects);
  sleep(1);
//Left
  moveit_msgs::CollisionObject cyLeft;
  cyLeft.id = "floor_cyLeft";//move_group.getPlanningFrame();//"floor_cyLeft";  
  cyLeft.header.frame_id="world";
  shape_msgs::SolidPrimitive primitve_left;
  primitve_left.type = primitve_left.BOX;
  primitve_left.dimensions.resize(3);
  primitve_left.dimensions[0] = 0.1;
  primitve_left.dimensions[1] = 0.8;
  primitve_left.dimensions[2] = 0.5;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -0.20;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.25;

  cyLeft.primitives.push_back(primitve_left);
  cyLeft.primitive_poses.push_back(box_pose);
  cyLeft.operation=cyLeft.ADD;

  collision_objects.push_back(cyLeft);
  current_scene.addCollisionObjects(collision_objects);
  ros::Duration(1.0).sleep();
  ros::shutdown();
  return 0;
}
