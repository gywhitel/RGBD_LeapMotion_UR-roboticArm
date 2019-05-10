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
  ros::init (argc, argv, "motionPlan_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  //==================
  moveit::planning_interface::PlanningSceneInterface current_scene;
  sleep(5.0);
  moveit_msgs::CollisionObject cylinder;

  cylinder.id = "floor_cylinder";
  
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
  pose.position.z =0.0;

  cylinder.primitives.push_back(primitve);
  cylinder.primitive_poses.push_back(pose);
  cylinder.operation=cylinder.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(cylinder);

  current_scene.addCollisionObjects(collision_objects);
  ros::shutdown();
  return 0;
}
