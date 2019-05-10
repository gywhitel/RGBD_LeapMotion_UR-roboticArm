#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "remove_collision");
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    

    moveit::planning_interface::PlanningSceneInterface current_scene;

    sleep(1.0);

    std::vector<std::string> object_ids;
    object_ids.push_back("floor_cyLeft");
    current_scene.removeCollisionObjects(object_ids);
    object_ids.push_back("floor_cylinder");    
    ROS_INFO("remove an object from the world");
    current_scene.removeCollisionObjects(object_ids);

    ros::shutdown();
}