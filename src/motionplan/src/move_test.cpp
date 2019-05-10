#include <moveit/move_group_interface/move_group.h>  //MoveIt的头文件，API接口
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <vector>

using namespace std;
void dispersed(double pt1[3],double pt2[3]);        

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_test", ros::init_options::AnonymousName);
    // 创建一个异步的自旋线程（spinning thread）
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // 连接move_group节点中的机械臂实例组，这里的组名arm是我们之前在setup assistant中设置的

    // 假设我有俩个点pt1和pt2，我们需要将机械臂的末端从pt1移动到pt2
    double pt1[3]={0.10,-0.17,0.32};  
    double pt2[3]={0.24,-0.29,0.22};
    dispersed(pt1,pt2);
  ros::shutdown();
  return 0;
}

void dispersed(double pt1[3],double pt2[3])
{
    move_group_interface::MoveGroup group("manipulator");

    double Xf;
    double org_x,org_y,org_z;
    double crt_x;
    org_y=pt1[1];
    org_z=pt1[2];
    org_x = pt1[0];
    int xtimes=50;          //X轴直线离散化倍率
    Xf=(pt2[0]-pt1[0])/xtimes;   //X轴进给量
    cout << " xtimes = " << xtimes << endl;
    for(int x=0;x<=xtimes;x++)
    {
        cout << "x=" << x << endl;
        crt_x = org_x + x*Xf;
        geometry_msgs::Pose target_pose;
        target_pose.position.x = crt_x; //位姿
        target_pose.position.y = org_y;
        target_pose.position.z = org_z;
        cout << "target_pose.position.x = " << target_pose.position.x << endl;
        target_pose.orientation.w = 0;   //四元素
        target_pose.orientation.x = 1;
        target_pose.orientation.y = 0;
        target_pose.orientation.z = 0;
        group.setPoseTarget(target_pose);  
        //group.move();
        moveit::planning_interface::MoveGroup::Plan planner;
        bool is_success=group.plan(planner);
        if(is_success)
        {
            group.move();
            sleep(1);
        }
       else
        {
            cout<<"Planning fail!"<<endl;
        }
    }
}              