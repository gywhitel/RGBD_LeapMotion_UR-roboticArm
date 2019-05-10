
movit 相关函数（二）
//The kinematics.yaml file 运动学参数文件
//该文件是由MoveIt! Setup Assistant生成的记录初始组态的。
right_arm:
  kinematics_solver: pr2_arm_kinematics/PR2ArmKinematicsPlugin
  kinematics_solver_search_resolution: 0.001
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 3
right_arm_and_torso:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05

//kinematics_solver:是运动学处理插件的名字，应该和你在插件描述文件(plugin description file)中的声明是一样的。
//kinematics_solver_search_resolution: 声明了分辨率，sovler可能用它来搜索运动学反解的不可达空间。？？
//kinematics_solver_timeout:运动学反解每个内部循环的延迟。
//kinematics_solver_attempts: 反解中restarts的个数，关节个数？

//KDL运动学插件：封装的运动学反解包

//Position Only IK
position_only_ik: True//在描述文件中加入下列语句即可声明，用来做什么的？


//Planning Scene/C++ API  C++API中的PlanningScene类，提供了用户碰撞检查和约束检查的界面

//这样的实例化PlanningScene的方式并不推荐，这里只是简单的介绍
robot_model_loader::RobotModelLoader robot_model_loader("ur_description");
robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();//之上提到过
planning_scene::PlanningScene planning_scene(kinematic_model);//使用RobotModel类初始化PlanningScene类

//碰撞检查
  
//自碰撞检查
//我们要做的第一件事是检查现在机器人的组态会不会导致机器人的部分相互碰撞。
collision_detection::CollisionRequest collision_request;//实例化一个CollisionRequest对象
collision_detection::CollisionResult collision_result;//实例化一个CollisionResult对象
planning_scene.checkSelfCollision(collision_request, collision_result);//送入checkSelfCollison函数检查
ROS_INFO_STREAM("Test 1: Current state is "
                << (collision_result.collision ? "in" : "not in")
                << " self collision");//通过collision_result.collision反应出来

//改变机器人的状态
robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();//从PlanningScene类中获得机器人的当前信息
current_state.setToRandomPositions();//随机修改当前信息？
collision_result.clear();//修改信息后，需要调用.clear()才能进行判断
planning_scene.checkSelfCollision(collision_request, collision_result);
ROS_INFO_STREAM("Test 2: Current state is "
                << (collision_result.collision ? "in" : "not in")
                << " self collision");//对于新状态的自碰撞判断


//对于机器人一个“group”检查
collision_request.group_name = "right_arm";//制定要检查的group为“right_arm”
current_state.setToRandomPositions();//随机的改变当时的状态
collision_result.clear();
planning_scene.checkSelfCollision(collision_request, collision_result);
ROS_INFO_STREAM("Test 3: Current state is "
                << (collision_result.collision ? "in" : "not in")
                << " self collision");//进行碰撞判断

//Getting Contact Information 获得触点信息
//我们手动的将"right_arm"推到一个会碰撞的点，并会检查是不是超出关节限制
std::vector<double> joint_values;
const robot_model::JointModelGroup* joint_model_group =
  current_state.getJointModelGroup("right_arm");
current_state.copyJointGroupPositions(joint_model_group, joint_values);
joint_values[0] = 1.57; //hard-coded since we know collisions will happen here
current_state.setJointGroupPositions(joint_model_group, joint_values);
ROS_INFO_STREAM("Current state is "
                << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));

//现在我们可以获得“right_arm”可能出现的接触点的信息，
collision_request.contacts = true;
collision_request.max_contacts = 1000;

collision_result.clear();
planning_scene.checkSelfCollision(collision_request, collision_result);
ROS_INFO_STREAM("Test 4: Current state is "
                << (collision_result.collision ? "in" : "not in")
                << " self collision");
collision_detection::CollisionResult::ContactMap::const_iterator it;
for(it = collision_result.contacts.begin();
    it != collision_result.contacts.end();
    ++it)
{
  ROS_INFO("Contact between: %s and %s",
  it->first.first.c_str(),it->first.second.c_str());
}//这个contact是什么鬼？

//修改“允许碰撞模型”“the Allowed Collision Matrix”
//ACM提供了一种在“碰撞模型的世界”下忽略与某些物体(其中包括了机器人本身和外界世界)碰撞的机制。
//接下来我们将会复制“the Allowed Collision Matrix”和“the current state”，并将他们传递给碰撞检查的函数
ollision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
robot_state::RobotState copied_state = planning_scene.getCurrentState();

collision_detection::CollisionResult::ContactMap::const_iterator it2;
for(it2 = collision_result.contacts.begin();
    it2 != collision_result.contacts.end();
    ++it2)
{
  acm.setEntry(it2->first.first, it2->first.second, true);
}
collision_result.clear();
planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);//检查判断
ROS_INFO_STREAM("Test 5: Current state is "
                << (collision_result.collision ? "in" : "not in")
                << " self collision");//这里还是没看懂在干什么

//全局碰撞检查
//copied_state携带机器人信息，acm携带了环境的信息
collision_result.clear();
planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
ROS_INFO_STREAM("Test 6: Current state is "
                << (collision_result.collision ? "in" : "not in")
                << " self collision");

//约束检查
//PlanningScene类也提供了方便进行约束检查的调用函数。
//约束一般分为两类，（1）运动学约束：比如说关节约束，位置约束，方向约束和能见度约束  （2）使用者通过回调声明的约束
//检查运动学约束
//We will first define a simple position and orientation constraint on the end-effector of the right_arm of the PR2 robot.
std::string end_effector_name = joint_model_group->getLinkModelNames().back();

geometry_msgs::PoseStamped desired_pose;
desired_pose.pose.orientation.w = 1.0;
desired_pose.pose.position.x = 0.75;
desired_pose.pose.position.y = -0.185;
desired_pose.pose.position.z = 1.3;
desired_pose.header.frame_id = "base_footprint";
moveit_msgs::Constraints goal_constraint =
  kinematic_constraints::constructGoalConstraints(end_effector_name, desired_pose);

//检查这个约束
copied_state.setToRandomPositions();
copied_state.update();//什么鬼？
bool constrained = planning_scene.isStateConstrained(copied_state, goal_constraint);//以bool来表示新的位置的规划是否与这个约束冲突。
ROS_INFO_STREAM("Test 7: Random state is "
                << (constrained ? "constrained" : "not constrained"));

//还有一个更加高效的检查约束的方式(比如说：当你一次又一次的检查同一个约束时，考虑一个planner)；
//我们首先建立一个KinematicConstraintSet，它可以预处理ROS约束，并且快速建立这个约束。
kinematic_constraints::KinematicConstraintSet kinematic_constraint_set(kinematic_model);
kinematic_constraint_set.add(goal_constraint, planning_scene.getTransforms());//定义类并调用函数
bool constrained_2 =
  planning_scene.isStateConstrained(copied_state, kinematic_constraint_set);//检查约束
ROS_INFO_STREAM("Test 8: Random state is "
                << (constrained_2 ? "constrained" : "not constrained"));

//使用KinematicConstraintSet类可以直接完成这个行动
kinematic_constraints::ConstraintEvaluationResult constraint_eval_result =
  kinematic_constraint_set.decide(copied_state);
ROS_INFO_STREAM("Test 9: Random state is "
                << (constraint_eval_result.satisfied ? "constrained" : "not constrained"));

//用户定义的约束
bool userCallback(const robot_state::RobotState &kinematic_state, bool verbose)
{
  const double* joint_values = kinematic_state.getJointPositions("r_shoulder_pan_joint");
  return (joint_values[0] > 0.0);
}//这个简单的回调函数检测关节是不是活跃的

planning_scene.setStateFeasibilityPredicate(userCallback);
bool state_feasible = planning_scene.isStateFeasible(copied_state);
ROS_INFO_STREAM("Test 10: Random state is "
                << (state_feasible ? "feasible" : "not feasible"));

bool state_valid =
  planning_scene.isStateValid(copied_state, kinematic_constraint_set, "right_arm");
ROS_INFO_STREAM("Test 10: Random state is "
                << (state_valid ? "valid" : "not valid"));//代码没太看懂


//Planning Scene/ROS API
//Adding and removing objects into the world
//Attaching and detaching objects to the robot

//ROS API
//利用ROS API得到一个planning scene publisher是通过一个使用“diff”的话题界面，这也是这与之前的不同之处。


//声明一个必要的
ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
while(planning_scene_diff_publisher.getNumSubscribers() < 1)//与这个topic相连的用户的个数
{
  ros::WallDuration sleep_t(0.5); 
  sleep_t.sleep();//休眠0.5s
}


//定义the attached object消息
//我们将会使用这个消息从世界中加减物体，以及“attach the object to the robot”
moveit_msgs::AttachedCollisionObject attached_object;
attached_object.link_name = "r_wrist_roll_link";
/* The header must contain a valid TF frame*/
attached_object.object.header.frame_id = "r_wrist_roll_link";
/* The id of the object */
attached_object.object.id = "box";

/* A default pose */
geometry_msgs::Pose pose;
pose.orientation.w = 1.0;//为何不定义x,y,z?

/* Define a box to be attached */
shape_msgs::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions.resize(3);
primitive.dimensions[0] = 0.1;
primitive.dimensions[1] = 0.1;
primitive.dimensions[2] = 0.1;

attached_object.object.primitives.push_back(primitive);
attached_object.object.primitive_poses.push_back(pose);


//完成attach the objiect to the robot操作，opreation要制成ADD
attached_object.object.operation = attached_object.object.ADD;


//把物体加入环境，之前的怎么算？
ROS_INFO("Adding the object into the world at the location of the right wrist.");
moveit_msgs::PlanningScene planning_scene;
planning_scene.world.collision_objects.push_back(attached_object.object);
planning_scene.is_diff = true;
planning_scene_diff_publisher.publish(planning_scene);
sleep_time.sleep();


//Attach an object to the robot
//当机器人从环境中抓取物体时，我们需要“attach an object to the robot”，任何有关于机器人模型的操作都会考虑到这个物体

//Attach an object to the robot需要两部操作
//1.将物体从世界中移除
//2.将物体加入机器人模型

//首先，定义移除物体信息
moveit_msgs::CollisionObject remove_object;
remove_object.id = "box";
remove_object.header.frame_id = "odom_combined";
remove_object.operation = remove_object.REMOVE;//移除物体的操作，opreation定义成remove

ROS_INFO("Attaching the object to the right wrist and removing it from the world.");
planning_scene.world.collision_objects.clear();//首先确保这个话题不和其他的物体相关联
planning_scene.world.collision_objects.push_back(remove_object);//从世界移除物体
planning_scene.robot_state.attached_collision_objects.push_back(attached_object);//加入机器人
planning_scene_diff_publisher.publish(planning_scene);//发布出相应的消息

sleep_time.sleep();

//Detach an object from the robot
//上述操作分为两步
//1.从机器人移除物体
//2.重新加回世界
moveit_msgs::AttachedCollisionObject detach_object;
detach_object.object.id = "box";
detach_object.link_name = "r_wrist_roll_link";
detach_object.object.operation = attached_object.object.REMOVE;

ROS_INFO("Detaching the object from the robot and returning it to the world.");
planning_scene.robot_state.attached_collision_objects.clear();
planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
planning_scene.world.collision_objects.clear();
planning_scene.world.collision_objects.push_back(attached_object.object);
planning_scene_diff_publisher.publish(planning_scene);

sleep_time.sleep();//整体与之上是一致的


//将物体移除世界
ROS_INFO("Removing the object from the world.");
planning_scene.robot_state.attached_collision_objects.clear();
planning_scene.world.collision_objects.clear();
planning_scene.world.collision_objects.push_back(remove_object);
planning_scene_diff_publisher.publish(planning_scene);



//运动规划 C++ API
//我们将用C++代码完成运动规划

//start
//在开始规划之前，我们需要两样东西
//1.机器人模型（RobotModel class）
//2.规划现场的信息（PlanningScene class）

//首先创建一个robot model
robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

//使用这个机器人模型，我们可以创建一个PlanningScene对象
planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

//现在，我们要创建一个loader，用于加载一个规划
boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;  //规划插件loader
planning_interface::PlannerManagerPtr planner_instance; //规划实例
std::string planner_plugin_name;//规划插件名字

//我们将要获得从参数服务器加载规划的名称，然后加载这个规划并且确保包括了所有的信息
if (!node_handle.getParam("planning_plugin", planner_plugin_name))
  ROS_FATAL_STREAM("Could not find planner plugin name");
try
{
  planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
}
catch(pluginlib::PluginlibException& ex)
{
  ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
}
try
{
  planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
  if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
    ROS_FATAL_STREAM("Could not initialize planner instance");
  ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
}
catch(pluginlib::PluginlibException& ex)
{
  const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
  std::stringstream ss;
  for (std::size_t i = 0 ; i < classes.size() ; ++i)
    ss << classes[i] << " ";
  ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                   << "Available plugins: " << ss.str());
}

//目标位置
planning_interface::MotionPlanRequest req;
planning_interface::MotionPlanResponse res;
geometry_msgs::PoseStamped pose;
pose.header.frame_id = "torso_lift_link";
pose.pose.position.x = 0.75;
pose.pose.position.y = 0.0;
pose.pose.position.z = 0.0;
pose.pose.orientation.w = 1.0;

//设置位置和角度的允差0.01m和0.01rad
std::vector<double> tolerance_pose(3, 0.01);
std::vector<double> tolerance_angle(3, 0.01);

//我们将会利用kinematic_constraints包里提供的函数创建一个请求
req.group_name = "right_arm";
moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", pose, tolerance_pose, tolerance_angle);
req.goal_constraints.push_back(pose_goal);//利用pose_goal创建了一个req

//我们将会创建一个规划context，它将现场，请求和相应封装起来。我们通过这个context来调用这个planner
planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
context->solve(res);
if(res.error_code_.val != res.error_code_.SUCCESS)
{
  ROS_ERROR("Could not compute plan successfully");
  return 0;
}

//关节值的目标
//首先把规划现场设置为上一个规划的末状态
robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
planning_scene->setCurrentState(response.trajectory_start);
const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("right_arm");
robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

//建立一个关节空间的目标
robot_state::RobotState goal_state(robot_model);
std::vector<double> joint_values(7, 0.0);
joint_values[0] = -2.0;
joint_values[3] = -0.2;
joint_values[5] = -0.15;
goal_state.setJointGroupPositions(joint_model_group, joint_values);
moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
req.goal_constraints.clear();
req.goal_constraints.push_back(joint_goal);//利用这个目标简历了一个请求

//调用这个规划
context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
/* Call the Planner */
context->solve(res);
/* Check that the planning was successful */
if(res.error_code_.val != res.error_code_.SUCCESS)
{
  ROS_ERROR("Could not compute plan successfully");
  return 0;
}
/* Visualize the trajectory */
ROS_INFO("Visualizing the trajectory");
res.getMessage(response);
display_trajectory.trajectory_start = response.trajectory_start;
display_trajectory.trajectory.push_back(response.trajectory);

/* Now you should see two planned trajectories in series*/
display_publisher.publish(display_trajectory);

/* We will add more goals. But first, set the state in the planning
   scene to the final state of the last plan */
robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

/* Now, we go back to the first goal*/
req.goal_constraints.clear();
req.goal_constraints.push_back(pose_goal);
context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
context->solve(res);
res.getMessage(response);//标准方式
display_trajectory.trajectory.push_back(response.trajectory);
display_publisher.publish(display_trajectory);


//添加路径约束
//新添加一个目标位置，并且添加路径约束
/* Let's create a new pose goal */
pose.pose.position.x = 0.65;
pose.pose.position.y = -0.2;
pose.pose.position.z = -0.1;
moveit_msgs::Constraints pose_goal_2 = kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", pose, tolerance_pose, tolerance_angle);

/* First, set the state in the planning scene to the final state of the last plan */
robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
/* Now, let's try to move to this new pose goal*/
req.goal_constraints.clear();
req.goal_constraints.push_back(pose_goal_2);

geometry_msgs::QuaternionStamped quaternion;
quaternion.header.frame_id = "torso_lift_link";
quaternion.quaternion.w = 1.0;
req.path_constraints = kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", quaternion);//在req中设置了路径约束

//工作空间的限制
req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = req.workspace_parameters.min_corner.z = -2.0;
req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = req.workspace_parameters.max_corner.z =  2.0;


//call the planner
context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
context->solve(res);
res.getMessage(response);
display_trajectory.trajectory.push_back(response.trajectory);
display_publisher.publish(display_trajectory);//使用rviz观看结果