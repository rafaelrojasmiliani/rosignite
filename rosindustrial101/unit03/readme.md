# Build a MoveIt Package

In this unit we learn to build a moveIt package with the movit assistant wizard.

Tasks

1. Creationg of ros packages which contains the URDF description of the robot's workspace `src/my_robot_description`

2. Creation of a **MoveIt configuration package** `src/myrobot_moveit_config`.

3. Integration of the MoveIt package into Gazebo.

## Definitions

- **MoveIt configuration package**

- **MoveIt move group** It provides an interface for most operations that a user may want to carry out, specifically setting joint or pose goals, creating motion plans, moving the robot, adding objects into the environment and attaching/detaching objects from the robot. This interface communicates over ROS topics, services, and actions to the **MoveGroup Node**.
The figure above shows the high-level system architecture for the primary node provided by MoveIt called `move_group`.
This node serves as an integrator: pulling all the individual components together to provide a set of ROS actions and services for users to use.
a move group runs a `FollowJointTrajectoryAction`

The `move_group` type of node is defined in [moveit\_ros/move\_group/src/move\_group.cpp](https://github.com/ros-planning/moveit_ros/blob/kinetic-devel/move_group/src/move_group.cpp).
In this file the clas `MoveGroupExe` is created.
The `main` of this node is [here](https://github.com/ros-planning/moveit_ros/blob/200ff00b2cad2c49811991b3af64cab5eb19f6fb/move_group/src/move_group.cpp#L148)
```
int main(int argc, char **argv)
{
  ros::init(argc, argv, move_group::NODE_NAME);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // create a shared point to a transform listener
  boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(10.0)));
  // create planning_scene_monitor pointer
  planning_scene_monitor::PlanningSceneMonitorPtr
    planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION, tf));

  if (planning_scene_monitor->getPlanningScene())
  {
      // debug parameters setup
      // ...
      // debug parameters setup
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startWorldGeometryMonitor();
    planning_scene_monitor->startStateMonitor();

    move_group::MoveGroupExe mge(planning_scene_monitor, debug);

    planning_scene_monitor->publishDebugInformation(debug);

    mge.status();

    ros::waitForShutdown();
  }
  else
    ROS_ERROR("Planning scene not configured");

  return 0;
}
```

1. Instantiate a transform listener `tf` with `boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(10.0)));`

2. Instantiate a Planning scene monitor instance `planning_scene_monitor` with `planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION, tf));`

3. `planning_scene_monitor->startSceneMonitor();`

4. `planning_scene_monitor->startWorldGeometryMonitor();`

5. `planning_scene_monitor->startStateMonitor();`

6. Instantiate an instance of `move_group::MoveGroupExe`  claled `mge` with `move_group::MoveGroupExe mge(planning_scene_monitor, debug);`.
This instance is a container of the "capabilities of move\_group".
    1. Crates a `MoveGroupContext` instance and store it in the `MoveGroupContextPtr` variable using its `MoveGroupContextPtr::reset` method.
    2. Call `MoveGroupExe::configureCapabilities();` This function will load the capabilities presented in the ROS parameter "capabilities". These capabilities are loaded as pluging using the loader `pluginlib::ClassLoader<MoveGroupCapability>` and stored in `MoveGroupExe::capabilities_`.

7. Calls `MoveGroupExe::status` with `mge.status()`. This funcions only print the status, if `mge.capabilities_.empty()` is false it prints "You can start planning now!".



## Basic use of the moveit assistan wizard

The MoveIt! Setup Assistant is a graphical user interface for configuring any robot for use with MoveIt!.
Its primary function is generating a Semantic Robot Description Format (SRDF) file for your robot.
Additionally, it generates other necessary configuration files for use with the MoveIt!
pipeline.

1. Start the moveit stetup assistant. `roslaunch moveit_setup_assistant setup_assistant.launch`.
This will launch a windows with the following taps

- Start
- Self-collisions: searches for pairs of links on the robot that can safely be disabled from collision checking, decreasing motion planning processing time.
- Virtual Joints: Sets the base of the kinematic chain.
- Planning Groups: Creates planning groups
- Robot poses
- End Effectors
- Passive Joints
- 3D perception
- Simulation
- ROS control
- Author information
- Configuration files.

2. On the self-collisions tab, we generate the self collision matrix by clicking on generate collision matrix.
Looks for links that are always in collision, never in collision, in collision in the robot’s default position or when the links are adjacent to each other on the kinematic chain.
The sampling density specifies how many random robot positions to check for self collision.

3. Virtual joints are used primarily to attach the robot to the world.
This virtual joint represents the motion of the base of the robot in a plane.
In this tab we must define a joint by indicating
- Virtual Joint name
- Child link
- Parent Frame
- Joint type

4. Create a planning group
Planning groups are used for semantically describing different parts of your robot, such as defining what an arm is, or an end effector.

    1. Click on Add Group and you should see a new screen

    2. Enter Group Name
    3. Choose the kinematics solver `kdl_kinematics_plugin/KDLKinematicsPlugin` or `Kinematics/IKFast`.
This windows will also provide text entries to set the paramenter of the kinematic solver.
    4. Click on the **Add Joints button**.
You will see a list of joints on the left hand side.
You need to choose all the joints that belong to the arm and add them to the right hand side.
The joints are arranged in the order that they are stored in an internal tree structure.
This makes it easy to select a serial chain of joints.


If we have a gripper, we have to create another planning group for the gripper.

5. Add robot poses

6. In the **End Effectors** tab  we can designate groups as a special type of group: end effectors.
Designating a group as an end effectors allows some special operations to happen on them internally.
**The ros ignite accademi does not do nothing here**.

7. In the **Configuration Files** tab. Here you will create the moveIt configuration package
    1. Choose a location and name for the ROS package that will be generated containing your new set of configuration files.
        - Click browse, select a good location.
Here you selec the folder where you will put your package
    2. Write a name in the input text.
This will be the name of the package.
Click on the Generate Package button.
The Setup Assistant will now generate and write a set of launch and config files into the directory of your choosing.

## Combining MoveIt with Gazebo using RosControl

The ROS ignite accademy has in its we interface a Gazebo node running by default.
In this module it automatically loads a sia robot.

| yaml file | scope | is it created by moveIt setup? |
| ----------- | ---------- | ------------------------ |
| `controllers.yaml` | set ros parameters | YES |
`myrobot_planning_execution.launch` | loads `joint_names.yaml`, launches `$(find myrobot_moveit_config)/launch/planning_context.launch`, launches a joint state publisher with `/source_list=[/sia10f/joint_states]`, launches `$(find myrobot_moveit_config)/launch/move_group.launch`, and `"$(find myrobot_moveit_config)/launch/moveit_rviz.launch` | NO |



| launch file | scope | is it created by moveIt setup? |
| ----------- | ---------- | ------------------------ |
| `myrobot_moveit_controller_manager.launch.xml` | loads `controllers.yaml` and set the parameters `use_controller_manager`, `trajectory_execution/execution_duration_monitoring` and `moveit_controller_manager` | YES |
`myrobot_planning_execution.launch` | loads `joint_names.yaml`, launches `$(find myrobot_moveit_config)/launch/planning_context.launch`, launches a joint state publisher with `/source_list=[/sia10f/joint_states]`, launches `$(find myrobot_moveit_config)/launch/move_group.launch`, and `"$(find myrobot_moveit_config)/launch/moveit_rviz.launch` | NO |


1. Inside the config folder of your moveit package, create a new file named `controllers.yaml`.
Copy the following content inside it:
```
controller_list:
  - name: sia10f/joint_trajectory_controller
    action_ns: "follow_joint_trajectory"
    type: FollowJointTrajectory
    joints: [joint_s, joint_l, joint_e, joint_u, joint_r, joint_b, joint_t]
```

2. Next, you'll have to create a file to define the names of the joints of your robot.
Again inside the config directory, create a new file called `joint_names.yaml`, and copy the following content in it:
```
controller_joint_names: [joint_s, joint_l, joint_e, joint_u, joint_r, joint_b, joint_t]
```

3. Now, if you open the `myrobot_moveit_controller_manager.launch.xml`, which is inside the launch directory, you'll see that it's empty.
Put the next content inside it:
```
<launch>
  <rosparam file="$(find myrobot_moveit_config)/config/controllers.yaml"/>
  <param name="use_controller_manager" value="false"/>
  <param name="trajectory_execution/execution_duration_monitoring" value="false"/>
  <param name="moveit_controller_manager" value="moveit_simple_controller_manager/MoveItSimpleControllerManager"/>
</launch>
```

4. Finally, you will have to create a new launch file that sets up all the system to control your robot.
So, inside the launch directory, create a new launch file `myrobot_planning_execution.launch`.
```
<launch>

  <rosparam command="load" file="$(find myrobot_moveit_config)/config/joint_names.yaml"/>

  <include file="$(find myrobot_moveit_config)/launch/planning_context.launch" >
    <arg name="load_robot_description" value="true" />
  </include>

  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="/use_gui" value="false"/>
    <rosparam param="/source_list">[/sia10f/joint_states]</rosparam>
  </node>

  <include file="$(find myrobot_moveit_config)/launch/move_group.launch">
    <arg name="publish_monitored_planning_scene" value="true" />
  </include>

  <include file="$(find myrobot_moveit_config)/launch/moveit_rviz.launch">
    <arg name="config" value="true"/>
  </include>

</launch>
```

The launch file in 4. is all you need to start an rviz moveit planner.

## Ros packages and launch files

- `my_robot_description` Contains a URDF description of a robot into a workspace
    - `my_robot.launch`: loads sia robot into a workspace URDF description into `/sia/robot_description`
    - `my_robot_ur.launch`: loads a Universal Robots UR5 robot into a workspace URDF description into `/sia/robot_description`

- `myrobot_moveit_config` MoveIt configuration package
    - `planning_context.launch` loads the URDF and other data (as the one in `config/*.yaml`) into the ROS parameter server
    - `planning_pipeline.launch`: launch other launch files to setup OMPL (open motion planning library).
    - `trajectory_execution.launch`
    - `myrobot_moveit_controller_manager.launch.xml`
    - `sensor_manaer.launch`
    - `move_group.launch`
```
    <node name="move_group" launch-prefix="$(arg launch_prefix)" pkg="moveit_ros_move_group" type="move_group" respawn="false" output="screen" args="$(arg command_args)"/>
```
    - `demo.lauch` runs a MoveIt demo for the created moveit configuration package.

## MoveIt configuration package launch files

- `chomp_planning_pipeline.launch.xml`
- `default_warehouse_db.launch`
    - `warehouse.launch`
- `demo_gazebo.launch`
    - `gazebo.launch`
    - `planning_context.launch`
    - `move_group.launch`
    - `moveit_rviz.launch`
    - `default_warehouse_db.launch if=$(arg db)`
- `demo.launch`
    - `planning_context.launch`
    - `move_group.launch`
    - `moveit_rviz.launch if=$(arg use_rviz)`
    - `default_warehouse_db.launch if=$(arg db)`
- `fake_moveit_controller_manager.launch.xml`
- `gazebo.launch`
    - `$(find gazebo_ros)/launch/empty_world.launch`
    - `ros_controllers.launch`
- `joystick_control.launch`
- `move_group.launch`
    - `planning_context.launch`
    - `planning_pipeline.launch.xml ns="move_group"` and `pipeline="ompl"`
    - `trajectory_execution.launch.xml if="$(arg allow_trajectory_execution)" ns="move_group"`
    - `sensor_manager.launch.xml if="$(ar allow_trajectory_execution)" ns="move_group"`
- `moveit.rviz`
- `moveit_rviz.launch`
- `myrobot_moveit_controller_manager.launch.xml`
- `myrobot_moveit_sensor_manager.launch.xml`
- `myrobot_planning_execution.launch`
    - `planning_context.launch`
    - `move_group.launch`
    - `moveit_rviz.launch`
- `ompl_planning_pipeline.launch.xml`
- `planning_context.launch`
- `planning_pipeline.launch.xml`
    - `$(arg pipeline)_planning_pipeline.launch.xml`
- `ros_controllers.launch`
- `run_benchmark_ompl.launch`
    - `planning_context.launch`
    - `warehouse.launch`
- `sensor_manager.launch.xml`
    - `$(arg moveit_sensor_manager)_moveit_sensor_manager.launch.xml` with `moveit_sensor_manager=myrobot`
- `setup_assistant.launch`
- `trajectory_execution.launch.xml`
    - `$(arg moveit_controller_manager)_moveit_controller_manager.launch.xml` with `moveit_controller_manager=myrobot`
- `warehouse.launch`
    - `warehouse_settings.launch.xml`
- `warehouse_settings.launch.xml`


# `move_group` actions


- `moveit_msgs/ExecuteTrajectoryAction`
- `moveit_msgs/MoveGroupAction`
- `moveit_msgs/PickupAction`
- `moveit_msgs/PlaceAction`
