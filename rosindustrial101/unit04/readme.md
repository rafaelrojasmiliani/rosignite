# Motion Planning with Python


This Unit will show you how to perform motion planning with Python.
By completing this Unit, you will be able to create a Python program that performs motion planning on your robot

Tasks

1. Use the packages created in the previous unit:
    - Robot workspace geometrical description with URDF `my_robot_description`
    - MoveIt configuration package `myrobot_moveit_config` with integration with gazebo.
This requires the launch file `myrobot_planning_execution.launch`

2. Implementation of **MoveIt**  in python


## move\_commander interface

The **moveit\_commander** Python package offers wrappers for the functionality provided in MoveIt!.
The `moveit_commander` package also includes a command line interface, `moveit_commander_cmdline.py`.
The github code is available [here](https://github.com/ros-planning/moveit/tree/master/moveit_commander).

`moveit_commander` has the following modules

| module name | description |
| ----------- | ----------- |
|`exception`  | declares a default exception for this module |
|`roscpp_initializer ` | wrapper for `_moveit_roscpp_initializer` [implemented here](https://github.com/ros-planning/moveit/blob/cf218879dbc23aadf88dd56b8abe7970b7d61030/moveit_ros/planning_interface/py_bindings_tools/src/roscpp_initializer.cpp#L127) |
|`planning_scene_interface ` | wrapper for `_moveit_planning_scene_interface` [implemented here](https://github.com/ros-planning/moveit/blob/cf218879dbc23aadf88dd56b8abe7970b7d61030/moveit_ros/planning_interface/planning_scene_interface/src/planning_scene_interface.cpp#L50)|
|`move_group ` | wrapper for `_moveit_move_group_interface` [implemented here](https://github.com/ros-planning/moveit/blob/cf218879dbc23aadf88dd56b8abe7970b7d61030/moveit_ros/planning_interface/move_group_interface/src/move_group_interface.cpp#L93) |
|`robot ` | wrapper for `_moveit_robot_interface` [implemented here](https://github.com/ros-planning/moveit/blob/cf218879dbc23aadf88dd56b8abe7970b7d61030/moveit_ros/planning_interface/robot_interface/src/wrap_python_robot_interface.cpp#L57) |
|`interpreter ` | implements the class `MoveGroupCommandInterpreter` to translate simple commands |


All the modules in `moveit_commander` make reference to the python package `moveit_ros_planning_interface`. This is an empty python package which contains the following cpython modules
- `_moveit_roscpp_initializer`: code in `moveit/moveit_ros/planning_interface/py_bindings_tools/`
    - `moveit_commander.roscpp_initialize(sys.argv)` runs an async roscpp spinner as in [here](https://github.com/ros-planning/moveit/blob/cf218879dbc23aadf88dd56b8abe7970b7d61030/moveit_ros/planning_interface/py_bindings_tools/src/roscpp_initializer.cpp#L127)
- `_moveit_move_group_interface`
- `_moveit_planning_scene_interface`
- `_moveit_robot_interface`


## `trajectory_execution_manager`

MoveIt includes a library for managing controllers and the execution of trajectories.
This code exists in the `trajectory_execution_manager` namespace.
The class `TrajectoryExecutionManager` is [defined here](https://github.com/ros-planning/moveit/blob/7ad2bc7b86dad08061d98668ba34feba54bb05cc/moveit_ros/planning/trajectory_execution_manager/include/moveit/trajectory_execution_manager/trajectory_execution_manager.h#L59) and [implemented here](https://github.com/ros-planning/moveit/blob/7ad2bc7b86dad08061d98668ba34feba54bb05cc/moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp#L57).
`TrajectoryExecutionManager` implements a [dynamic reconfigure server](http://wiki.ros.org/dynamic_reconfigure) as the private class `TrajectoryExecutionManager::DynamicReconfigureImpl` [here](https://github.com/ros-planning/moveit/blob/7ad2bc7b86dad08061d98668ba34feba54bb05cc/moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp#L63).

The generation of the headers for dynamic reconfigure of the `moveit_ros_planning` package [are here](https://github.com/ros-planning/moveit/blob/9f9e69c54529e1aa156851abb6aae7047b20780d/moveit_ros/planning/CMakeLists.txt#L36).
[This dynamic reconfigure file-generator](https://github.com/ros-planning/moveit/blob/master/moveit_ros/planning/trajectory_execution_manager/cfg/TrajectoryExecutionDynamicReconfigure.cfg) generates  `moveit_ros_planning/TrajectoryExecutionDynamicReconfigureConfig.h` with the class `moveit_ros_planning::TrajectoryExecutionDynamicReconfigureConfig`.
This allows to automatically set many parameters.


The `trajectory_execution_manager::TrajectoryExecutionManager` class allows two main operations:

1. `trajectory_execution_manager::TrajectoryExecutionManager::push()` [implemented here](https://github.com/ros-planning/moveit/blob/7ad2bc7b86dad08061d98668ba34feba54bb05cc/moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp#L262) with various overloads.
Its main implementation is
```C++
bool TrajectoryExecutionManager::push(const moveit_msgs::RobotTrajectory& trajectory,
                                      const std::vector<std::string>& controllers)
```
adds trajectories specified as a `moveit_msgs::RobotTrajectory` message type to a queue of trajectories to be executed in sequence.
This is done by wrapping the inputs `std::vector<moveit_msgs::RobotTrajectory> trajectory` and `std::vector<std::string> controllers` into a simple `struct` [defined here](https://github.com/ros-planning/moveit/blob/a29a30caaecbd130d85056d959d4eb1c30d4088f/moveit_ros/planning/trajectory_execution_manager/include/moveit/trajectory_execution_manager/trajectory_execution_manager.h#L73) which is created with `TrajectoryExecutionManager::configure` [here](https://github.com/ros-planning/moveit/blob/a29a30caaecbd130d85056d959d4eb1c30d4088f/moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp#L1032) and pushed into `TrajectoryExecutionManager::trajectories_`.
Each trajectory can be specified for any set of joints in the robot.
Because controllers may only be available for certain groups of joints, this function may decide to split one trajectory into multiple ones and pass them to corresponding controllers (this time in parallel, using the same time stamp for the trajectory points).
This approach assumes that controllers respect the time stamps specified for the waypoints.

2. `trajectory_execution_manager::TrajectoryExecutionManager::execute()` passes the appropriate trajectories to different controllers, monitors execution, optionally waits for completion of the execution and, very importantly, switches active controllers as needed (optionally) to be able to execute the specified trajectories.
This function has different overloaders, but its main definition [is here](https://github.com/ros-planning/moveit/blob/a29a30caaecbd130d85056d959d4eb1c30d4088f/moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp#L1208)

```C++
void TrajectoryExecutionManager::execute(const ExecutionCompleteCallback& callback,
                                         const PathSegmentCompleteCallback& part_callback, bool auto_clear)
```

This method starts a `boost::thread` [here](https://github.com/ros-planning/moveit/blob/a29a30caaecbd130d85056d959d4eb1c30d4088f/moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp#L1226) which executes `TrajectoryExecutionManager::executeThread` [implemented here](https://github.com/ros-planning/moveit/blob/a29a30caaecbd130d85056d959d4eb1c30d4088f/moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp#L1269) that is defined as

```C++
void TrajectoryExecutionManager::executeThread(const ExecutionCompleteCallback& callback,
                                               const PathSegmentCompleteCallback& part_callback, bool auto_clear)
```

the arguments `callback` and `part_callback` can be defined by the user and their defaults are implementations of `boost::function` [see here](https://theboostcpplibraries.com/boost.function) [defined here](https://github.com/ros-planning/moveit/blob/a29a30caaecbd130d85056d959d4eb1c30d4088f/moveit_ros/planning/trajectory_execution_manager/include/moveit/trajectory_execution_manager/trajectory_execution_manager.h#L66) and [here](https://github.com/ros-planning/moveit/blob/a29a30caaecbd130d85056d959d4eb1c30d4088f/moveit_ros/planning/trajectory_execution_manager/include/moveit/trajectory_execution_manager/trajectory_execution_manager.h#L70).
The main functionality of `TrajectoryExecutionManager::executeThread` is to call `TrajectoryExecutionManager::executePart` [here](https://github.com/ros-planning/moveit/blob/a29a30caaecbd130d85056d959d4eb1c30d4088f/moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp#L1290).

`TrajectoryExecutionManager::executePart` is [implemented here](https://github.com/ros-planning/moveit/blob/a29a30caaecbd130d85056d959d4eb1c30d4088f/moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp#L1321) as
```C++
bool TrajectoryExecutionManager::executePart(std::size_t part_index)
```
This function executes the trajectory encapsulated in the trajectory context `trajectories_[part_index]` in something like
```C++
context =  trajectories_[part_index]
for(std::size_t i = 0; i < context.controllers_.size(); ++i)
    handles[i] = controller_manager_->getControllerHandle(context.controllers_[i]); //context.controllers_[i] is a string
    handles[i]->sendTrajectory(context.trajectory_parts_[i])

for (moveit_controller_manager::MoveItControllerHandlePtr& handle : handles)
        if (execution_duration_monitoring_)
           // stops the trajectory execution if it takes too long
        else
            handle->waitForExecution();
```

- **Variables**
    - `std::vector<TrajectoryExecutionContext*> trajectories_;`
    - `std::deque<TrajectoryExecutionContext*> continuous_execution_queue_;`
    - `std::unique_ptr<pluginlib::ClassLoader<moveit_controller_manager::MoveItControllerManager> > controller_manager_loader_;`
    - `moveit_controller_manager::MoveItControllerManagerPtr     controller_manager_;`
    - `bool manage_controllers_;` This may be defined in the `TrajectoryExecutionManager` constructor.
Otherwise this is read from the ROS parameter `moveit_manage_controllers`.

- **Parameters**
    - `moveit_manage_controllers`
    - `moveit_controller_manager`. `TrajectoryExecutionManager` stores the controller manager specified in `moveit_controller_manager` into an instance of  `moveit_controller_manager::MoveItControllerManager` in a way similar to
    ```C++
    std::string controller;
    node_handle_.getParam("moveit_controller_manager", controller);
    controller_manager_ = controller_manager_loader_->createUniqueInstance(controller);
    ```
    - `controller_list`
    - `~/trajectory_execution/execution_duration_monitoring` (with dynamic reconfigure)
    - `~/trajectory_execution/allowed_execution_duration_scaling` (with dynamic reconfigure)
    - `~/trajectory_execution/allowed_goal_duration_margin` (with dynamic reconfigure)
    - `~/trajectory_execution/execution_velocity_scaling` (with dynamic reconfigure)
    - `~/trajectory_execution/allowed_start_tolerance` (with dynamic reconfigure)
    - `~/trajectory_execution/wait_for_trajectory_completion` (with dynamic reconfigure)

- **Topics**
    - Subscribes to `trajectory_execution_event` with callback `TrajectoryExecutionManager::receiveEvent`.
The method `TrajectoryExecutionManager::receiveEvent` does just one operation: it calls `TrajectoryExecutionManager::processEvent`.

- **Functions**: `TrajectoryExecutionManager::initialize`
    1. calls `TrajectoryExecutionManager::loadControllerParams`
    ```C++
    XmlRpc::XmlRpcValue controller_list;
    node_handle_.getParam("controller_list", controller_list)
    for (const XmlRpc::XmlRpcValue& controller : controller_list)
    {
        controller_allowed_execution_duration_scaling_[std::string(controller["name"])] =
              controller["allowed_execution_duration_scaling"];
        controller_allowed_goal_duration_margin_[std::string(controller["name"])] =
              controller["allowed_goal_duration_margin"];
    }
    ```
    2. Load the parameters

    ```C++
    void *myplug = new pluginlib::ClassLoader<moveit_controller_manager::MoveItControllerManager>("moveit_core", "moveit_controller_manager::MoveItControllerManager")

    controller_manager_loader_.reset(myplug);
    node_handle_.getParam("moveit_controller_manager", controller)
    const std::vector<std::string>& classes = controller_manager_loader_->getDeclaredClasses();
    controller = classes[0];

    if (!controller.empty())
        controller_manager_ = controller_manager_loader_->createUniqueInstance(controller);
    ```
    3. Calls `TrajectoryExecutionManager::reloadControllerInformation`.
This function initializes the vector `std::map<std::string, ControllerInformation> known_controllers_;`.
The class `TrajectoryExecutionManager::ControllerInformation` is [defined here](https://github.com/ros-planning/moveit/blob/7ad2bc7b86dad08061d98668ba34feba54bb05cc/moveit_ros/planning/trajectory_execution_manager/include/moveit/trajectory_execution_manager/trajectory_execution_manager.h#L246).

    4. Subscribes to `trajectory_execution_event` with callback `TrajectoryExecutionManager::receiveEvent`

    5. Instantiantes `TrajectoryExecutionManager::DynamicReconfigureImpl` for the current instance of `TrajectoryExecutionManager`.
`TrajectoryExecutionManager::DynamicReconfigureImpl` is [implemented here](https://github.com/ros-planning/moveit/blob/7ad2bc7b86dad08061d98668ba34feba54bb05cc/moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp#L57)


- **Function**: `TrajectoryExecutionManager::receiveEvent` and `TrajectoryExecutionManager::processEvent` (callback for `trajectory_execution_event`)
    1. `TrajectoryExecutionManager::receiveEvent` calls `TrajectoryExecutionManager::processEvent`.
    2. Stops the trajectory
```C++
  if (event == "stop")
    stopExecution(true);
  else
    ROS_WARN_STREAM_NAMED(name_, "Unknown event type: '" << event << "'");
```
## MoveIt controller manager

MoveIt does not enforce how controllers are implemented.
To make your controllers usable by MoveIt, this interface needs to be implemented.
The main purpose of this interface is to expose the set of known controllers and potentially to allow activating and deactivating them, if multiple controllers are available.

`moveit_controller_manager::MoveItControllerManager` is an abstract class that defines the functionality needed by `trajectory_execution_manager::TrajectoryExecutionManager::execute` and needs to be implemented for each robot type.
Often, the implementation of these plugins are quite similar and it is easy to modify existing code to achieve the desired functionality.

MoveIt controller managers, somewhat a misnomer, are the interfaces to your custom low level controllers.
A better way to think of them are controller interfaces.
For most use cases, the included [MoveItSimpleControllerManager](https://github.com/ros-planning/moveit/blob/master/moveit_plugins/moveit_simple_controller_manager) is sufficient if your robot controllers already provide ROS actions for FollowJointTrajectory.
If you use ros_control, the included [MoveItRosControlInterface](https://github.com/ros-planning/moveit/blob/master/moveit_plugins/moveit_ros_control_interface) is also ideal.

However, for some applications you might desire a more custom controller manager.
An example template for starting your custom controller manager is provided here.

There are two different **abstract classes** called `MoveItControllerManager`

- `moveit_controller_manager::MoveItControllerManager` from the `moveit_core` package [defined here](https://github.com/ros-planning/moveit_core/blob/a8cd90b2d4798f67ccc07c5f24ea52b0f9539f51/controller_manager/include/moveit/controller_manager/controller_manager.h#L144)

- `moveit_ros_control_interface::MoveItControllerManager` from the `moveit_plugins` package [defined and implemented here in the same file](https://github.com/ros-planning/moveit_plugins/blob/cf0ddc86cf843688c8d172cf233a5d0e63e7f9de/moveit_ros_control_interface/src/controller_manager_plugin.cpp#L79).
`moveit_ros_control_interface::MoveItControllerManager` inherits from `moveit_controller_manager::MoveItControllerManager`.


### MoveIt Simple Controller manager

The `MoveItSimpleControllerManager` is [defined and implemented here](https://github.com/ros-planning/moveit/blob/7ad2bc7b86dad08061d98668ba34feba54bb05cc/moveit_plugins/moveit_simple_controller_manager/src/moveit_simple_controller_manager.cpp)

- **Variables**
    - `std::map<std::string, ActionBasedControllerHandleBasePtr> controllers_;`
    - `std::map<std::string, moveit_controller_manager::MoveItControllerManager::ControllerState> controller_states_;`

- **Function** Constructor
    ```C++

    XmlRpc::XmlRpcValue controller_list;
    node_handle_.getParam("controller_list", controller_list);

    /* actually create each controller */
    for (int i = 0; i < controller_list.size(); ++i){
        name = controller_list[i]["name"];
        action_ns = controller_list[i]["action_ns"];
        type = controller_list[i]["type"];

        ActionBasedControllerHandleBasePtr new_handle;
        if (type == "GripperCommand"){ ...}
        else if (type == "FollowJointTrajectory")
        {
          auto h = new FollowJointTrajectoryControllerHandle(name, action_ns);
          new_handle.reset(h);
        }
        else{
          ROS_ERROR_STREAM_NAMED(LOGNAME, "Unknown controller type: " << type.c_str());
        }

        moveit_controller_manager::MoveItControllerManager::ControllerState state;
        // initialiaze moveit_controller_manager
        this->controller_states_[name] = state;

        for (int j = 0; j < controller_list[i]["joints"].size(); ++j)
          new_handle->addJoint(std::string(controller_list[i]["joints"][j]));

        new_handle->configure(controller_list[i]);
      }
    ```
### `FollowJointTrajectoryControllerHandle`

The class `FollowJointTrajectoryControllerHandle` is [defined here](https://github.com/ros-planning/moveit/blob/master/moveit_plugins/moveit_simple_controller_manager/include/moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h) and [implemented here](https://github.com/ros-planning/moveit/blob/master/moveit_plugins/moveit_simple_controller_manager/src/follow_joint_trajectory_controller_handle.cpp).
This class inherits from `ActionBasedControllerHandle<control_msgs::FollowJointTrajectoryAction>`.
This class wraps `ActionBasedControllerHandle<control_msgs::FollowJointTrajectoryAction>` and implements the `sendTrajectory` function.
`ActionBasedControllerHandle` receives the namespace of the `FollowJointTrajectoryAction` in its constructor which is called with the constructor
```
FollowJointTrajectoryControllerHandle(const std::string& name, const std::string& action_ns)
```

- **Function** `sendTrajectory`
    ```
    bool FollowJointTrajectoryControllerHandle::sendTrajectory(const moveit_msgs::RobotTrajectory& trajectory){
          control_msgs::FollowJointTrajectoryGoal goal = goal_template_;
  goal.trajectory = trajectory.joint_trajectory;
  controller_action_client_->sendGoal(
      goal, boost::bind(&FollowJointTrajectoryControllerHandle::controllerDoneCallback, this, _1, _2),
      boost::bind(&FollowJointTrajectoryControllerHandle::controllerActiveCallback, this),
      boost::bind(&FollowJointTrajectoryControllerHandle::controllerFeedbackCallback, this, _1));
  done_ = false;
  last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    }
    ```

- **Variables**
    - `control_msgs::FollowJointTrajectoryGoal goal_template_`

- **Actions**
    - [joint\_trajectory\_action](http://wiki.ros.org/joint_trajectory_action)


### `ActionBasedControllerHandle`

This is an abstract template class [defined here](https://github.com/ros-planning/moveit/blob/802e596a2283b64f4582d802c5f79e1f3d57def0/moveit_plugins/moveit_simple_controller_manager/include/moveit_simple_controller_manager/action_based_controller_handle.h).

- **Variables**
    - `std::shared_ptr<actionlib::SimpleActionClient<T> > controller_action_client_;`


## Procedure
1. We create a new package with
```
catkin_create_pkg my_motion_scripts std_msgs geometry_msgs rospy
```

2. Write the following python code in `myrobot_python/src/python_moveit.py`
```python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")
    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = 0.3
    pose_target.position.y = 0
    pose_target.position.z = 1.1
    group.set_pose_target(pose_target)

    plan1 = group.plan()

    rospy.sleep(5)

    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
```

## Code

1. `moveit_commander.roscpp_initialize(sys.argv)` Initialize an asynchronous spinner
2. `robot = moveit_commander.RobotCommander()`. `RobotCommander` is a class defined in `moveit_commander.robot`.
This class is a wrapper for `moveit::RobotInterfacePython` [implemented here](https://github.com/ros-planning/moveit/blob/cf218879dbc23aadf88dd56b8abe7970b7d61030/moveit_ros/planning_interface/robot_interface/src/wrap_python_robot_interface.cpp#L57).
This class inherits from `py_bindings_tools::ROScppInitializer` [defined here](https://github.com/ros-planning/moveit/blob/cf218879dbc23aadf88dd56b8abe7970b7d61030/moveit_ros/planning_interface/py_bindings_tools/include/moveit/py_bindings_tools/roscpp_initializer.h#L51) and [implemented here](https://github.com/ros-planning/moveit/blob/cf218879dbc23aadf88dd56b8abe7970b7d61030/moveit_ros/planning_interface/py_bindings_tools/src/roscpp_initializer.cpp#L144).
The class `moveit::RobotInterfacePython` is a (asynchronous) code which represents the robot (the virtual twin?).
Their main members are `moveit::core::RobotModelConstPtr robot_model_`  and `planning_scene_monitor::CurrentStateMonitorPtr current_state_monitor_`.
For example `moveit::RobotInterfacePython::getRobotRootLink()` is just a call to `moveit::core::RobotModelConstPtr::getRootLinkName()`.
Another example `moveit::RobotInterfacePython::getRobotMarkersPythonDictList()` is a wrapper to a a call to `moveit::CurrentStateMonitorPtr::getCurrentState`.

3. `scene = moveit_commander.PlanningSceneInterface()`. The class `PlanningSceneInterface` is a python wrapper for `moveit::planning_interface::PlanningSceneInterace` [defined here](https://github.com/ros-planning/moveit/blob/cf218879dbc23aadf88dd56b8abe7970b7d61030/moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h#L52) and [implemented here](https://github.com/ros-planning/moveit/blob/cf218879dbc23aadf88dd56b8abe7970b7d61030/moveit_ros/planning_interface/planning_scene_interface/src/planning_scene_interface.cpp#L282).
This ckass is a wrapper for services of `moveit_msgs::GetPlanningScene` [defined here](http://docs.ros.org/en/api/moveit_msgs/html/srv/GetPlanningScene.html) and `moveit_msgs::ApplyPlanningScene` [defined here](http://docs.ros.org/en/api/moveit_msgs/html/srv/ApplyPlanningScene.html)

4. `group = moveit_commander.MoveGroupCommander("manipulator")`.
This line creates an instance of the class `moveit_commander.move_group.MoveGroupCommander`, which is a wrapper for `_moveit_move_group_interface.MoveGroupInterface` [defined here](https://github.com/ros-planning/moveit/blob/cf218879dbc23aadf88dd56b8abe7970b7d61030/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h#L99) and [implemented here](https://github.com/ros-planning/moveit/blob/cf218879dbc23aadf88dd56b8abe7970b7d61030/moveit_ros/planning_interface/move_group_interface/src/move_group_interface.cpp#L1314).
    - **Services**
        - `moveit_msgs::QueryPlannerInterfaces` in
        - `moveit_msgs::GetPlannerParams`
        - `moveit_msgs::SetPlannerParams`
        - `moveit_msgs::GetCartesianPath`
        - `moveit_msgs::GraspPlanning`

    - **Topics**
        - published `moveit_msgs::AttachedCollisionObject`
        - published `trajectory_execution_manager::TrajectoryExecutionManager::EXECUTION_EVENT_TOPIC`


## The `move_group` node

The `move_group` node is implemented here `https://github.com/ros-planning/moveit/blob/master/moveit_ros/move_group/src/move_group.cpp`

- **Publicated Topics**
    - `/move_group/display_contacts [visualization_msgs/MarkerArray]`
    - `/move_group/display_cost_sources [visualization_msgs/MarkerArray]`
    - `/move_group/display_grasp_markers [visualization_msgs/MarkerArray]`
    - `/move_group/display_planned_path [moveit_msgs/DisplayTrajectory]`
    - `/move_group/filtered_cloud [sensor_msgs/PointCloud2]`
    - `/move_group/monitored_planning_scene [moveit_msgs/PlanningScene]`
    - `/move_group/motion_plan_request [moveit_msgs/MotionPlanRequest]`
    - `/move_group/ompl/parameter_descriptions [dynamic_reconfigure/ConfigDescription]`
    - `/move_group/ompl/parameter_updates [dynamic_reconfigure/Config]`
    - `/move_group/plan_execution/parameter_descriptions [dynamic_reconfigure/ConfigDescription]`
    - `/move_group/plan_execution/parameter_updates [dynamic_reconfigure/Config]`
    - `/move_group/planning_scene_monitor/parameter_descriptions [dynamic_reconfigure/ConfigDescription]`
    - `/move_group/planning_scene_monitor/parameter_updates [dynamic_reconfigure/Config]`
    - `/move_group/sense_for_plan/parameter_descriptions [dynamic_reconfigure/ConfigDescription]`
    - `/move_group/sense_for_plan/parameter_updates [dynamic_reconfigure/Config]`
    - `/move_group/trajectory_execution/parameter_descriptions [dynamic_reconfigure/ConfigDescription]`
    - `/move_group/trajectory_execution/parameter_updates [dynamic_reconfigure/Config]`

- **Subscribed Topics**
    - `/head_mount_kinect/depth_registered/points [unknown type]`
    - `/trajectory_execution_event [unknown type]`
    - `/tf [tf2_msgs/TFMessage]`
    - `/tf_static [tf2_msgs/TFMessage]`
    - `/joint_states [sensor_msgs/JointState]`
    - `/attached_collision_object [unknown type]`
    - `/collision_object [unknown type]`
    - `/planning_scene [moveit_msgs/PlanningScene]`
    - `/planning_scene_world [moveit_msgs/PlanningSceneWorld]`


- **Actions offered** (in the sense that `move_group` subscribes to `namespace/goal`)
    - `moveit_msgs/ExecuteTrajectoryAction` exposed in `/execute_trajectory/goal`
    - `moveit_msgs/MoveGroupAction` exposed in `/move_group/goal`
    - `moveit_msgs/PickupAction` exposed in `/pickup/goal`
    - `moveit_msgs/PlaceAction` exposed in `/place/goal`


The `main` function [is here](https://github.com/ros-planning/moveit/blob/f85db808303cb7787320dc48162eabdc07593fdc/moveit_ros/move_group/src/move_group.cpp#L182) is something like

```C++
int main(int argc, char **argv) {
  ros::init(argc, argv, move_group::NODE_NAME);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;

  tf_buffer = std::make_shared<tf2_ros::Buffer>(ros::Duration(10.0));
  // subscrive to /tf and /tf_static, make that info available at tf_buffer
  tfl = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, nh);

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(
      new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION,
                                                       tf_buffer));

  bool debug = read_from_args("--debug");
  // subscribe to planning_scene topic of type `moveit_msgs::PlanningScene`
  planning_scene_monitor->startSceneMonitor();
  // subscribe to collision_object of type  `moveit_msgs::CollisionObject`
  // subscribe to planning_scene_world of type `moveit_msgs::PlanningScene`
  planning_scene_monitor->startWorldGeometryMonitor();
  // subscrive to joint_states with the member `PlanningSceneMonitor::current_state_monitor_` of type `planning_scene_monitor::CurrentStateMonitor`
  // subscribe to `attached_collision_object` of type `moveit_msgs::AttachedCollisionObject`
  planning_scene_monitor->startStateMonitor();

  move_group::MoveGroupExe mge(planning_scene_monitor, debug);

  planning_scene_monitor->publishDebugInformation(debug);

  mge.status();

  ros::waitForShutdown();

  return 0;
}
```


## Move Group Context

The class `move_group::MoveGroupContext` is [defined here](https://github.com/ros-planning/moveit/blob/f85db808303cb7787320dc48162eabdc07593fdc/moveit_ros/move_group/include/moveit/move_group/move_group_context.h) and [implemented here](https://github.com/ros-planning/moveit/blob/f85db808303cb7787320dc48162eabdc07593fdc/moveit_ros/move_group/src/move_group_context.cpp).
This class requires a `PlanningSceneMonitor` in its constructor.

- **Functions**
    - `MoveGroupContext`
    ```C++
    MoveGroupContext(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, bool allow_trajectory_execution = false, bool debug = false):
     planning_scene_monitor_(planning_scene_monitor)
      , allow_trajectory_execution_(allow_trajectory_execution)
      , debug_(debug)
    {
      planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(planning_scene_monitor_->getRobotModel()));

      if (allow_trajectory_execution_)
      {
        trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(
            planning_scene_monitor_->getRobotModel(), planning_scene_monitor_->getStateMonitor()));
        plan_execution_.reset(new plan_execution::PlanExecution(planning_scene_monitor_, trajectory_execution_manager_));
        plan_with_sensing_.reset(new plan_execution::PlanWithSensing(trajectory_execution_manager_));
        if (debug)
          plan_with_sensing_->displayCostSources(true);
      }
      planning_pipeline_->displayComputedMotionPlans(true);
      planning_pipeline_->checkSolutionPaths(true);

      if (debug_)
        planning_pipeline_->publishReceivedRequests(true);
    }
    ```
    - `bool status() const;` returns if `MoveGroupContext::planner_interface` is null or not.

- **Variables**
    - `planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;` Input to the constructor.
    - `trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;`
    - `planning_pipeline::PlanningPipelinePtr planning_pipeline_;`
    - `plan_execution::PlanExecutionPtr plan_execution_;`
    - `plan_execution::PlanWithSensingPtr plan_with_sensing_;`
    - `bool allow_trajectory_execution_;`
    - `bool debug_;`



## Motion Request message and `MotionRequest` class
The most flexible way to interact with motion planning at the ROS level is through the `GetMotionPlan` and `ComputePlanningBenchmark` services. 
The ROS services for motion planning share most of the request part of the service: the MotionPlanRequest message.

### The `MotionPlanRequest` Message
The content of this message may look intimidating, but most of its fields are optional:

    - `group_name [string]` (**required**) This is the name of the group to plan for. This member is required and must be a group name defined in the SRDF.
    - `start_state [moveit_msgs/RobotState]` (**optional**) This is the state to assume the robot starts at. This member does not need to be specified. 
    - `goal_constraints [moveit_msgs/Constraints]` (**required**) Instead of defining a goal state, goals are specified in a more generic fashion: in terms of constraints.
    - `path_constraints [moveit_msgs/Constraints]` (**optional**) These are optional constraints that are to be imposed along the solution path. All the states along the solution path must satisfy these constraints. 
    - `planner_id [string]` (**optional**)  is specified, the library will use the planning algorithm identified by the specified.
    - `workspace_parameters [moveit_msgs/WorkspaceParameters]` (**required**) define a bounding box in R^3.
    - `num_planning_attempts [int32]` (**required**) for algorithms that use randomization in their execution, it is likely that different planner executions will produce different solutions. Setting this parameter to a value above 1 has the effect of running that many additional motion plans (perhaps in parallel), and obtaining multiple solutions. The shortest solution is then reported by the planner as the final result.
    - `allowed_planning_time` (**required**) Each planner execution needs a time limit; this parameter defaults to 1 if not specified, and indicates the number of seconds allowed to plan.

### The GetMotionPlan service MotionPlanResponse

    - `trajectory [moveit_msgs/RobotTrajectory]` This is an array of `trajectory_msgs/JointTrajectoryPoint` which is a struct with four arrays `float64[]` for position, velocity, acceleration and effort. These are the points along the solution path. Only joints for the group planned for are included.
    - `trajectory_start [moveit_msgs/RobotState]` __The computed trajectory does not include all robot joints; as such, the trajectory is not complete, since we do not know the values for the links that were not included in planning. For this reason, the full robot state is included__. This state is usually the robot's current state (as in the planning scene set for planning), but with values specified by the `MotionPlanRequest.start_state` overwritten.
    - `planning_time` This is the amount of time the planner took to compute the motion plan
    - `error_code` The status code reported after planning.

