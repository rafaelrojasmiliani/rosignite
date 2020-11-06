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

## `PlanningSceneMonitor`

`PlanningSceneMonitor` is [defined here](https://github.com/ros-planning/moveit/blob/47884198c2585215de8f365a7ff20479f8bb4b51/moveit_ros/planning/planning_scene_monitor/include/moveit/planning_scene_monitor/planning_scene_monitor.h#L61) and [implemented here](https://github.com/ros-planning/moveit/blob/47884198c2585215de8f365a7ff20479f8bb4b51/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp).
In its construction, this runs a `ros::AsyncSpinner`.

- **Function** `PlanningSceneMonitor::startSceneMonitor` by default it creates a subscriber to `planning_scene` with callback `PlanningSceneMonitor::newPlanningSceneCallback`

- **Function** `PlanningSceneMonitor::newPlanningSceneCallback`, just calls `PlanningSceneMonitor::newPlanningSceneMessage`

- **Function** `PlanningSceneMonitor::newPlanningSceneMessage`. Uses the tools from the package `moveit_ros_occupancy_map_monitor` and the class `occupancy_map_monitor::OccupancyMapMonitor` to update the scene.

- **Function** `PlanningSceneMonitor::startWorldGeometryMonitor` by default it creates a subscriber to `collision_object` with callback `PlanningSceneMonitor::collisionObjectCallback` and a subscriber to `planning_scene_world` with callback `PlanningSceneMonitor::newPlanningSceneWorldCallback`

- **Function** `PlanningSceneMonitor::collisionObjectCallback` 
- **Function** `PlanningSceneMonitor::newPlanningSceneWorldCallback` 




- **Subscribed topics**
    - `planning_scene` of type `moveit_msgs::PlanningScene` [defined here](http://docs.ros.org/en/api/moveit_msgs/html/msg/PlanningScene.html). Thus subscriber is instantiated at `PlanningSceneMonitor::startSceneMonitor`.
On this message the function `PlanningSceneMonitor::newPlanningSceneMessage` is called.
    - `collision_object` of type `moveit_msgs::CollisionObject` [defined here](http://docs.ros.org/en/jade/api/moveit_msgs/html/msg/CollisionObject.html) with callback `PlanningSceneMonitor::collisionObjectCallback`. This subscriber is instantiated at `PlanningSceneMonitor::startWorldGeometryMonitor`
    - `planning_scene_world` of type `moveit_msgs::PlanningScene` with callback `PlanningSceneMonitor::newPlanningSceneWorldCallback`.  This subscriber is instantiated at `PlanningSceneMonitor::startWorldGeometryMonitor`
    - `attached_collision_object` of type `moveit_msgs::AttachedCollisionObject` [defined here](http://docs.ros.org/en/jade/api/moveit_msgs/html/msg/AttachedCollisionObject.html) with callback `PlanningSceneMonitor::attachObjectCallback`
    - `joint_states` subscribed by `PlanningSceneMonitor::current_state_monitor_` of type `CurrentStateMonitorPtr` with callback `planning_scene_monitor::CurrentStateMonitor::jointStateCallback` [implemented here](https://github.com/ros-planning/moveit/blob/382aa5a8cdd39eace07536d39c497a4b21f0f653/moveit_ros/planning/planning_scene_monitor/src/current_state_monitor.cpp#L336).

- **Published topics**
    - `monitored_planning_scene` of type `moveit_msgs::PlanningScene`.

- **Required services**
    - **Optional?** `get_planning_scene` of type `moveit_msgs::GetPlanningScene` [defined here](http://docs.ros.org/en/api/moveit_msgs/html/srv/GetPlanningScene.html).
This service is called by `PlanningSceneMonitor::requestPlanningSceneState`

- **Offered services**
    - **Optional?** `get_planning_scene`with callback `PlanningSceneMonitor::getPlanningSceneServiceCallback` optional service for getting the complete planning scene.
This is useful for satisfying the Rviz PlanningScene display's need for a service **without having to use a `move_group` node**.
 _Be careful not to use this in conjunction with `PlanningSceneMonitor::requestPlanningSceneState`_, as it will create a pointless feedback loop
This service is initiated by `PlanningSceneMonitor::providePlanningSceneService`

    - `tf2_frames` through `PlanningSceneMonitor::tf_buffer_`

    - ` `
- **Threads**
    - `PlanningSceneMonitor::scenePublishingThread` [implemented here](https://github.com/ros-planning/moveit/blob/47884198c2585215de8f365a7ff20479f8bb4b51/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp#L334)


- **Members**
    - `planning_scene::PlanningScenePtr scene_;` Argument of the constructor, by default `planning_scene::PlanningScenePtr()`
    - `std::shared_ptr<tf2_ros::Buffer> tf_buffer_;` (this type is [defined here](https://github.com/ros/geometry2/blob/ad04943f23608ab757389ce57d04f110df1c692b/tf2_ros/include/tf2_ros/buffer.h#L51) and [implemented here](https://github.com/ros/geometry2/blob/ad04943f23608ab757389ce57d04f110df1c692b/tf2_ros/src/buffer.cpp)) Argument of the constructor, stores known frames.    tf2_ros::TransformListener
    - `std::unique_ptr<occupancy_map_monitor::OccupancyMapMonitor> octomap_monitor_;`
    - `robot_model_loader::RobotModelLoaderPtr rm_loader_;`, loads the model
    - `moveit::core::RobotModelConstPtr robot_model_;`
    - `collision_detection::CollisionPluginLoader collision_loader_;`
    - `DynamicReconfigureImpl* reconfigure_impl_;`

## Planning Scene Manager

The planning scene `planning_scene::PlanningScene` is the central class for motion planning in MoveIt.
It is [defined here](https://github.com/ros-planning/moveit/blob/382aa5a8cdd39eace07536d39c497a4b21f0f653/moveit_core/planning_scene/include/moveit/planning_scene/planning_scene.h#L87) and [implemented here](https://github.com/ros-planning/moveit/blob/master/moveit_core/planning_scene/src/planning_scene.cpp).
A planning scene represents all the information needed to compute motion plans: 
    - The robot's current state
    - its representation (geometric, kinematic, dynamic)
    - the world representation.
Using this information, things like forward kinematics, inverse kinematics, evaluation of constraints, collision checking, are all possible.

The `planning_scene::PlanningScene` class is tightly connected to the `planning_scene_monitor::PlannningSceneMonitor` class, which maintains a planning scene using information from the ROS Parameter Server and subscription to topics.

The `PlanningScene` class provides the main interface that you will use for collision checking and constraint checking. 

This class maintains the representation of the environment as seen by a planning instance. The environment geometry, the robot geometry and state are maintained. 

The `PlanningScene` class can be easily setup and configured using a RobotModel or a URDF and SRDF.
This is, however, not the recommended way to instantiate a `PlanningScene`.
The `PlanningSceneMonitor` is the recommended method to create and maintain the current planning scene using data from the robot’s joints and the sensors on the robot.
In this tutorial, we will instantiate a `PlanningScene` class directly, but this method of instantiation is only intended for illustration.

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

## Planning Pipeline

In MoveIt, the motion planners are setup to plan paths.
The class `planning_pipeline::PlanningPipelie` is [defined here](https://github.com/ros-planning/moveit/blob/master/moveit_ros/planning/planning_pipeline/include/moveit/planning_pipeline/planning_pipeline.h) and [implemented here](https://github.com/ros-planning/moveit/blob/master/moveit_ros/planning/planning_pipeline/src/planning_pipeline.cpp).
However, there are often times when we may want to pre-process the motion planning request or post-process the planned path (e.g. for time parameterization).
In such cases, we use the planning pipeline which chains a motion planner with pre-processing and post-processing stages.
The pre and post-processing stages, called planning request adapters, can be configured by name from the ROS parameter server.

This class facilitates loading planning plugins and planning request adapted plugin.
It also allows calling `planning_interface::PlanningContext::solve()` from a loaded planning plugin and the `planning_request_adapter::PlanningRequestAdapter` plugins, in the specified order.

- **Requirements to instantiate a Planning pipeline**
    - A robot model (`moveit::core::RobotModel`) for which this pipeline is initialized.
    - (default `ros::NodeHandle("~")`) ROS node handle that should be used for reading parameters needed for configuration
    - (default in ROS parameter `"planning_plugin"` which by default is `ompl_interface/OMPLPlanner`) The name of the ROS parameter under which the name of the planning plugin is specified
    - (default in ROS parameter `"request_adapters"`) The name of the ROS parameter under which the names of the request adapter plugins are specified (plugin names separated by space; order matters) or array of plugin names. This is stored in `PlanningPipeline::adapter_plugin_names_`. By default the `"request_adapters"` ROS paramter has
    ```
    default_planner_request_adapters/AddTimeParameterization            default_planner_request_adapters/FixWorkspaceBounds            default_planner_request_adapters/FixStartStateBounds            default_planner_request_adapters/FixStartStateCollision            default_planner_request_adapters/FixStartStatePathConstraints
    ```

- **Published topics**
    - `display_planned_path` of type `moveit_msgs::DisplayTrajectory`
    - `motion_plan_request` of type `moveit_msgs::MotionPlanRequest`
    - `display_contacts` `visualization_msgs::MarkerArray`

- **Variables**
    - `std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader_;` is used to instantiate `planner_instance_`
    - `planning_interface::PlannerManagerPtr planner_instance_;` is used to get a `PlanningContext` and solve the planning problem
    - `std::unique_ptr<pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter> > adapter_plugin_loader_;` is used to instantiate `adapter_chain_`
    - `std::unique_ptr<planning_request_adapter::PlanningRequestAdapterChain> adapter_chain_;`
    - `moveit::core::RobotModelConstPtr robot_model_;`
    

- **Constructor**
    ```
      // load parameters ...
      // ...
      planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));
      planner_instance_ = planner_plugin_loader_->createUniqueInstance(planner_plugin_name_);
      planner_instance_->initialize(robot_model_, nh_.getNamespace())

      // load the planner request adapters
      if (!adapter_plugin_names_.empty())
      {
        std::vector<planning_request_adapter::PlanningRequestAdapterConstPtr> ads;
        adapter_plugin_loader_.reset(new pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter>(
              "moveit_core", "planning_request_adapter::PlanningRequestAdapter"));

        for (const std::string& adapter_plugin_name : adapter_plugin_names_)
        {
          planning_request_adapter::PlanningRequestAdapterPtr ad;
          ad = adapter_plugin_loader_->createUniqueInstance(adapter_plugin_name);
          ad->initialize(nh_);
          ads.push_back(std::move(ad));
        }
        adapter_chain_.reset(new planning_request_adapter::PlanningRequestAdapterChain());
        for (planning_request_adapter::PlanningRequestAdapterConstPtr& ad : ads)
          adapter_chain_->addAdapter(ad);
      }
      displayComputedMotionPlans(true);
      checkSolutionPaths(true);
    ```

- **Generate plan** 
    ```C++
    // input 
    //   - const planning_scene::PlanningSceneConstPtr& planning_scene,
    //   - const planning_interface::MotionPlanRequest& req,
    // output
    //   - planning_interface::MotionPlanResponse& res,
    bool planning_pipeline::PlanningPipeline::generatePlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                           const planning_interface::MotionPlanRequest& req,
                                                           planning_interface::MotionPlanResponse& res,
                                                           std::vector<std::size_t>& adapter_added_state_index) const
    {
      if (publish_received_requests_) received_request_publisher_.publish(req);

      bool solved = false;
       if (adapter_chain_)
       {
         solved = adapter_chain_->adaptAndPlan(planner_instance_, planning_scene, req, res, adapter_added_state_index);
         if (!adapter_added_state_index.empty())
         {
           std::stringstream ss;
           for (std::size_t added_index : adapter_added_state_index)
             ss << added_index << " ";
           ROS_INFO("Planning adapters have added states at index positions: [ %s]", ss.str().c_str());
         }
       }
       else
       {
         planning_interface::PlanningContextPtr context =
             planner_instance_->getPlanningContext(planning_scene, req, res.error_code_);
         solved = context ? context->solve(res) : false;
       }

      bool valid = true;

      if (solved && res.trajectory_)
      {
        std::size_t state_count = res.trajectory_->getWayPointCount();
        ROS_DEBUG_STREAM("Motion planner reported a solution path with " << state_count << " states");
        if (check_solution_paths_)
        {
          std::vector<std::size_t> index;
          if (!planning_scene->isPathValid(*res.trajectory_, req.path_constraints, req.group_name, false, &index))
          {
            // check to see if there is any problem with the states that are found to be invalid
            // they are considered ok if they were added by a planning request adapter
          }
          else
            ROS_DEBUG("Planned path was found to be valid when rechecked");
        }
      }

      if (display_computed_motion_plans_ && solved)
      {
      // display solution path to `display_planned_path`
      }

      if (!solved)
      {
        // This should alert the user if planning failed because of contradicting constraints.
        // Could be checked more thoroughly, but it is probably not worth going to that length.
      }

      return solved && valid;
    }
    ```


## Planning context and Planner manager
A `PlanningContext` is an abstract class that encapsulates a planning scene and a motion planning request.
It is [defined here](https://github.com/ros-planning/moveit/blob/382aa5a8cdd39eace07536d39c497a4b21f0f653/moveit_core/planning_interface/include/moveit/planning_interface/planning_interface.h#L80) and [implemened here](https://github.com/ros-planning/moveit/blob/master/moveit_core/planning_interface/src/planning_interface.cpp).

The planner manager `PlannerManager` is [defined here](https://github.com/ros-planning/moveit/blob/ba4b60e079fd14a61c50ef34c156eee6d63e58f7/moveit_core/planning_interface/include/moveit/planning_interface/planning_interface.h#L150) and [implemented here](https://github.com/ros-planning/moveit/blob/master/moveit_core/planning_interface/src/planning_interface.cpp#L94)

- **To constcut a PlanningContext instance**
    - 
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
