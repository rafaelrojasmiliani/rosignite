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
    - `moveit_controller_manager`
    - `controller_list`

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

`TrajectoryExecutionManager` uses an instance of `moveit_controller_manager::MoveItControllerManager` which is inizialized as
```C++
    controller_manager_ = controller_manager_loader_->createUniqueInstance(controller);
```
where `controller` is a string available in the ROS paramenter `
moveit_controller_manager`.



## Procedure
1. We create a new package with
```
catkin_create_pkg my_motion_scripts std_msgs geometry_msgs rospy
```

2. Write the following python code in `myrobot_python/src/python_moveit.py`
```
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
