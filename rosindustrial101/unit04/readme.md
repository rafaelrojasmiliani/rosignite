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
