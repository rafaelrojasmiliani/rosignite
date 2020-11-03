# Motion Planning with Python


This Unit will show you how to perform motion planning with Python.
By completing this Unit, you will be able to create a Python program that performs motion planning on your robot

Tasks

1. Use the packages created in the previous unit:
    - Robot workspace geometrical description with URDF `my_robot_description`
    - MoveIt configuration package `myrobot_moveit_config` with integration with gazebo.
This requires the launch file `myrobot_planning_execution.launch`

2. Implementation of **MoveIt**  in python


## Definitions

The **moveit\_commander** Python package offers wrappers for the functionality provided in MoveIt!.
The `moveit_commander` package also includes a command line interface, `moveit_commander_cmdline.py`.

We create a new pacakge witjh
```
catkin_create_pkg myrobot_python std_msgs geometry_msgs rospy
```
