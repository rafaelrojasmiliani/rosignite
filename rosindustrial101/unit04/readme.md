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
