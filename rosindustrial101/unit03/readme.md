# Build a MoveIt Package

In this unit we learn to build a moveIt package with the movit assistant wizard.

Tasks

1. Creationg of ros packages which contains the URDF description of the robot's workspace `src/my_robot_description`

2. Creation of a **MoveIt configuration package** `src/myrobot_moveit_config`. 

3. Integration of the MoveIt package into Gazebo. 



## Basic use of the moveit assistan wizard

## Combining MoveIt with Gazebo using RosControl

- Inside the config folder of your moveit package, create a new file named controllers.yaml. Copy the following content inside it: 
```
controller_list:
  - name: sia10f/joint_trajectory_controller
    action_ns: "follow_joint_trajectory"
    type: FollowJointTrajectory
    joints: [joint_s, joint_l, joint_e, joint_u, joint_r, joint_b, joint_t]
```
