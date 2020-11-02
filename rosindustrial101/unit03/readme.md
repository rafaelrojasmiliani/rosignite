# Build a MoveIt Package

In this unit we learn to build a moveIt package with the movit assistant wizard.

Tasks

1. Creationg of ros packages which contains the URDF description of the robot's workspace `src/my_robot_description`

2. Creation of a **MoveIt configuration package** `src/myrobot_moveit_config`. 

3. Integration of the MoveIt package into Gazebo. 

## Definitions

- **MoveIt configuration package**

- **MoveIt move group** It provides an interface for most operations that a user may want to carry out, specifically setting joint or pose goals, creating motion plans, moving the robot, adding objects into the environment and attaching/detaching objects from the robot. This interface communicates over ROS topics, services, and actions to the **MoveGroup Node**.



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

7. In the **Configuration Files** tab. Here you will create the moveIt configuration package
    1. Choose a location and name for the ROS package that will be generated containing your new set of configuration files. 
        - Click browse, select a good location
        - click Create New Folder and click Choose. 
All generated files will go directly into the directory you have chosen.
    2. Click on the Generate Package button. The Setup Assistant will now generate and write a set of launch and config files into the directory of your choosing. 

## Combining MoveIt with Gazebo using RosControl

- Inside the config folder of your moveit package, create a new file named controllers.yaml. Copy the following content inside it: 
```
controller_list:
  - name: sia10f/joint_trajectory_controller
    action_ns: "follow_joint_trajectory"
    type: FollowJointTrajectory
    joints: [joint_s, joint_l, joint_e, joint_u, joint_r, joint_b, joint_t]
```
