# ME495 Embedded Systems Final Project
Authors: Joel Goh, Maximiliano Palay, Rahul Roy, Sophia Schiffer, Jialu Yu




https://github.com/ME495-EmbeddedSystems/final-project-group2/assets/41023326/e53cf0b2-0609-47e5-a805-149bd48352c2






In this project, the Franka arm searches for known randomly placed pins and shoots them down. It is fitted with an onboard camera to see its environment and a Nerf blaster to be able to shoot the targets. The user can specify the color of the pins that the arm is going to shoot first and then once all of those colored pins are shot, the arm will wait for a second input to continue shooting different colored targets. 

## Quickstart
0. Connect the arduino and realsense camera to the computer
    - Optional to connect an external microphone to your computer but the built-in computer microphone works as well
1. Ensure that the two nerf guns are in front of the Franka arm roughly symmetrical around the y axis
    - The two guns need the apriltag 42 and 95 from the 36h11 family
    - Each gun is also loaded with 6 bullets
1. Start the moveit group on the Franka station using `ros2 launch franka_moveit_config moveit.launch.py use_rviz:=false robot_ip:=panda0.robot`
2. Connect to the realsense by running on your computer `ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true align_depth.enable:=true`
3. On your computer run the launch file, `ros2 launch control shoot_pins.launch.xml`, which starts the rest of the necessary nodes
    - An error message will show with the msg, "Waiting for input", where the audio input can be given starting the demo

## Overall System 
The image below shows the different nodes in the system and how they communicate with each other.
![image](https://github.com/ME495-EmbeddedSystems/final-project-group2/assets/61445107/208b8e67-7a8c-4eb3-9676-8359ce41ecad)

## Nodes
Created by the authors:
- Control
    - Main node which is used to call services to other nodes to carry out the demo.
- Shoot
    - Node that carries out all moveit-interface services such as cartesian planning, IK planning, and gripper requests. 
- Yolo
    - Node that runs YOLO to find the pin with respect to the base of the Franka arm. 
- User_Input
    - Node that listens to the user's audio input to set the color of the targeted pins. 
- Trigger
    - Node that controls the Arduino for the gun's trigger.

Not created by the authors:
- Apriltag_node
    - Node that scans and gives the coordinates for the April tags.

## Launch Files
- Shoot_pins.launch.xml in control package
    - This launch file launches the franka_moveit_config rviz launch file and the control, shoot, yolo, user_input, trigger, and apriltag nodes.
 
## Future Work
- 
