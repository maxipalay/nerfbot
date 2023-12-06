# ME495 Embedded Systems Final Project
Authors: Joel Goh, Maximiliano Palay, Rahul Roy, Sophia Schiffer, Jialu Yu

Brief project overview

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

## Nodes
Created by the authors
- Control
    - Node that calls services from other nodes to run the demo
- Shoot
    - Node that controls the goal poses of the arm and the gripper
- Yolo
    - Node that runs YOLO and the target's real world coordinates
- User_Input
    - Node that listens to user's audio input
- Trigger
    - Node that controls the arduino for the gun's trigger

Not created by the authors
- Apriltag_node
    - Node that scans and gives the coordinates for the apriltags

## Launch Files

## Future Work