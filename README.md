# ME495 Embedded Systems Final Project
Author: Joel Goh, Maximiliano Palay, Rahul Roy, Sophia Schiffer, Jialu Yu

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

## Launch Files

## Future Work