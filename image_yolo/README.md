# YOLOv8 Package

This packages uses the model that is trained on the pins to find the pins and their coordinates with respect to the base of the arm. 

## Quickstart
1. Use `ros2 run image_yolo yolo` to start the yolo node
    - This node has a service call that detect pins that is called while the gun is scanning for targets. 
    - It is unable to detect without the realsense camera node running.