#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
import cv2
from std_srvs.srv import Empty
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
import sys
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import os
import numpy as np
from geometry_msgs.msg import PointStamped
import pyrealsense2 as rs2

if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2
import os


bridge = CvBridge()

class Camera_subscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        # self.sub = self.create_subscription(msg_Image, '/camera/color/image_raw', self.camera_callback, 1)
        self._depth_image_topic = 'camera/aligned_depth_to_color/image_raw'#'/camera/depth/image_rect_raw'
        self._depth_info_topic = '/camera/depth/camera_info'
        self.sub_depth = self.create_subscription(msg_Image, self._depth_image_topic, self.imageDepthCallback, 1)
        self.sub_info = self.create_subscription(CameraInfo, self._depth_info_topic, self.imageDepthInfoCallback, 1)

        self.sub1 = self.create_subscription(msg_Image, '/camera/color/image_raw', self.get_latest_frame, 1)
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None
        path = os.path.dirname(__file__)
        self.model = YOLO('/home/max/courses/495_Embedded_Systems/ws/final_project/src/final-project-group2/image_yolo/image_yolo/best.pt')
        self.centroid=self.create_service(Empty,'coordinates',self.detect_pins)
        self._latest_depth_img = None
        self._latest_color_img=None
        self.img_pub = self.create_publisher(msg_Image, "/inference_result", 1)
        self.point_r = PointStamped()
        self.point_y = PointStamped()
        self.point_g = PointStamped()
        self.point_b = PointStamped()
        self.pub = self.create_publisher(msg_Image, "pixel_img", 10)
        self.marker_array = MarkerArray() 
        self.red_count=0
        self.blue_count=0
        self.green_count=0
        self.yellow_count=0

        markerQoS = QoSProfile(
            depth=10, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.pub2 = self.create_publisher(
            MarkerArray, "visualization_marker_array", markerQoS)


    def detect_pins(self, request, response):
        red_pins = []  # List to store centroids of red pins
        yellow_pins = [] 
        green_pins =[]
        blue_pins=[] 
        
            
        results = self.model(self._latest_color_img)  # Use the YOLO model to get detection results

        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # Get box coordinates in (top, left, bottom, right) format
                centroid = ((b[1] + b[3]) // 2, (b[0] + b[2]) // 2)  # Calculate centroid from box coordinates
                class_name = self.model.names[int(box.cls)]  # Get class name
                if class_name != "not_pins":  # Exclude class "not_pins"
                    if class_name == "red_pins":
                        x, y, z = self.depth_world(centroid[0], centroid[1])  # Get x, y, z from depth_world function
                        self.get_logger().info(f"{x},{y},{z}")
                        red_pins.append((x, y, z)) 
                        for i in red_pins:
                            self.point_r.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp
                            self.point_r.header.frame_id = f"red_pins_{self.red_count}"
                            self.point_r.point.x = i[2]#i[0]  # Set x, y, z coordinates
                            self.point_r.point.y = -i[0]#i[1]
                            self.point_r.point.z = -i[1]#i[2]  
                            self.create_marker(self.point_r.point.x,self.point_r.point.y,self.point_r.point.z,self.red_count,'red')
                            self.red_count += 1

                    elif class_name == "yellow_pins":
                        x1, y1, z1 = self.depth_world(centroid[0], centroid[1])  # Get x, y, z from depth_world function
                        yellow_pins.append((x1, y1, z1))
                        for j in yellow_pins:
                            self.point_y.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp
                            self.point_y.header.frame_id = f"yellow_pins_{self.yellow_count}" 
                            self.point_y.point.x = j[2]#j[0]  # Set x, y, z coordinates
                            self.point_y.point.y = -j[0]#j[1]
                            self.point_y.point.z = -j[1]#j[2]
                            self.create_marker(self.point_y.point.x,self.point_y.point.y,self.point_y.point.z,self.yellow_count,'yellow')
                            self.yellow_count += 1

                    elif class_name == "green_pins":
                        x2, y2, z2 = self.depth_world(centroid[0], centroid[1])  # Get x, y, z from depth_world function
                        green_pins.append((x2, y2, z2))
                        for k in green_pins:
                            self.point_g.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp
                            self.point_g.header.frame_id = f"green_pins_{self.green_count}"
                            self.point_g.point.x = k[2]#k[0]  # Set x, y, z coordinates
                            self.point_g.point.y = -k[0]#k[1]
                            self.point_g.point.z = -k[1]#k[2] 
                            self.create_marker(self.point_g.point.x,self.point_g.point.y,self.point_g.point.z,self.green_count,'green')
                            self.green_count += 1

                    elif class_name == "blue_pins":
                        x3, y3, z3 = self.depth_world(centroid[0], centroid[1])  # Get x, y, z from depth_world function
                        blue_pins.append((x3, y3, z3))
                        for l in blue_pins:
                            self.point_b.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp
                            self.point_b.header.frame_id = f"blue_pins_{self.blue_count}" 
                            self.point_b.point.x = l[2]#l[0]  # Set x, y, z coordinates
                            self.point_b.point.y = -l[0]#l[1]
                            self.point_b.point.z = -l[1]#l[2] 
                            self.create_marker(self.point_b.point.x,self.point_b.point.y,self.point_b.point.z,self.blue_count,'blue')
                            self.blue_count += 1
                        
            self.pub2.publish(self.marker_array)
            self.marker_array= MarkerArray()
        return response
    
    def depth_world(self, x, y):
        if self.intrinsics and self._latest_depth_img is not None and self._latest_color_img is not None:
            self.get_logger().info("processing request")
            
            depth_x = int(x)
            depth_y = int(y)
            depth = self._latest_depth_img[depth_x, depth_y]

            result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [y, x], depth)
            x_new, y_new, z_new = result[0], result[1], result[2]
            
            return x_new, y_new, z_new


    def create_marker(self, x, y, z, count,ns):
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = count
        marker.ns = ns
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.pose.position.x = x / 1000
        marker.pose.position.y = y / 1000
        marker.pose.position.z = z / 1000
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.0
        if ns=='red':
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        elif ns=='green':
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        elif ns=='blue':
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
        elif ns=='yellow':
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        self.marker_array.markers.append(marker)
          # Append the marker to the MarkerArray 

    def get_latest_frame(self,data):
        # Wait for a new frame from the RealSense camera
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self._latest_color_img = cv_image
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return
             
     
    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.d]
        except CvBridgeError as e:
            print(e)
            return
    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self._latest_depth_img = cv_image
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return


def main(args=None):
    rclpy.init(args=args)
    node = Camera_subscriber()
    rclpy.spin(node)
    rclpy.shutdown()