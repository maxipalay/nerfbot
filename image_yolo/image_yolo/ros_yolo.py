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



from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class Camera_subscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        # self.sub = self.create_subscription(msg_Image, '/camera/color/image_raw', self.camera_callback, 1)
        self._depth_image_topic = '/camera/depth/image_rect_raw'
        self._depth_info_topic = '/camera/depth/camera_info'
        self.sub_depth = self.create_subscription(msg_Image, self._depth_image_topic, self.imageDepthCallback, 1)
        self.sub_info = self.create_subscription(CameraInfo, self._depth_info_topic, self.imageDepthInfoCallback, 1)

        self.sub1 = self.create_subscription(msg_Image, '/camera/color/image_raw', self.get_latest_frame, 1)
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None
        path = os.path.dirname(__file__)
        self.model = YOLO('/home/rahulroy/final_project/src/final/image_yolo/image_yolo/best.pt')
        self.centroid=self.create_service(Empty,'cordinates',self.detect_pins)
        self.yolov8_inference = Yolov8Inference()
        self._latest_depth_img = None
        self._latest_color_img=None
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(msg_Image, "/inference_result", 1)
        self.point_msg = PointStamped()
        self.pub = self.create_publisher(msg_Image, "pixel_img", 10)

        markerQoS = QoSProfile(
            depth=10, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.pub2 = self.create_publisher(
            MarkerArray, "visualization_marker_array", markerQoS)


    def detect_pins(self, request, response):
        pin_centroids = {}  # Dictionary to store centroids for each class
      
        results = self.model(self._latest_color_img)  # Use the YOLO model to get detection results

        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # Get box coordinates in (top, left, bottom, right) format
                centroid = ((b[1] + b[3]) // 2, (b[0] + b[2]) // 2)  # Calculate centroid from box coordinates
                class_name = self.model.names[int(box.cls)]  # Get class name
                if class_name != "not_pins":  # Exclude class "not_pins"
                    if class_name not in pin_centroids:
                        pin_centroids[class_name] = []  # Initialize list for the class if not present
                    pin_centroids[class_name].append(centroid)
                    
                    self.point_msg.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp
                    self.point_msg.header.frame_id = class_name 
                    self.point_msg.point.x = centroid[0]  # Set x, y, z coordinates
                    self.point_msg.point.y = centroid[1]
                    self.point_msg.point.z = 0  
                    
                    if self.intrinsics and self._latest_depth_img is not None and self._latest_color_img is not None:
                        self.get_logger().info("processing request")
                    
                        depth_x = int(self.point_msg.point.x)
                        depth_y = int(self.point_msg.point.y)
                        depth = self._latest_depth_img[depth_x, depth_y]

                        result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [self.point_msg.point.x, self.point_msg.point.y], depth)
                        self.point_msg.point.x = result[0]
                        self.point_msg.point.y = result[1]
                        self.point_msg.point.z = result[2]
                        self.markerarr = MarkerArray()
                        self.m = Marker()
                        self.m.header.frame_id = "camera_color_frame"
                        self.m.header.stamp = self.get_clock().now().to_msg()
                        self.m.id = 1
                        self.m.type = Marker.SPHERE
                        self.m.action = Marker.ADD
                        self.m.scale.x = 0.3
                        self.m.scale.y = 0.3
                        self.m.scale.z = 0.3
                        self.m.pose.position.x = self.point_msg.point.x/1000
                        self.m.pose.position.y = self.point_msg.point.y/1000
                        self.m.pose.position.z = self.point_msg.point.z/1000
                        self.m.pose.orientation.x = 0.0
                        self.m.pose.orientation.y = 0.0
                        self.m.pose.orientation.z = 0.0
                        self.m.pose.orientation.w = 0.0
                        self.m.color.r = 0.0
                        self.m.color.g = 0.0
                        self.m.color.b = 1.0
                        self.m.color.a = 1.0
                        self.markerarr.markers.append(self.m)
                        self.pub2.publish(self.markerarr)
        return response
    
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

