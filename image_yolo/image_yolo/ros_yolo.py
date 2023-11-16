#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
import cv2
from std_srvs.srv import Empty
from cv_bridge import CvBridge, CvBridgeError
import sys
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
                    print(self.point_msg)
                    if self.intrinsics and self._latest_depth_img is not None and self._latest_color_img is not None:
                        self.get_logger().info("processing request")
                        # this is where the processing takes place
                        depth_x = int(self.point_msg.point.x)
                        depth_y = int(self.point_msg.point.y)
                        depth = self._latest_depth_img[depth_x, depth_y]

                        result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [self.point_msg.point.x, self.point_msg.point.y], depth)
                        self.point_msg.point.x = result[0]
                        self.point_msg.point.y = result[1]
                        self.point_msg.point.z = result[2]
                        print(self.point_msg)
                        # this is just a visualization thing
                        img = cv2.circle(self._latest_color_img, (self.point_msg.point.x,self.point_msg.point.y), radius=5, color=(0,0,255), thickness=-1)
                        img_msg = self.bridge.cv2_to_imgmsg(img)
                        self.pub.publish(img_msg)
                
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
        

    # def getPixelCoords(self):
    #     if self.intrinsics and self._latest_depth_img is not None and self._latest_color_img is not None:
    #         self.get_logger().info("processing request")
    #         # this is where the processing takes place
    #         depth = self._latest_depth_img[self.point_msg.point.x,self.point_msg.point.y]
    #         result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [self.point_msg.point.x, self.point_msg.point.y], depth)
    #         self.point_msg.point.x = result[0]
    #         self.point_msg.point.y = result[1]
    #         self.point_msg.point.z = result[2]
    #         print(self.point_msg)
    #         # this is just a visualization thing
    #         img = cv2.circle(self._latest_color_img, (self.point_msg.point.x,self.point_msg.point.y), radius=5, color=(0,0,255), thickness=-1)
    #         img_msg = self.bridge.cv2_to_imgmsg(img)
    #         self.pub.publish(img_msg)

    #     return       
     
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

    # def camera_callback(self, data):

    #     img = bridge.imgmsg_to_cv2(data, "bgr8")
    #     results = self.model(img)

    #     self.yolov8_inference.header.frame_id = "inference"
    #     self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

    #     for r in results:
    #         boxes = r.boxes
    #         for box in boxes:
    #             self.inference_result = InferenceResult()
    #             b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
    #             c = box.cls
    #             self.inference_result.class_name = self.model.names[int(c)]
    #             if self.inference_result.class_name == "no_pins":
    #                 continue
    #             self.inference_result.top = int(b[0])
    #             self.inference_result.left = int(b[1])
    #             self.inference_result.bottom = int(b[2])
    #             self.inference_result.right = int(b[3])
    #             self.yolov8_inference.yolov8_inference.append(self.inference_result)

    #         #camera_subscriber.get_logger().info(f"{self.yolov8_inference}")

    #     annotated_frame = results[0].plot()
    #     img_with_overlay = cv2.addWeighted(img, 0.7, annotated_frame, 0.3, 0)
    #     img_msg = bridge.cv2_to_imgmsg(img_with_overlay)
    #     # img_msg = bridge.cv2_to_imgmsg(annotated_frame)  

    #     self.img_pub.publish(img_msg)
    #     self.yolov8_pub.publish(self.yolov8_inference)
    #     self.yolov8_inference.yolov8_inference.clear()

        
   

    # def imageDepthCallback(self, data):
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
    #         # pick one pixel among all the pixels with the closest range:
    #         indices = np.array(np.where(cv_image == cv_image[cv_image > 0].min()))[:,0]
    #         pix = (indices[1], indices[0])
    #         self.pix = pix
    #         line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])

    #         if self.intrinsics:
    #             depth = cv_image[pix[1], pix[0]]
    #             result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
    #             line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
    #         if (not self.pix_grade is None):
    #             line += ' Grade: %2d' % self.pix_grade
    #         line += '\r'
    #         sys.stdout.write(line)
    #         sys.stdout.flush()

    #     except CvBridgeError as e:
    #         print(e)
    #         return
    #     except ValueError as e:
    #         return


    # def imageDepthInfoCallback(self, cameraInfo):
    #     try:
    #         if self.intrinsics:
    #             return
    #         self.intrinsics = rs2.intrinsics()
    #         self.intrinsics.width = cameraInfo.width
    #         self.intrinsics.height = cameraInfo.height
    #         self.intrinsics.ppx = cameraInfo.k[2]
    #         self.intrinsics.ppy = cameraInfo.k[5]
    #         self.intrinsics.fx = cameraInfo.k[0]
    #         self.intrinsics.fy = cameraInfo.k[4]
    #         if cameraInfo.distortion_model == 'plumb_bob':
    #             self.intrinsics.model = rs2.distortion.brown_conrady
    #         elif cameraInfo.distortion_model == 'equidistant':
    #             self.intrinsics.model = rs2.distortion.kannala_brandt4
    #         self.intrinsics.coeffs = [i for i in cameraInfo.d]
    #     except CvBridgeError as e:
    #         print(e)
    #         return

def main(args=None):
    rclpy.init(args=args)
    node = Camera_subscriber()
    rclpy.spin(node)
    rclpy.shutdown()

