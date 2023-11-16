# Adapted from https://github.com/IntelRealSense/realsense-ros/blob/ros2-development/realsense2_camera/scripts/show_center_depth.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import pyrealsense2 as rs2
from pixel_to_world_interfaces.srv import Pixel

if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

class DepthToWorld(Node):
    def __init__(self):
        
        super().__init__('depth_to_world')
        
        self._depth_image_topic = '/camera/depth/image_rect_raw'
        self._depth_info_topic = '/camera/depth/camera_info'
        self._color_image_topic = '/camera/color/image_raw'

        self.bridge = CvBridge()
        
        self._sub_color = self.create_subscription(msg_Image, self._color_image_topic, self.imageColorCallback, 1)
        self.sub_depth = self.create_subscription(msg_Image, self._depth_image_topic, self.imageDepthCallback, 1)
        self.sub_info = self.create_subscription(CameraInfo, self._depth_info_topic, self.imageDepthInfoCallback, 1)

        self.pub = self.create_publisher(msg_Image, "pixel_img", 10)

        self.serv = self.create_service(Pixel, "pixel_to_world", self.getPixelCoords)

        self.intrinsics = None
        self._latest_depth_img = None
        self._latest_color_img = None


    def getPixelCoords(self, request, response):
        if self.intrinsics and self._latest_depth_img is not None and self._latest_color_img is not None:
            self.get_logger().info("processing request")
            # this is where the processing takes place
            depth = self._latest_depth_img[request.x, request.y]
            result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [request.x, request.y], depth)
            response.x = result[0]
            response.y = result[1]
            response.z = result[2]
            # this is just a visualization thing
            img = cv2.circle(self._latest_color_img, (request.x,request.y), radius=5, color=(0,0,255), thickness=-1)
            img_msg = self.bridge.cv2_to_imgmsg(img)
            self.pub.publish(img_msg)

        return response
        

    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self._latest_depth_img = cv_image
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return
        
    def imageColorCallback(self, data):
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

def main():
    rclpy.init()
    node = DepthToWorld()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()