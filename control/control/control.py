import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class ControlNode(Node):

    def __init__(self):
        super().__init__("control")

        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        # Callback group
        self._cbgrp = ReentrantCallbackGroup()

        # clients and publishers
        self._input_client = self.create_client(Empty, 'input', callback_group = self._cbgrp)
        self._vision_client = self.create_client(Empty, 'inference', callback_group = self._cbgrp)
        self._gun_client = self.create_client(Empty, 'gun_scan', callback_group=self._cbgrp)
        self._marker_pub = self.create_publisher(MarkerArray, "visualization_marker", markerQoS)
        
        # wait for services to become available
        while not self._input_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        while not self._vision_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        while not self._gun_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # main loop timer
        self._loop_timer = self.create_timer(0.01, self.loop_cb)
        self._tf_timer = self.create_timer(0.01, self.tf_cb)
        
        # variables
        self._markers = None    # store the MarkerArray
        
        # TF listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    async def loop_cb(self):
        """ Main loop. """
        # RUN ONCE!
        # scan targets
        # scan guns
        

        # wait for user input
        # shoot
        return
    
    def tf_cd(self):
        """ Listens to tf data to track April Tags. """
        # TF listener
        try:
            # get the latest transform between left and right
            # (rclpy.time.Time() means get the latest information)
            tag_1 = self.buffer.lookup_transform("camera_link","tag36h11:1",rclpy.time.Time())
            self.t1_x = tag_1.transform.translation.x
            self.t1_y = tag_1.transform.translation.y
            self.t1_z = tag_1.transform.translation.z
            self.t1_ox = tag_1.transform.rotation.x
            self.t1_oy = tag_1.transform.rotation.y
            self.t1_ow = tag_1.transform.rotation.w
            self.t1_oz = tag_1.transform.rotation.z
        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().debug(f"Lookup exception: {e}")
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().debug(f"Connectivity exception: {e}")
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().debug(f"Extrapolation exception: {e}")

    def publish_markers(self):
        """ Pusblishes YOLO detections as Markers to visualize them in Rviz. """
        self._marker_pub.publish(self._markers)

        return
    

    def scan_targets(self, ):
        """ Moves the robot to scanning positions and requests for detections. """
        return


def main():
    rclpy.init()
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()