import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

class ControlNode(Node):

    def __init__(self):
        super().__init__("control")

        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        # Callback group
        self._cbgrp = ReentrantCallbackGroup()

        # clients and publishers
        self._input_client = self.create_client(Empty, 'input', callback_group = self._cbgrp)
        self._vision_client = self.create_client(Empty, 'inference', callback_group = self._cbgrp)
        self._marker_pub = self.create_publisher(MarkerArray, "visualization_marker", markerQoS)
        
        # wait for services to become available
        while not self._input_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        while not self._vision_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # main loop timer
        self._loop_timer = self.create_timer(0.01, self.loop_cb)
        
        # variables
        self._markers = None    # store the MarkerArray

    async def loop_cb(self):
        """ Main loop. """
        # RUN ONCE!
        # scan targets
        # scan guns
        # wait for user input
        # shoot
        return

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