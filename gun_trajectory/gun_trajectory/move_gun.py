import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Point, Pose, Quaternion
from std_srvs.srv import Empty

from trajectory_interfaces.srv import Target

from .moveit_api import MoveItAPI
from .quaternion import *

class MoveGun(Node):

    def __init__(self):
        super().__init__('move_gun')

        # Move it API
        self.planning_group = "panda_manipulator"
        self.gripper_action = 'panda_gripper/gripper_action'
        self.pipeline_id = "move_group"
        self.constraints_link = "panda_hand_tcp"
        self.executed = False
        self._cbgrp = ReentrantCallbackGroup()

        self.moveit_api = MoveItAPI(
            self,
            self.planning_group,
            self.gripper_action,
            self.pipeline_id,
            self.constraints_link)

        ### variables
        self.scan_x = 0.30689
        self.scan_y = 0.5
        self.scan_z = 0.48688

        ### callback_groups

        ### parameters

        ### services
        self.shot = self.create_service(Target, '/shoot', self.shoot_SrvCallback)
        self.gun_scan = self.create_service(Empty, '/gun_scan', self.gun_scan_callback)

        ### timer

        ### clients

        ### subscribers

        ### publisher 

    async def shoot_SrvCallback(self, request, response):
        self.get_logger().info("Found target")
        target_x = request.target.x
        target_y = request.target.y
        target_z = request.target.z
        target_p, target_o = self.find_pose(target_x, target_y, target_z)

        target = Pose(
            position=target_p,
            orientation=target_o)


        await self.moveit_api.plan_and_execute(
                self.moveit_api.plan_position_and_orientation, target)
        self.get_logger().info('Aimed at target')
        return response
    
    async def gun_scan_callback(self, request, response):
        self.get_logger().info("Starting scan")

        # move arm to starting location
        target = Pose()
        target.position.x = self.scan_x
        target.position.y = -self.scan_y
        target.position.z = self.scan_z

        target.orientation.x = 1.0
        target.orientation.y = 0.0
        target.orientation.z = 0.0
        target.orientation.w = 0.0

        await self.moveit_api.plan_and_execute(
                self.moveit_api.plan_position_and_orientation,target)
        
        # move arm to ending location
        target.position.x = self.scan_x
        target.position.y = self.scan_y
        target.position.z = self.scan_z

        await self.moveit_api.plan_and_execute(
                self.moveit_api.plan_position_and_orientation,target)
        self.get_logger().info("Scan complete")
        return response

    def find_pose(self, x, y, z):
        self.get_logger().info(f'{x}, {y}')
        yaw = np.arctan2(y, x)
        pitch = np.arctan2(z - 0.5, x)
        self.get_logger().info(f'{pitch}, {yaw}')

        target_p = Point()
        target_p.x = 0.6 * np.cos(yaw)
        target_p.y = 0.6 * np.sin(yaw)
        target_p.z = 0.5

        target_o = euler_to_quaternion(0.0, np.pi/2 - pitch, yaw)
        self.get_logger().info(f"{target_p}")
        self.get_logger().info(f"{target_o}")

        return target_p, target_o

def entry_point(args=None):
    """ The entry_point function. """
    rclpy.init(args=args)
    move_gun = MoveGun()

    try:
        rclpy.spin(move_gun)
    except:
        pass