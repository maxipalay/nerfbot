import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Point, Pose, Quaternion
from std_srvs.srv import Empty

from trajectory_interfaces.srv import Target, TargetScanRequest

from .moveit_api import MoveItAPI
from .quaternion import *


class MoveGun(Node):
    def __init__(self):
        super().__init__("move_gun")

        # Move it API
        self.planning_group = "panda_manipulator"
        self.gripper_action = "panda_gripper/grasp"
        self.pipeline_id = "move_group"
        self.constraints_link = "panda_hand_tcp"
        self.executed = False
        self._cbgrp = ReentrantCallbackGroup()

        self.moveit_api = MoveItAPI(
            self,
            self.planning_group,
            self.gripper_action,
            self.pipeline_id,
            self.constraints_link,
        )

        ### variables
        self.scan_x = 0.30689
        self.scan_y = 0.5
        self.scan_z = 0.48688

        ### callback_groups

        ### parameters

        ### services
        self.shot = self.create_service(Target, "/shoot", self.shoot_SrvCallback)
        self.gun_scan = self.create_service(Empty, "/gun_scan", self.gun_scan_callback)
        self.target_scan = self.create_service(
            TargetScanRequest, "/target_scan", self.target_scan_callback
        )

        ### timer

        ### clients

        ### subscribers

        ### publisher

        # pin scanning stuff
        # base EE orientation for the pin scanning pose, x=-180deg, y=60deg
        base_orientation = Quaternion(x=-0.8660254, y=0.0, z=-0.5, w=0.0)
        # rotation around x axis 45 degrees
        rot_1 = Quaternion(x=0.3826834, y=0.0, z=0.0, w=0.9238795)
        # rotation around x axis -45 degrees
        rot_2 = Quaternion(x=-0.3826834, y=0.0, z=0.0, w=0.9238795)

        self._scan_positions = [
            Pose(
                position=Point(x=0.5, y=0.0, z=0.6),
                orientation=quaternion_multiply(base_orientation, rot_1),
            ),
            Pose(
                position=Point(x=0.5, y=0.0, z=0.6),
                orientation=quaternion_multiply(base_orientation, rot_2),
            ),
        ]

        self._scanning_targets = False
        self._target_scan_index = 0

    async def grip(self):
        await self.moveit_api.move_gripper(0.025, 0.05, 10.0)

    async def shoot_SrvCallback(self, request, response):
        self.get_logger().info("Found target")
        target_x = request.target.x
        target_y = request.target.y
        target_z = request.target.z
        target_p, target_o = self.find_pose(target_x, target_y, target_z)

        target = Pose(position=target_p, orientation=target_o)

        await self.moveit_api.plan_and_execute(
            self.moveit_api.plan_position_and_orientation, target
        )
        self.get_logger().info("Aimed at target")
        return response

    async def target_scan_callback(self, request, response):
        # await self.grip()
        # return response
        # if we're not running a scan already
        if not self._scanning_targets:
            # set the scanning in progress flag
            self._scanning_targets = True
        # if we're running a scan and there are more points
        if self._scanning_targets and self._target_scan_index < len(
            self._scan_positions
        ):
            # move to the next point
            self.get_logger().info(str(self._scan_positions[self._target_scan_index]))
            await self.moveit_api.plan_and_execute(
                self.moveit_api.plan_position_and_orientation,
                self._scan_positions[self._target_scan_index],
            )
            # increase the index by one
            self._target_scan_index += 1
            # if the index has gone above all our points
            if self._target_scan_index >= len(self._scan_positions):
                response.more_scans = False
                self._scanning_targets = False
                self._target_scan_index = 0
            else:
                response.more_scans = True

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
            self.moveit_api.plan_position_and_orientation, target
        )

        # move arm to ending location
        target.position.x = self.scan_x
        target.position.y = self.scan_y
        target.position.z = self.scan_z

        await self.moveit_api.plan_and_execute(
            self.moveit_api.plan_position_and_orientation, target
        )
        self.get_logger().info("Scan complete")
        return response

    def find_pose(self, x, y, z):
        self.get_logger().info(f"{x}, {y}")
        yaw = np.arctan2(y, x)
        pitch = np.arctan2(z - 0.5, x)
        self.get_logger().info(f"{pitch}, {yaw}")

        target_p = Point()
        target_p.x = 0.6 * np.cos(yaw)
        target_p.y = 0.6 * np.sin(yaw)
        target_p.z = 0.5

        target_o = euler_to_quaternion(0.0, np.pi / 2 - pitch, yaw)
        self.get_logger().info(f"{target_p}")
        self.get_logger().info(f"{target_o}")

        return target_p, target_o


def entry_point(args=None):
    """The entry_point function."""
    rclpy.init(args=args)
    move_gun = MoveGun()

    rclpy.spin(move_gun)
