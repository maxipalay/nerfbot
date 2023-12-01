import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Point, Pose, Quaternion
from std_srvs.srv import Empty

from trajectory_interfaces.srv import Grab, Target, TargetScanRequest

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
        self.scan_forward = 0.3
        self.scan_up = 0.6

        self.tag_offset = np.array([0.02,0.0,0.11])  # position of end
                                                    # effector relative to tag
        self.ready_offset = 0.05

        ### callback_groups

        ### parameters

        ### services
        self.shot = self.create_service(Target, "/aim", self.shoot_SrvCallback)
        self.gun_scan = self.create_service(Empty, "/gun_scan", self.gun_scan_callback)
        self.target_scan = self.create_service(
            TargetScanRequest, "/target_scan", self.target_scan_callback
        )
        self.grab_gun = self.create_service(Grab, "/grab", self.grab_gun_callback)
        self.calibrate_gun = self.create_service(Empty, "/cali", self.grip_callback)

        self.move_cartesian_srv = self.create_service(Grab, "/move_cart", self.move_cart_callback) # shoudl be removed, used for testing

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

        # self._scan_positions = [
        #     Pose(
        #         position=Point(x=0.5, y=0.0, z=0.6),
        #         orientation=quaternion_multiply(base_orientation, rot_1),
        #     ),
        #     Pose(
        #         position=Point(x=0.5, y=0.0, z=0.6),
        #         orientation=quaternion_multiply(base_orientation, rot_2),
        #     ),
        # ]
        self._scan_positions = [
            Pose(
                position=Point(x=0.31, y=-0.172, z=0.6),
                orientation=Quaternion(x=0.806, y=-0.184, z=0.549, w=0.122),
            ),
            Pose(
                position=Point(x=0.31, y=0.172, z=0.6),
                orientation=Quaternion(x=0.7865, y=0.2054, z=0.562, w=-0.153),
            ),
        ]

        self._scanning_targets = False
        self._target_scan_index = 0

    async def move_cart_callback(self, request, response):
        await self.moveit_api.move_cartesian(request.pose)
        return response

    async def grip_callback(self, request, response):
        await self.moveit_api.move_gripper(0.025, 0.05, 10.0)
        return response

    async def shoot_SrvCallback(self, request, response):
        self.get_logger().info("Found target")
        target_x = request.target.x
        target_y = request.target.y
        target_z = request.target.z
        target_p, target_o = self.find_pose(target_x, target_y, target_z)

        target = Pose(position=target_p, orientation=target_o)

        # await self.moveit_api.plan_and_execute(
        #     self.moveit_api.plan_position_and_orientation, target
        # )
        await self.moveit_api.move_cartesian(target)
            
        self.get_logger().info("Aimed at target")
        return response
    
    async def grab_gun_callback(self, request, response):
       
        self.get_logger().info("Initiated grab")
        
        # move arm to starting location
        target = Pose()
        target.position.x = request.pose.position.x + self.tag_offset[0] + self.ready_offset
        target.position.y = request.pose.position.y + self.tag_offset[1]
        target.position.z = request.pose.position.z + self.tag_offset[2] + 0.25

        # target.orientation.x = request.pose.orientation.x
        # target.orientation.y = request.pose.orientation.y
        # target.orientation.z = request.pose.orientation.z
        # target.orientation.w = request.pose.orientation.w

        target.orientation.x = 1.0
        target.orientation.y = 0.0
        target.orientation.z = 0.0
        target.orientation.w = 0.0

        self.get_logger().info(f"Moving to standoff position.")
        
        await self.moveit_api.plan_and_execute(
            self.moveit_api.plan_position_and_orientation, target
        )
        # await self.moveit_api.move_cartesian(target)
            
        self.get_logger().info("Finished motion to standoff.")
        
        self.get_logger().info("Homing")       
        # await self.moveit_api.home_gripper()   

        target.position.x = request.pose.position.x + self.tag_offset[0]
        target.position.y = request.pose.position.y + self.tag_offset[1]
        target.position.z = request.pose.position.z + self.tag_offset[2]
        
        self.get_logger().info("Moving to pick position.")
        self.get_logger().info(f"Moving to z: {target.position.z}")    

        # success = False
        # count = 0 
        # while not success:
        #     success, plan, executed = await self.moveit_api.plan_and_execute(
        #         self.moveit_api.plan_position_and_orientation, target
        #     )
        #     count += 1
        #     if count == 10:
        #         self.get_logger().info(f"You are useless.")
        #         return response
        await self.moveit_api.move_cartesian(target)

        self.get_logger().info("Grasping")
        await self.moveit_api.move_gripper(0.025, 0.05, 10.0)

        # Move back to ready
        self.get_logger().info("Standoff")
        target.position.x = 0.4
        target.position.y = 0.0
        target.position.z = 0.6
        # await self.moveit_api.plan_and_execute(
        #     self.moveit_api.plan_position_and_orientation, target
        # )
        await self.moveit_api.move_cartesian(target)

        return response

    async def target_scan_callback(self, request, response):
        # if we're not running a scan already
        if not self._scanning_targets:
            # set the scanning in progress flag
            self._scanning_targets = True
        # if we're running a scan and there are more points
        if self._scanning_targets and self._target_scan_index < len(
            self._scan_positions
        ):
            # move to the next point
            self.get_logger().info(f"Next point: {str(self._scan_positions[self._target_scan_index])}")
            await self.moveit_api.plan_and_execute(
                self.moveit_api.plan_position_and_orientation,
                self._scan_positions[self._target_scan_index],
            )
            # await self.moveit_api.move_cartesian(self._scan_positions[self._target_scan_index])
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
        self.get_logger().info("Initiated gun scan")

        orientation = Quaternion(x=1.0,y=0.0,z=0.0,w=0.0)

        # self._scan_positions = [
        #     Pose(
        #         position=Point(x=self.scan_x, y=-self.scan_y, z=self.scan_z),
        #         orientation=orientation,
        #     ),
        #     Pose(
        #         position=Point(x=self.scan_forward, y=0.0, z=self.scan_up),
        #         orientation=orientation,
        #     ),
        #     Pose(
        #         position=Point(x=self.scan_x, y=self.scan_y, z=self.scan_z),
        #         orientation=orientation,
        #     ),
        # ]

        self._scan_positions = [
            Pose(
                position=Point(x=self.scan_forward, y=0.0, z=self.scan_up),
                orientation=orientation,
            ),
        ]

        self.get_logger().info("Starting gun scan")


        # # move arm to starting location
        # target = Pose()
        # target.position.x = self.scan_x
        # target.position.y = -self.scan_y
        # target.position.z = self.scan_z

        # target.orientation.x = 1.0
        # target.orientation.y = 0.0
        # target.orientation.z = 0.0
        # target.orientation.w = 0.0

        # await self.moveit_api.plan_and_execute(
        #     self.moveit_api.plan_position_and_orientation, self._scan_positions[0]
        # )
        await self.moveit_api.move_cartesian(self._scan_positions[0])

        # # move arm to ending location
        # target.position.x = self.scan_x
        # target.position.y = self.scan_y
        # target.position.z = self.scan_z

        # await self.moveit_api.plan_and_execute(
        #     self.moveit_api.plan_position_and_orientation, target
        # )
        self.get_logger().info("Scan complete")
        return response

    def find_pose(self, x, y, z):
        # y = y + np.sign(y) * 0.10
        self.get_logger().info(f"{x}, {y}")
        yaw = np.arctan2(y, x)
        pitch = np.arctan2(0.55 -z - 0.09, np.sqrt(x**2 + y**2) - 0.45)
        self.get_logger().info(f"{pitch}, {yaw}")

        target_p = Point()
        target_p.x = 0.5 * np.cos(yaw)
        target_p.y = 0.5 * np.sin(yaw)
        target_p.z = 0.55 

        target_o = euler_to_quaternion(np.pi, pitch, yaw)
        self.get_logger().info(f"{target_p}")
        self.get_logger().info(f"{target_o}")

        return target_p, target_o


def entry_point(args=None):
    """The entry_point function."""
    rclpy.init(args=args)
    move_gun = MoveGun()

    rclpy.spin(move_gun)
