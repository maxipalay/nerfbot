import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Point, Pose, Quaternion
from std_srvs.srv import Empty

from trajectory_interfaces.srv import Grab, Target, TargetScanRequest
from controller_manager_msgs.srv import SwitchController
from .moveit_api import MoveItAPI
from .quaternion import *
from franka_msgs.srv import SetLoad
from moveit_msgs.msg import RobotState, JointConstraint, Constraints
from sensor_msgs.msg import JointState

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

        self.tag_offset = np.array([0.05,0.0,-0.005])  # position of end
                                                    # effector relative to tag
        self.ready_offset = 0.0

        ### callback_groups

        ### parameters

        ### services
        self.shot = self.create_service(Target, "/aim", self.shoot_SrvCallback, callback_group=self._cbgrp)
        self.gun_scan = self.create_service(Empty, "/gun_scan", self.gun_scan_callback, callback_group=self._cbgrp)
        self.target_scan = self.create_service(
            TargetScanRequest, "/target_scan", self.target_scan_callback, callback_group=self._cbgrp
        )
        self.grab_gun = self.create_service(Grab, "/grab", self.grab_gun_callback, callback_group=self._cbgrp)
        self.place_gun = self.create_service(Grab, "/place", self.place_gun_callback)
        self.calibrate_gun = self.create_service(Empty, "/cali", self.grip_callback,callback_group=self._cbgrp)

        self.controller_client = self.create_client(SwitchController, "/controller_manager/switch_controller", callback_group=self._cbgrp)
        self.payload_client = self.create_client(SetLoad, "/service_server/set_load", callback_group=self._cbgrp)

        self.move_cartesian_srv = self.create_service(Grab, "/move_cart", self.move_cart_callback, callback_group=self._cbgrp) # shoudl be removed, used for testing

        while not self.controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("controller service not available, waiting again...")

        ### timer

        ### clients

        ### subscribers

        ### publisher

        # pin scanning stuff
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

        rs_1 = RobotState()
        js_1 = JointState()
        js_1.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        js_1.position = [-30*np.pi/180.0, -68*np.pi/180.0, 1*np.pi/180.0, -159*np.pi/180.0, 12*np.pi/180.0, 162*np.pi/180.0, 34*np.pi/180.0]
        rs_1.joint_state = js_1

        joint_constraints_1 = []
        for name,value in zip(js_1.name,js_1.position):
            joint_constraints_1.append(JointConstraint(joint_name=name, position=value, tolerance_below=0.1, tolerance_above=0.1, weight=1.0))
        
        constraint_1 = Constraints(joint_constraints = joint_constraints_1)

        rs_2 = RobotState()
        js_2 = JointState()
        js_2.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        js_2.position = [44*np.pi/180.0, -70*np.pi/180.0, -15*np.pi/180.0, -159*np.pi/180.0, -39*np.pi/180.0, 162*np.pi/180.0, 72*np.pi/180.0]
        rs_2.joint_state = js_2

        joint_constraints_2 = []
        for name,value in zip(js_2.name,js_2.position):
            joint_constraints_2.append(JointConstraint(joint_name=name, position=value, tolerance_below=0.1, tolerance_above=0.1, weight=1.0))
        
        constraint_2 = Constraints(joint_constraints = joint_constraints_2)

        self._scan_hints = [rs_1, rs_2]
        self._constraints = [constraint_1, constraint_2]

        self._scanning_targets = False
        self._target_scan_index = 0

    async def move_cart_callback(self, request, response):
        await self.moveit_api.move_cartesian(request.pose)
        return response

    async def grip_callback(self, request, response):
        await self.moveit_api.move_gripper(0.025, 0.05, 10.0)
        return response
    
    async def set_payload(self, weight:float):
        # SET PAYLOAD

        # deactivate controller
        request = SwitchController.Request()
        request.deactivate_controllers = ["panda_arm_controller"]
        await self.controller_client.call_async(request)

        # set payload
        request = SetLoad.Request()
        request.mass = weight
        request.center_of_mass = [0.0, 0.0, 0.12]
        await self.payload_client.call_async(request)

        # reactivate controller
        request = SwitchController.Request()
        request.activate_controllers = ["panda_arm_controller"]
        await self.controller_client.call_async(request)

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

        target.orientation.x = 1.0
        target.orientation.y = 0.0
        target.orientation.z = 0.0
        target.orientation.w = 0.0

        self.get_logger().info(f"Moving to standoff position.")
        
        await self.moveit_api.plan_and_execute(
            self.moveit_api.plan_position_and_orientation, target
        )
            
        self.get_logger().info("Finished motion to standoff.")
        
        self.get_logger().info("Opening")        
        await self.moveit_api.move_gripper(0.04, 0.5, 10.0)

        target.position.x = request.pose.position.x + self.tag_offset[0]
        target.position.y = request.pose.position.y + self.tag_offset[1]
        target.position.z = request.pose.position.z + self.tag_offset[2]
        
        self.get_logger().info("Moving to pick position.")
        self.get_logger().info(f"Moving to z: {target.position.z}")    

        await self.moveit_api.move_cartesian(target)

        self.get_logger().info("Grasping")
        await self.moveit_api.move_gripper(0.02, 0.5, 50.0)

        await self.set_payload(1.0)

        # Move back to ready
        self.get_logger().info("Standoff")
        
        # move arm to starting location
        target = Pose()
        target.position.x = request.pose.position.x + self.tag_offset[0] + self.ready_offset
        target.position.y = request.pose.position.y + self.tag_offset[1]
        target.position.z = request.pose.position.z + self.tag_offset[2] + 0.25

        target.orientation.x = 1.0
        target.orientation.y = 0.0
        target.orientation.z = 0.0
        target.orientation.w = 0.0

        self.get_logger().info(f"Moving to standoff position.")

        await self.moveit_api.move_cartesian(target)

        return response
    
    async def place_gun_callback(self,request,response):
        '''Replaces gun on the mount after shooting'''
        self.get_logger().info("Initiated place")

        # move arm to starting location
        target = Pose()
        target.position.x = request.pose.position.x + self.tag_offset[0] + self.ready_offset
        target.position.y = request.pose.position.y + self.tag_offset[1]
        target.position.z = request.pose.position.z + self.tag_offset[2] + 0.25

        target.orientation.x = 1.0
        target.orientation.y = 0.0
        target.orientation.z = 0.0
        target.orientation.w = 0.0

        self.get_logger().info(f"Moving to standoff position.")

        await self.moveit_api.move_cartesian(target)

        target.position.x = request.pose.position.x + self.tag_offset[0]
        target.position.y = request.pose.position.y + self.tag_offset[1]
        target.position.z = request.pose.position.z + self.tag_offset[2]
        
        self.get_logger().info("Moving to pick position.")
        self.get_logger().info(f"Moving to z: {target.position.z}")    

        await self.moveit_api.move_cartesian(target)

        self.get_logger().info("Opening")        
        await self.moveit_api.move_gripper(0.04, 0.5, 10.0)

        # move arm to starting location
        target = Pose()
        target.position.x = request.pose.position.x + self.tag_offset[0] + self.ready_offset
        target.position.y = request.pose.position.y + self.tag_offset[1]
        target.position.z = request.pose.position.z + self.tag_offset[2] + 0.25

        target.orientation.x = 1.0
        target.orientation.y = 0.0
        target.orientation.z = 0.0
        target.orientation.w = 0.0

        self.get_logger().info(f"Moving to standoff position.")
        
        await self.moveit_api.plan_and_execute(
            self.moveit_api.plan_position_and_orientation, target
        )
            
        self.get_logger().info("Finished motion to standoff.")

        return response
        

    async def target_scan_callback(self, request, response):
        self.get_logger().info(f"received target scan request. Current status: {self._scanning_targets}, {self._target_scan_index}")

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
                target=self._scan_positions[self._target_scan_index],
                hint_state=self._scan_hints[self._target_scan_index],
                constraints = self._constraints[self._target_scan_index]
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

        await self.moveit_api.plan_and_execute(
            self.moveit_api.plan_position_and_orientation, self._scan_positions[0]
        )
        # await self.moveit_api.move_cartesian(self._scan_positions[0])

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
