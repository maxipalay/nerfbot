"""
    A MoveIt Api that allows the user to plan and execute paths easily.

    Uses an already running move group node.

    PUBLISHERS:
        planning_scene (moveit_msgs/msg/PlanningScene): represents all the information
        needed to compute motion plans

    SERVICE CLIENTS:
        compute_ik (moveit_msgs/srv/GetPositionIK):
            Computes Inverse Kinematics.
        get_planning_scene (moveit_msgs/srv/GetPlanningScene) -
            Gets the planning scene message.

    ACTION CLIENTS:
        execute_trajectory (moveit_msgs/action/ExecuteTrajectory):
            Executes a planned joint trajectory.
        move_action (moveit_msgs/action/MoveGroup):
            Plans and executes the robot's trajectory.
        grippercommand (control_msgs/GripperCommand.action):
            Controls movement of the gripper.

"""
from rclpy.node import Node
from moveit_msgs.msg import MotionPlanRequest, WorkspaceParameters, RobotState, Constraints, \
    JointConstraint, PlanningOptions, PositionIKRequest, OrientationConstraint, CollisionObject, \
    PlanningScene
from moveit_msgs.srv import GetPlanningScene, GetPositionIK
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Vector3, Pose, PoseStamped, Quaternion


class MoveItAPI():

    def __init__(self, node: Node, planning_group: str, gripper_action: str,
                 pipeline_id: str, constraints_link: str):
        self._node = node
        self._planning_group = planning_group
        self._gripper_action = gripper_action
        self._pipeline_id = pipeline_id
        self._constraints_link = constraints_link

        # Clients
        self._node.cli = self._node.create_client(GetPlanningScene, 'get_planning_scene',
                                                  callback_group=self._node._cbgrp)
        self._node.ik_cli = self._node.create_client(GetPositionIK, 'compute_ik',
                                                     callback_group=self._node._cbgrp)
        self._node._action_client = ActionClient(self._node,
                                                 MoveGroup, 'move_action',
                                                 callback_group=self._node._cbgrp)
        self._node._execute_client = ActionClient(self._node,
                                                  ExecuteTrajectory, "execute_trajectory",
                                                  callback_group=self._node._cbgrp)
        self._node._gripper_action_client = ActionClient(self._node,
                                                         GripperCommand, self._gripper_action,
                                                         callback_group=self._node._cbgrp)

        # Publisher
        self._node.pscene_pub = self._node.create_publisher(PlanningScene, 'planning_scene', 10)

        # Wait for clients to become available
        while not self._node.cli.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().debug('Planning scene service not available, waiting again...')

        while not self._node.ik_cli.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().debug('IK service not available, waiting again...')

        self._node._action_client.wait_for_server()

    async def move_gripper(self, width: float,
                           effort: float = 10.0) -> (bool, GripperCommand.Result()):
        """Move the gripper to a specified position."""
        req = GripperCommand.Goal()
        req.command.position = width
        req.command.max_effort = effort

        future = await self._node._gripper_action_client.send_goal_async(req)

        result = await future.get_result_async()

        if result.result.reached_goal:
            return True, result.result

        return False, result.result

    async def get_planning_scene(self) -> GetPlanningScene.Response():
        """Request the planning scene from the MoveGroup."""
        # Request the planning scene Planning Scene Request
        req = GetPlanningScene.Request()
        self._node.future = await self._node.cli.call_async(req)
        planning_scene = self._node.future
        return planning_scene

    async def compute_ik(self, target_pose: Pose) -> (bool, RobotState):
        # GETTING THE ROBOT STATE FROM THE SCENE
        # Request the planning scene Planning Scene Request
        planning_scene = await self.get_planning_scene()

        # Assemble the motion plan request
        ik_req = PositionIKRequest()
        ik_req.robot_state.joint_state = planning_scene.scene.robot_state.joint_state
        ik_req.group_name = self._planning_group
        ik_req.timeout = Duration(sec=5)

        ik_req.pose_stamped = PoseStamped(
            header=Header(
                stamp=self._node.get_clock().now().to_msg()),
            pose=target_pose)

        future = await self._node.ik_cli.call_async(GetPositionIK.Request(ik_request=ik_req))

        goal_state = future.solution
        if future.error_code.val == -31:
            self._node.get_logger().debug('No IK Solution Found.')
        elif future.error_code.val == 1:
            self._node.get_logger().debug('IK Solution Found')
            return True, goal_state
        return False, None

    async def request_motion_plan(self, goal_state, goal_orientation, start_state, execute):
        planning_scene = await self.get_planning_scene()
        # Assemble the motion plan request
        ts = self._node.get_clock().now().to_msg()  # timestamp

        # WORKSPACE PARAMETERS
        workspace_params = WorkspaceParameters(
            header=Header(
                stamp=ts,
                frame_id=planning_scene.scene.robot_state.joint_state.header.frame_id),
            min_corner=Vector3(x=-1.0, y=-1.0, z=-1.0),
            max_corner=Vector3(x=1.0, y=1.0, z=1.0))

        # START STATEself.get_logger().info
        if start_state is None:
            start_state = planning_scene.scene.robot_state

        # Joint constraints / GOAL STATE
        joint_constraints = []

        if goal_state is not None:
            i = 0
            for joint in planning_scene.scene.robot_state.joint_state.name:
                constraint = JointConstraint(
                            joint_name=joint,
                            position=goal_state.joint_state.position[i],
                            tolerance_above=0.0001,
                            tolerance_below=0.0001,
                            weight=1.0)
                joint_constraints.append(constraint)
                i += 1

        # Orientation_constraints
        orientation_constraints = []
        if goal_orientation is not None:
            orientation_constraints = []
            constraint = OrientationConstraint(
                        link_name=self._constraints_link,
                        orientation=goal_orientation,
                        absolute_x_axis_tolerance=0.0001,
                        absolute_y_axis_tolerance=0.0001,
                        absolute_z_axis_tolerance=0.0001,
                        weight=1.0)
            orientation_constraints.append(constraint)

        goal_constraints = [Constraints(joint_constraints=joint_constraints,
                                        orientation_constraints=orientation_constraints)]
        path_constraints = Constraints(orientation_constraints=orientation_constraints)

        # PIPELINE_ ID
        pipeline_id = self._pipeline_id

        # PLANNER ID
        # planner_id=''

        # GROUP NAME
        group_name = self._planning_group

        # PLANNING ATTEMPTS
        num_planning_attempts = 10  # should be configurable

        # PLANNING TIME
        allowed_planning_time = 5.0  # should be configurable

        # MAX VELOCITY
        max_velocity_scaling_factor = 0.1  # should be configurable

        # MAX ACCELERATION
        max_acceleration_scaling_factor = 0.1  # should be configurable

        # assemble motion plan request
        motion_request = MotionPlanRequest(
            workspace_parameters=workspace_params,
            start_state=start_state,
            goal_constraints=goal_constraints,
            path_constraints=path_constraints,
            pipeline_id=pipeline_id,
            group_name=group_name,
            num_planning_attempts=num_planning_attempts,
            allowed_planning_time=allowed_planning_time,
            max_velocity_scaling_factor=max_velocity_scaling_factor,
            max_acceleration_scaling_factor=max_acceleration_scaling_factor
            )

        # planning options
        planning_options = PlanningOptions()

        # set plan only! so it does not execute
        planning_options.plan_only = not execute

        move_group_req = MoveGroup.Goal(
            request=motion_request,
            planning_options=planning_options)

        self._node.future = await self._node._action_client.send_goal_async(move_group_req)

        goal_handle = self._node.future
        if not goal_handle.accepted:
            self._node.get_logger().debug("MoveGroup rejected the goal!")

        self._node._get_result_future = await goal_handle.get_result_async()

        if self._node._get_result_future.result.error_code.val == 1:
            # success!
            self._node.get_logger().debug("MoveGroup returned motion plan!")
            return True, self._node._get_result_future.result
        else:
            self._node.get_logger().debug("Error on MoveGroup response!")

        return False, None

    async def plan_position(self, target: Pose, start_state: RobotState = None):
        """
        Plan position without orientation.

        Args
        ----
            target - desired end position
            start_state - robot start if not current

        Returns
        -------
            success - T/F plan was generated
            plan - motion plan for move node

        """
        success, goal_state = await self.compute_ik(target)
        print("ik: "+str(success))

        # Plan trajectory to desired pose
        success, plan = await self.request_motion_plan(goal_state, goal_orientation=None,
                                                       start_state=start_state, execute=False)
        executed = "N/A"

        return success, plan, executed

    async def plan_orientation(self, target: Pose, start_state: RobotState = None):
        """
        Plan orientation without position.

        Args
        ----
            target - desired end-effector orientation
            start_state - robot start if not current

        Returns
        -------
            success - T/F plan was generated
            plan - motion plan for move node

        """
        orientation = Quaternion()
        orientation.w = target.orientation.w
        orientation.x = target.orientation.x
        orientation.y = target.orientation.y
        orientation.z = target.orientation.z

        # Plan trajectory to desired orientation
        success, plan = await self.request_motion_plan(goal_state=None,
                                                       goal_orientation=orientation,
                                                       start_state=start_state,
                                                       execute=False)
        executed = "N/A"

        return success, plan, executed

    async def plan_position_and_orientation(self, target: Pose,
                                            start_state: RobotState = None):
        """
        Plan both position and orientation.

        Args
        ----
            target - desired end-effector position and orientation
            start_state - robot start if not current

        Returns
        -------
            success - T/F plan was generated
            plan - motion plan for move node

        """
        success, goal_state = await self.compute_ik(target)
        print("ik: "+str(success))

        # Plan trajectory to desired pose
        success, plan = await self.request_motion_plan(goal_state=goal_state,
                                                       goal_orientation=None,
                                                       start_state=start_state,
                                                       execute=False)
        executed = "N/A"

        return success, plan, executed

    async def plan_and_execute(self, planner, target: Pose,
                               start_state: RobotState = None):
        """
        Plan AND execute.

        Args
        ----
            planner - planning function to use
            start_state - robot start if not current

        Returns
        -------
            success - T/F plan was generated
            plan - motion plan for move node

        """
        success, plan, executed = await planner(target, start_state=start_state)
        if success:
            executed, response = await self.request_execute(plan)

        return success, plan, executed

    async def add_collison_objects(self, item, pose_stamped):
        """
        Add collision object to planning scene.

        Args
        ----
            item - item to be added
            pose_stamped - position of item added(Pose_stamped message)

        Returns
        -------
            Null

        """
        ps = await self.get_planning_scene()
        col_obj = CollisionObject()
        col_obj.header = pose_stamped.header
        col_obj.pose = pose_stamped.pose
        col_obj.primitives = [item]
        col_obj.primitive_poses = [Pose()]
        ps.scene.world.collision_objects.append(col_obj)
        self._node.pscene_pub.publish(ps.scene)

    async def request_execute(self,
                              motion_plan=MoveGroup.Result()) -> (bool,
                                                                  ExecuteTrajectory.Result()):
        """Request execution of motion plan."""
        execute_request = ExecuteTrajectory.Goal()
        execute_request.trajectory = motion_plan.planned_trajectory
        future_response = await self._node._execute_client.send_goal_async(execute_request)
        goal_handle = future_response
        if not goal_handle.accepted:
            self._node.get_logger().debug("MoveGroup rejected the execute!")
        execute_response = await future_response.get_result_async()
        if execute_response.result.error_code.val != 1:
            self._node.get_logger().debug("MoveGroup rejected the execute!")
            return False, execute_response
        else:
            return True, execute_response
