#!/usr/bin/env python3
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive


class RobotMotion:
    def __init__(self, node, config):
        self.node = node
        self.config = config

        # ============================================================
        # MoveIt Cartesian Path service
        # ============================================================
        self.cartesian = node.create_client(
            GetCartesianPath,
            "/compute_cartesian_path",
        )

        # ============================================================
        # MoveIt ExecuteTrajectory action
        # ============================================================
        self.execute_client = ActionClient(
            node,
            ExecuteTrajectory,
            "/execute_trajectory",
        )

        # ============================================================
        # MoveIt MoveGroup action
        # Used to move UR3 to the drawing start point
        # ============================================================
        self.move_client = ActionClient(
            node,
            MoveGroup,
            "/move_action",
        )

    # ================================================================
    # Wait for an asynchronous ROS 2 future
    # ================================================================
    def wait_result(self, future, timeout=60.0):
        deadline = time.monotonic() + timeout

        while (
            rclpy.ok()
            and not future.done()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                self.node,
                timeout_sec=0.1,
            )

        if not future.done():
            raise RuntimeError(
                "MoveIt request timed out; "
                "check move_group and simulation"
            )

        return future.result()

    # ================================================================
    # Wait for /compute_cartesian_path
    # ================================================================
    def wait_for_cartesian_service(self):
        self.node.get_logger().info(
            "Waiting for /compute_cartesian_path..."
        )

        if not self.cartesian.wait_for_service(
            timeout_sec=30.0
        ):
            raise RuntimeError(
                "/compute_cartesian_path unavailable; "
                "MoveIt did not start"
            )

        self.node.get_logger().info(
            "/compute_cartesian_path is ready"
        )

    # ================================================================
    # Wait for /execute_trajectory
    # ================================================================
    def wait_for_execute_server(self):
        self.node.get_logger().info(
            "Waiting for /execute_trajectory..."
        )

        if not self.execute_client.wait_for_server(
            timeout_sec=30.0
        ):
            raise RuntimeError(
                "/execute_trajectory unavailable"
            )

        self.node.get_logger().info(
            "/execute_trajectory is ready"
        )

    # ================================================================
    # Compute Cartesian trajectory
    # ================================================================
    def compute_cartesian(self, poses, header, label):
        request = GetCartesianPath.Request()

        # ------------------------------------------------------------
        # Reference frame
        # ------------------------------------------------------------
        request.header = header

        # ------------------------------------------------------------
        # Use current robot state as starting state
        # ------------------------------------------------------------
        request.start_state.is_diff = True

        # ------------------------------------------------------------
        # MoveIt planning group
        # ------------------------------------------------------------
        request.group_name = self.config["group_name"]

        # ------------------------------------------------------------
        # End-effector link
        # ------------------------------------------------------------
        request.link_name = self.config["ee_link"]

        # ------------------------------------------------------------
        # Cartesian waypoints
        # ------------------------------------------------------------
        request.waypoints = poses

        # ------------------------------------------------------------
        # Maximum Cartesian step between consecutive points
        # ------------------------------------------------------------
        request.max_step = self.config["step"]

        # ------------------------------------------------------------
        # Joint-space jump threshold
        # ------------------------------------------------------------
        request.jump_threshold = 5.0

        # ------------------------------------------------------------
        # Revolute joint jump threshold
        # ------------------------------------------------------------
        request.revolute_jump_threshold = 1.0

        # ------------------------------------------------------------
        # Enable collision checking
        # ------------------------------------------------------------
        request.avoid_collisions = True

        # ------------------------------------------------------------
        # Velocity / acceleration scaling
        #
        # These fields are available in the MoveIt interface
        # used by the UR3 Jazzy setup.
        # ------------------------------------------------------------
        if hasattr(
            request,
            "max_velocity_scaling_factor",
        ):
            request.max_velocity_scaling_factor = (
                self.config["speed_scale"]
            )

        if hasattr(
            request,
            "max_acceleration_scaling_factor",
        ):
            request.max_acceleration_scaling_factor = (
                self.config["speed_scale"]
            )

        # ------------------------------------------------------------
        # Send request
        # ------------------------------------------------------------
        future = self.cartesian.call_async(request)

        response = self.wait_result(
            future,
            timeout=60.0,
        )

        # ------------------------------------------------------------
        # Print Cartesian planning result
        # ------------------------------------------------------------
        self.node.get_logger().info(
            f"{label}: "
            f"{response.fraction:.1%}"
        )

        # ------------------------------------------------------------
        # Check MoveIt error code
        # ------------------------------------------------------------
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"{label} failed; "
                f"MoveIt error code = "
                f"{response.error_code.val}"
            )

        # ------------------------------------------------------------
        # Require almost complete Cartesian path
        # ------------------------------------------------------------
        if response.fraction < 0.999:
            raise RuntimeError(
                f"{label} is incomplete "
                f"({response.fraction:.1%}); "
                "robot will not move. "
                "Adjust x/y/z/radius to keep the "
                "trajectory reachable and collision-free."
            )

        # ------------------------------------------------------------
        # Check trajectory is not empty
        # ------------------------------------------------------------
        if not response.solution.joint_trajectory.points:
            raise RuntimeError(
                f"MoveIt returned an empty trajectory "
                f"for {label}"
            )

        self.node.get_logger().info(
            f"{label}: Cartesian trajectory is valid"
        )

        return response.solution

    # ================================================================
    # Move UR3 to the beginning of the drawing
    # ================================================================
    def move_to_start(self):
        # ------------------------------------------------------------
        # Wait for MoveGroup action server
        # ------------------------------------------------------------
        self.node.get_logger().info(
            "Waiting for /move_action..."
        )

        if not self.move_client.wait_for_server(
            timeout_sec=30.0
        ):
            raise RuntimeError(
                "/move_action unavailable; "
                "MoveIt did not start"
            )

        self.node.get_logger().info(
            "/move_action is ready"
        )

        # Small delay to make sure the simulation state is stable
        time.sleep(1.0)

        # ============================================================
        # Target pose
        # ============================================================
        target = Pose()

        target.position.x = self.config["x"]
        target.position.y = self.config["y"]
        target.position.z = self.config["z"]

        # Orientation:
        #
        # Quaternion:
        #     x = 1
        #     y = 0
        #     z = 0
        #     w = 0
        #
        # This corresponds to the tool orientation used by
        # the original UR3 drawing implementation.
        # ============================================================
        target.orientation.x = 1.0
        target.orientation.y = 0.0
        target.orientation.z = 0.0
        target.orientation.w = 0.0

        # ============================================================
        # Position constraint
        # ============================================================
        region = BoundingVolume()

        region.primitives = [
            SolidPrimitive(
                type=SolidPrimitive.SPHERE,
                dimensions=[0.003],
            )
        ]

        region.primitive_poses = [
            target
        ]

        # ============================================================
        # Reference frame
        # ============================================================
        header = PoseStamped().header
        header.frame_id = self.config["frame_id"]

        # ============================================================
        # Create MoveIt constraints
        # ============================================================
        constraints = Constraints()
        constraints.name = "circle_start"

        # ------------------------------------------------------------
        # Position constraint
        # ------------------------------------------------------------
        constraints.position_constraints = [
            PositionConstraint(
                header=header,
                link_name=self.config["ee_link"],
                constraint_region=region,
                weight=1.0,
            )
        ]

        # ------------------------------------------------------------
        # Orientation constraint
        # ------------------------------------------------------------
        constraints.orientation_constraints = [
            OrientationConstraint(
                header=header,
                link_name=self.config["ee_link"],
                orientation=target.orientation,

                # Tight orientation tolerances
                absolute_x_axis_tolerance=0.02,
                absolute_y_axis_tolerance=0.02,
                absolute_z_axis_tolerance=0.02,

                weight=1.0,
            )
        ]

        # ============================================================
        # Build MoveGroup goal
        # ============================================================
        goal = MoveGroup.Goal()

        # Planning group
        goal.request.group_name = (
            self.config["group_name"]
        )

        # OMPL / MoveIt pipeline
        goal.request.pipeline_id = "ompl"

        # Number of planning attempts
        goal.request.num_planning_attempts = 10

        # Maximum planning time
        goal.request.allowed_planning_time = 10.0

        # ------------------------------------------------------------
        # Approach speed
        #
        # Limit the speed when moving to the drawing origin.
        # ============================================================
        approach_scale = min(
            self.config["speed_scale"],
            0.05,
        )

        goal.request.max_velocity_scaling_factor = (
            approach_scale
        )

        goal.request.max_acceleration_scaling_factor = (
            approach_scale
        )

        # Use current robot state
        goal.request.start_state.is_diff = True

        # Add target constraints
        goal.request.goal_constraints = [
            constraints
        ]

        # ============================================================
        # Planning options
        # ============================================================
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        # ============================================================
        # Send MoveGroup goal
        # ============================================================
        self.node.get_logger().info(
            "Planning motion to drawing start..."
        )

        goal_future = self.move_client.send_goal_async(
            goal
        )

        handle = self.wait_result(
            goal_future,
            timeout=30.0,
        )

        # ------------------------------------------------------------
        # Check goal acceptance
        # ------------------------------------------------------------
        if not handle.accepted:
            raise RuntimeError(
                "MoveIt rejected the approach goal"
            )

        self.node.get_logger().info(
            "MoveIt accepted the approach goal"
        )

        # ============================================================
        # Wait for planning/execution result
        # ============================================================
        result_future = handle.get_result_async()

        result = self.wait_result(
            result_future,
            timeout=90.0,
        )

        error_code = result.result.error_code.val

        self.node.get_logger().info(
            f"MoveGroup result error_code = {error_code}"
        )

        if error_code != MoveItErrorCodes.SUCCESS:
            if error_code == MoveItErrorCodes.CONTROL_FAILED:
                raise RuntimeError(
                    "The approach was planned, but Gazebo "
                    "could not track the final goal."
                )

            raise RuntimeError(
                "MoveIt could not reach the circle start. "
                f"MoveIt error code = {error_code}."
            )

        self.node.get_logger().info(
            "Reached the circle start using MoveIt"
        )

    # ================================================================
    # Execute the computed trajectory
    # ================================================================
    def execute(self, trajectory):
        # ------------------------------------------------------------
        # Create ExecuteTrajectory goal
        # ------------------------------------------------------------
        goal = ExecuteTrajectory.Goal()

        goal.trajectory = trajectory

        # ------------------------------------------------------------
        # Send trajectory to MoveIt
        #
        # MoveIt will select the configured controller
        # (scaled_joint_trajectory_controller).
        # ------------------------------------------------------------
        self.node.get_logger().info(
            "Sending trajectory to MoveIt..."
        )

        goal_future = (
            self.execute_client.send_goal_async(goal)
        )

        handle = self.wait_result(
            goal_future,
            timeout=30.0,
        )

        # ------------------------------------------------------------
        # Check acceptance
        # ------------------------------------------------------------
        if not handle.accepted:
            raise RuntimeError(
                "MoveIt rejected trajectory execution"
            )

        self.node.get_logger().info(
            "MoveIt accepted trajectory execution"
        )

        # ============================================================
        # Calculate suitable timeout
        # ============================================================
        result_future = handle.get_result_async()

        try:
            if (
                trajectory.joint_trajectory.points
            ):
                duration = (
                    trajectory
                    .joint_trajectory
                    .points[-1]
                    .time_from_start
                )

                timeout = max(
                    60.0,
                    duration.sec
                    + duration.nanosec / 1e9
                    + 30.0,
                )
            else:
                timeout = 60.0

            # --------------------------------------------------------
            # Wait for execution result
            # --------------------------------------------------------
            result = self.wait_result(
                result_future,
                timeout,
            )

        except (
            RuntimeError,
            KeyboardInterrupt,
        ):
            # --------------------------------------------------------
            # Cancel execution if something goes wrong
            # --------------------------------------------------------
            self.node.get_logger().warn(
                "Cancelling trajectory execution..."
            )

            self.wait_result(
                handle.cancel_goal_async(),
                timeout=5.0,
            )

            raise

        # ============================================================
        # Check execution result
        # ============================================================
        error_code = result.result.error_code.val

        if error_code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                "Execution failed: "
                f"MoveIt error code = {error_code}"
            )

        self.node.get_logger().info(
            "Trajectory execution completed successfully"
        )