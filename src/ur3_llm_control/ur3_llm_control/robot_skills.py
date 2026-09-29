import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import math
import copy
import subprocess
import threading

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, Constraints, JointConstraint, PlanningScene, PositionConstraint, OrientationConstraint, BoundingVolume, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import Float64MultiArray
import yaml

class RobotSkills(Node):
    def __init__(self, scene_config_path):
        super().__init__('robot_skills')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.gazebo_cube_timer = self.create_timer(
               0.1,
        self.update_gazebo_cube
        )
        # Load scene config
        with open(scene_config_path, 'r') as f:
            self.scene = yaml.safe_load(f)
        table = self.scene['table']
        table_top = table['center'][2] + table['size'][2] / 2
        self.robot_base_height = self.scene.get('robot_base_height', 0.0)
            
        self.group_name = "ur_manipulator"
        self.ee_link = "tool0"
        self.frame_id = "base_link"
        self.motion_scaling_factor = 0.2
        self.gripper_reach = 0.06
        
        # Action clients
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.gripper_joint_name = "robotiq_85_left_knuckle_joint"
        self.gripper_joint_states = {}
        self.gripper_joint_names = [
            "robotiq_85_left_knuckle_joint",
            "robotiq_85_right_knuckle_joint",
            "robotiq_85_left_inner_knuckle_joint",
            "robotiq_85_right_inner_knuckle_joint",
            "robotiq_85_left_finger_tip_joint",
            "robotiq_85_right_finger_tip_joint",
        ]
        self.gripper_command_publisher = self.create_publisher(
            Float64MultiArray, "/robotiq_gripper_controller/commands", 10
        )
        self.gripper_state_subscription = self.create_subscription(
            JointState, "/joint_states", self.gripper_state_callback, 10
        )
        
        # Wait for servers
        self.get_logger().info("Waiting for MoveIt servers...")
        self.move_client.wait_for_server()
        self.cartesian_client.wait_for_service()
        self.ik_client.wait_for_service()
        self.execute_client.wait_for_server()
        self.scene_client.wait_for_service()
        self.get_logger().info("MoveIt servers ready.")
        
        # Hardcoded orientation for pointing down
        self.down_orientation = Pose().orientation
        self.down_orientation.x = 1.0
        self.down_orientation.y = 0.0
        self.down_orientation.z = 0.0
        self.down_orientation.w = 0.0
        self.gazebo_cube_name = None
        self.gazebo_cube_attached = False
        self.attached_object = None
     


    def set_gazebo_model_pose(self, model_name, x, y, z):
        """Dat vi tri model trong Gazebo."""
        request = (
            f'name: "{model_name}" '
            f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}} '
            f'orientation: {{w: 1.0}}'
        )

        try:
            result = subprocess.run(
                [
                    "gz",
                    "service",
                    "-s", "/world/empty/set_pose",
                    "--reqtype", "gz.msgs.Pose",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "2000",
                    "--req", request
                ],
                capture_output=True,
                text=True,
                timeout=3.0
            )

            if "data: true" in result.stdout:
                return True

            self.get_logger().error(
                f"Gazebo set_pose failed: {result.stdout.strip()}"
            )
            return False

        except Exception as error:
            self.get_logger().error(
                f"Gazebo set_pose error: {error}"
            )
            return False

    def get_tool0_position(self):
        """Lay vi tri tool0 trong he toa do base_link."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.ee_link,
                rclpy.time.Time()
            )

            t = transform.transform.translation

            return t.x, t.y, t.z

        except Exception as error:
            self.get_logger().warn(
                f"Khong lay duoc TF {self.frame_id} -> {self.ee_link}: {error}"
            )
            return None

    def update_gazebo_cube(self):
        """Cap nhat vi tri vat the dang gap theo tool0."""
        if not self.gazebo_cube_attached:
            return

        if self.gazebo_cube_name is None:
            return

        position = self.get_tool0_position()

        if position is None:
            return

        x, y, z = position

        # Vat the nam phia truoc va duoi tool0
        cube_x = x - 0.014
        cube_y = y
        cube_z = z - 0.110

        self.set_gazebo_model_pose(
            self.gazebo_cube_name,
            cube_x,
            cube_y,
            cube_z
        )
    def gripper_state_callback(self, message):
        for index, name in enumerate(message.name):
            if name != self.gripper_joint_name or index >= len(message.position):
                continue
            velocity = message.velocity[index] if index < len(message.velocity) else 0.0
            self.gripper_joint_states[name] = (message.position[index], velocity)

    def command_gripper(self, position, require_object=False):
        # Robotiq 2F-85:
        # left_knuckle        =  +q
        # right_knuckle       =  -q
        # left_inner          =  +q
        # right_inner         = -q
        # left_finger_tip     =  -q
        # right_finger_tip    =  +q

        target = [
            position,
            -position,
            position,
            -position,
            -position,
            position
        ]

        command = Float64MultiArray(data=target)

        deadline = time.monotonic() + (8.0 if require_object else 5.0)

        while rclpy.ok() and time.monotonic() < deadline:
            self.gripper_command_publisher.publish(command)

            rclpy.spin_once(self, timeout_sec=0.1)

            if self.gripper_joint_name not in self.gripper_joint_states:
                continue

            master_position, master_velocity = self.gripper_joint_states[
                self.gripper_joint_name
            ]

            # Gripper mở
            if position == 0.0:
                if abs(master_position) < 0.04:
                    self.get_logger().info("Gripper opened.")
                    return True
                continue

            # Gripper đang đóng
            position_error = abs(master_position - position)

            if position_error < 0.03 and abs(master_velocity) < 0.05:
                self.get_logger().info(
                    f"Gripper reached target position: {master_position:.3f}"
                )
                return True

        self.get_logger().error(
            f"Gripper failed to reach target position {position:.3f}"
        )
        return False

    def update_attached_object(self, obj_name, attached, world_position=None):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        attached_object = AttachedCollisionObject()
        attached_object.link_name = self.ee_link
        attached_object.object.id = obj_name

        if attached:
            attached_object.object.header.frame_id = self.ee_link
            box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.04, 0.04, 0.04])
            box_pose = Pose()
            box_pose.position.x = -0.014
            box_pose.position.y = 0.0
            box_pose.position.z = 0.110
            attached_object.object.primitives = [box]
            attached_object.object.primitive_poses = [box_pose]
            attached_object.object.operation = CollisionObject.ADD
            attached_object.touch_links = [
                "robotiq_85_base_link",
                "robotiq_85_left_finger_link",
                "robotiq_85_right_finger_link",
                "robotiq_85_left_finger_tip_link",
                "robotiq_85_right_finger_tip_link",
                "robotiq_85_left_contact_pad",
                "robotiq_85_right_contact_pad",
                "robotiq_85_left_inner_knuckle_link",
                "robotiq_85_right_inner_knuckle_link",
            ]
        else:
            attached_object.object.operation = CollisionObject.REMOVE
            world_object = CollisionObject()
            world_object.id = obj_name
            world_object.header.frame_id = self.frame_id
            world_object.operation = CollisionObject.ADD
            box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.04, 0.04, 0.04])
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = world_position
            world_object.primitives = [box]
            world_object.primitive_poses = [pose]
            scene.world.collision_objects.append(world_object)

        scene.robot_state.attached_collision_objects = [attached_object]
        request = ApplyPlanningScene.Request(scene=scene)
        try:
            result = self.wait_result(self.scene_client.call_async(request), timeout=5.0)
            return result.success
        except Exception as error:
            self.get_logger().error(f"Failed to update planning scene: {error}")
            return False

    def wait_result(self, future, timeout=60.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise RuntimeError("Timeout")
        return future.result()

    def home(self):
        self.get_logger().info("Moving to HOME...")
        # Since we use MoveGroup action, we can request named target 'home'
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        
        # We don't have a direct named target in the action request easily without using the MoveItCpp/Python API
        # but we can set joint constraints.
        # Home state: shoulder_pan_joint: 0, shoulder_lift_joint: -1.5707, elbow_joint: 0, wrist_1_joint: 0, wrist_2_joint: 0, wrist_3_joint: 0
        from moveit_msgs.msg import JointConstraint
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        vals = [0.0, -1.5707, 0.0, 0.0, 0.0, 0.0]
        
        constraints = Constraints()
        constraints.name = "home"
        for j, v in zip(joints, vals):
            jc = JointConstraint()
            jc.joint_name = j
            jc.position = v
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            
        goal.request.goal_constraints = [constraints]
        goal.request.start_state.is_diff = True
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 3
        goal.request.max_velocity_scaling_factor = self.motion_scaling_factor
        goal.request.max_acceleration_scaling_factor = self.motion_scaling_factor
        
        goal_future = self.move_client.send_goal_async(goal)
        try:
            handle = self.wait_result(goal_future, timeout=10.0)
            if not handle.accepted:
                return "PLANNING_FAILED"
            result_future = handle.get_result_async()
            result = self.wait_result(result_future, timeout=30.0)
            if result.result.error_code.val == MoveItErrorCodes.SUCCESS:
                return "SUCCESS"
            else:
                self.get_logger().error(
                    f"Home MoveGroup failed with error code {result.result.error_code.val}"
                )
                return "EXECUTION_FAILED"
        except Exception as e:
            self.get_logger().error(f"Home failed: {e}")
            return "EXECUTION_FAILED"

    def move_to_pose(self, x, y, z):
        target = Pose()
        target.position.x = x
        target.position.y = y
        target.position.z = z - self.robot_base_height
        target.orientation = self.down_orientation

        region = BoundingVolume()
        region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])]
        region.primitive_poses = [target]

        header = PoseStamped().header
        header.frame_id = self.frame_id

        position_constraint = PositionConstraint(
            header=header,
            link_name=self.ee_link,
            constraint_region=region,
            weight=1.0,
        )

        for attempt, tolerance in enumerate((0.15, 0.25)):
            orientation_constraint = OrientationConstraint(
                header=header,
                link_name=self.ee_link,
                orientation=target.orientation,
                absolute_x_axis_tolerance=tolerance,
                absolute_y_axis_tolerance=tolerance,
                absolute_z_axis_tolerance=tolerance,
                weight=1.0,
            )
            constraints = Constraints(
                position_constraints=[position_constraint],
                orientation_constraints=[orientation_constraint],
            )

            goal = MoveGroup.Goal()
            goal.request.group_name = self.group_name
            goal.request.pipeline_id = "ompl"
            goal.request.goal_constraints = [constraints]
            goal.request.start_state.is_diff = True
            goal.request.allowed_planning_time = 20.0
            goal.request.num_planning_attempts = 20
            goal.request.max_velocity_scaling_factor = self.motion_scaling_factor
            goal.request.max_acceleration_scaling_factor = self.motion_scaling_factor

            try:
                handle = self.wait_result(
                    self.move_client.send_goal_async(goal), timeout=30.0
                )
                if not handle.accepted:
                    self.get_logger().error(
                        f"MoveIt rejected pose goal at ({x:.3f}, {y:.3f}, {z:.3f})"
                    )
                    return "PLANNING_FAILED"
                result = self.wait_result(handle.get_result_async(), timeout=60.0)
                error_code = result.result.error_code.val
                if error_code == MoveItErrorCodes.SUCCESS:
                    return "SUCCESS"
                if error_code not in (
                    MoveItErrorCodes.PLANNING_FAILED,
                    MoveItErrorCodes.FAILURE,
                ):
                    self.get_logger().error(
                        f"MoveIt failed at ({x:.3f}, {y:.3f}, {z:.3f}) "
                        f"with error code: {error_code}"
                    )
                    return "EXECUTION_FAILED"
                if attempt == 0:
                    self.get_logger().warn(
                        f"Planning failed with 0.15 rad orientation tolerance; "
                        "retrying with 0.25 rad"
                    )
                else:
                    self.get_logger().error(
                        f"MoveIt failed at ({x:.3f}, {y:.3f}, {z:.3f}) "
                        f"with error code: {error_code}"
                    )
            except Exception as e:
                self.get_logger().error(f"Move failed: {e}")
                return "PLANNING_FAILED"

        self.get_logger().warn(
            "Pose planning failed; trying a collision-aware IK joint goal"
        )
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = self.group_name
        ik_request.ik_request.robot_state.is_diff = True
        ik_request.ik_request.avoid_collisions = True
        ik_request.ik_request.ik_link_name = self.ee_link
        ik_request.ik_request.pose_stamped = PoseStamped()
        ik_request.ik_request.pose_stamped.header.frame_id = self.frame_id
        ik_request.ik_request.pose_stamped.pose = target
        ik_request.ik_request.timeout.sec = 2

        try:
            ik_result = self.wait_result(
                self.ik_client.call_async(ik_request), timeout=5.0
            )
            if ik_result.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().error(
                    f"No collision-free IK solution at ({x:.3f}, {y:.3f}, {z:.3f}); "
                    f"error_code={ik_result.error_code.val}"
                )
                return "PLANNING_FAILED"

            joint_constraints = []
            for joint_name, joint_position in zip(
                ik_result.solution.joint_state.name,
                ik_result.solution.joint_state.position,
            ):
                constraint = JointConstraint()
                constraint.joint_name = joint_name
                constraint.position = joint_position
                constraint.tolerance_above = 0.2
                constraint.tolerance_below = 0.2
                constraint.weight = 1.0
                joint_constraints.append(constraint)

            constraints = Constraints(joint_constraints=joint_constraints)
            goal = MoveGroup.Goal()
            goal.request.group_name = self.group_name
            goal.request.pipeline_id = "ompl"
            goal.request.goal_constraints = [constraints]
            goal.request.start_state.is_diff = True
            goal.request.allowed_planning_time = 20.0
            goal.request.num_planning_attempts = 10
            goal.request.max_velocity_scaling_factor = self.motion_scaling_factor
            goal.request.max_acceleration_scaling_factor = self.motion_scaling_factor

            handle = self.wait_result(
                self.move_client.send_goal_async(goal), timeout=30.0
            )
            if not handle.accepted:
                self.get_logger().error("MoveIt rejected IK-seeded joint goal")
                return "PLANNING_FAILED"
            result = self.wait_result(handle.get_result_async(), timeout=60.0)
            error_code = result.result.error_code.val
            if error_code == MoveItErrorCodes.SUCCESS:
                return "SUCCESS"
            self.get_logger().error(
                f"IK-seeded joint planning/execution failed with error_code={error_code}"
            )
            return (
                "EXECUTION_FAILED"
                if error_code == MoveItErrorCodes.CONTROL_FAILED
                else "PLANNING_FAILED"
            )
        except Exception as e:
            self.get_logger().error(f"IK-seeded planning failed: {e}")
            return "PLANNING_FAILED"
    

    def wait_trajectory_result(self, future, timeout=60.0):
        """Cho trajectory chay va cap nhat vat the dang gap."""
        deadline = time.monotonic() + timeout
        last_update = 0.0

        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            now = time.monotonic()

            if (
                self.gazebo_cube_attached
                and now - last_update >= 0.1
            ):
                self.update_gazebo_cube()
                last_update = now

            rclpy.spin_once(self, timeout_sec=0.05)

        if not future.done():
            raise RuntimeError("Trajectory execution timeout")

        return future.result()
    def move_cartesian(self, waypoints):
        request = GetCartesianPath.Request()
        request.header.frame_id = self.frame_id
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state.is_diff = True
        request.group_name = self.group_name
        request.link_name = self.ee_link
        request.max_step = 0.005
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = self.motion_scaling_factor
        request.max_acceleration_scaling_factor = self.motion_scaling_factor

        for x, y, z in waypoints:
            waypoint = Pose()
            waypoint.position.x = x
            waypoint.position.y = y
            waypoint.position.z = z - self.robot_base_height
            waypoint.orientation = self.down_orientation
            request.waypoints.append(waypoint)

        try:
            path = self.wait_result(
                self.cartesian_client.call_async(request), timeout=30.0
            )
            if path.error_code.val != MoveItErrorCodes.SUCCESS or path.fraction < 0.99:
                self.get_logger().error(
                    f"Cartesian path incomplete: fraction={path.fraction:.3f}, "
                    f"error_code={path.error_code.val}"
                )
                return "PLANNING_FAILED"

            goal = ExecuteTrajectory.Goal()
            goal.trajectory = path.solution
            goal_future = self.execute_client.send_goal_async(goal)
            handle = self.wait_result(goal_future, timeout=30.0)
            if not handle.accepted:
                self.get_logger().error("MoveIt rejected Cartesian trajectory")
                return "PLANNING_FAILED"

            result = self.wait_trajectory_result(
                   handle.get_result_async(),
                    timeout=60.0
) 
            if result.result.error_code.val == MoveItErrorCodes.SUCCESS:
                return "SUCCESS"
            self.get_logger().error(
                f"Cartesian execution failed with error code "
                f"{result.result.error_code.val}"
            )
            return "EXECUTION_FAILED"
        except Exception as e:
            self.get_logger().error(f"Cartesian motion failed: {e}")
            return "PLANNING_FAILED"

    def pick(self, obj_name):
        if obj_name not in self.scene.get('objects', {}):
            return "INVALID_OBJECT"
            
        pos = self.scene['objects'][obj_name]['position']
        x, y, z = pos[0], pos[1], pos[2]
        
        self.get_logger().info("Opening Robotiq gripper...")
        if not self.command_gripper(0.0):
            return "EXECUTION_FAILED"

        self.get_logger().info(f"Moving above {obj_name}...")
        # Cube cao khoang 4 cm, z la tam cube.
# Dua gripper xuong de tam kep nam gan giua cube.
        grasp_z = z + 0.06
        approach_z = grasp_z + 0.05
        result = self.move_to_pose(x, y, approach_z)
        if result != "SUCCESS":
            return result
            
        self.get_logger().info(f"Lowering to grasp {obj_name}...")
        result = self.move_cartesian([(x, y, grasp_z)])
        if result != "SUCCESS":
            return result
            
        self.get_logger().info(f"Closing Robotiq gripper on {obj_name}...")
        if not self.command_gripper(0.22, require_object=True):
            return "EXECUTION_FAILED"
        if not self.update_attached_object(obj_name, attached=True):
            return "EXECUTION_FAILED"

        self.attached_object = obj_name

# Bat che do bam vat the trong Gazebo
        self.gazebo_cube_name = obj_name
        self.gazebo_cube_attached = True
        self.get_logger().info(f"Lifting {obj_name}...")
        result = self.move_to_pose(x, y, approach_z)
        if result != "SUCCESS":
            return result
            
        return "SUCCESS"

    def place(self, obj_name, zone_name):
        if obj_name not in self.scene.get('objects', {}):
            return "INVALID_OBJECT"
        if zone_name not in self.scene.get('zones', {}):
            return "INVALID_ZONE"
            
        if self.attached_object != obj_name:
            self.get_logger().error(f"Cannot place {obj_name}: it is not attached")
            return "NOT_ATTACHED"
            
        pos = self.scene['zones'][zone_name]['position']
        x, y, z = pos[0], pos[1], pos[2]
        
        self.get_logger().info(f"Moving above {zone_name}...")
        release_z = z + self.gripper_reach + 0.02
        approach_z = release_z + 0.05
        result = self.move_cartesian([(x, y, approach_z)])
        if result != "SUCCESS":
            return result
            
        self.get_logger().info(f"Lowering to place in {zone_name}...")
        result = self.move_cartesian([(x, y, release_z)])
        if result != "SUCCESS":
            return result
            
        self.get_logger().info(f"Opening Robotiq gripper to release {obj_name}...")
        if not self.command_gripper(0.0):
            return "EXECUTION_FAILED"

    
        self.gazebo_cube_attached = False
        placed_position = [x, y, z + 0.021]

        self.get_logger().info("Retreating...")
        retreat_result = self.move_cartesian([(x, y, approach_z)])

        if not self.set_gazebo_model_pose(
            obj_name,
            x,
            y,
            placed_position[2]
        ):
            return "EXECUTION_FAILED"

        if not self.update_attached_object(
            obj_name, attached=False, world_position=placed_position
        ):
            return "EXECUTION_FAILED"
        self.attached_object = None
        self.scene['objects'][obj_name]['position'] = placed_position
        return retreat_result
