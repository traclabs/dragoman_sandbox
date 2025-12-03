#!/usr/bin/env python3
"""
CLR Trajectory Router

CLR-specific utilities for routing and sending trajectories to different
subsystems (arm, rail, lift, gripper). This module contains configuration
and logic specific to the CLR (Chonkur + Lift + Rail) robot system.

For other robot systems, create a similar module (e.g., chonkur_trajectory_router.py)
with robot-specific constants and routing logic.
"""

from typing import Dict, List

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import ParallelGripperCommand


# *************************
# Constants
# *************************
# Default trajectory durations (seconds)
DEFAULT_TRAJECTORY_DURATION = 4
CLR_TRAJECTORY_DURATION = 6

# Joint name constants (from SRDF group definitions)
ARM_JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
]
RAIL_JOINT_NAMES = ["vention_rail_base_to_carriage"]
LIFT_JOINT_NAMES = ["ewellix_lift_lower_to_higher"]

# Gripper constants
DEFAULT_GRIPPER_EFFORT = 100.0  # Newtons


# *************************
# Trajectory Helpers
# *************************
def create_trajectory_point(positions: List[float], duration_sec: int) -> JointTrajectoryPoint:
    """Helper to create a trajectory point with given positions and duration"""
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = Duration(sec=duration_sec)
    return point


def publish_subsystem_trajectory(joint_names: List[str], positions: List[float],
                                  publisher, duration_sec: int):
    """Helper to publish trajectory for a subsystem"""
    traj = JointTrajectory()
    traj.joint_names = joint_names
    traj.points.append(create_trajectory_point(positions, duration_sec))
    publisher.publish(traj)


# *************************
# Trajectory Senders
# *************************
def send_trajectory(pose_config: Dict, publisher, logger, duration_sec: int = DEFAULT_TRAJECTORY_DURATION):
    """Send a joint trajectory to a single publisher"""
    traj = JointTrajectory()
    traj.joint_names = pose_config["joints"]
    traj.points.append(create_trajectory_point(pose_config["values"], duration_sec))

    publisher.publish(traj)
    logger.info(f"Sent trajectory: {len(pose_config['joints'])} joints")


def send_clr_trajectory(pose_config: Dict, rail_pub, lift_pub, arm_pub, logger,
                        duration_sec: int = CLR_TRAJECTORY_DURATION):
    """Send coordinated CLR trajectory by splitting joints across subsystems"""
    joints = pose_config["joints"]
    values = pose_config["values"]

    # Separate joints by subsystem using exact joint names from SRDF groups
    rail_joints = [(j, v) for j, v in zip(joints, values) if j in RAIL_JOINT_NAMES]
    lift_joints = [(j, v) for j, v in zip(joints, values) if j in LIFT_JOINT_NAMES]
    arm_joints = [(j, v) for j, v in zip(joints, values) if j in ARM_JOINT_NAMES]

    # Send to each subsystem
    if rail_joints:
        publish_subsystem_trajectory(
            [j for j, _ in rail_joints], [v for _, v in rail_joints],
            rail_pub, duration_sec
        )

    if lift_joints:
        publish_subsystem_trajectory(
            [j for j, _ in lift_joints], [v for _, v in lift_joints],
            lift_pub, duration_sec
        )

    if arm_joints:
        publish_subsystem_trajectory(
            [j for j, _ in arm_joints], [v for _, v in arm_joints],
            arm_pub, duration_sec
        )

    logger.info(f"Sent coordinated CLR: rail={len(rail_joints)}, lift={len(lift_joints)}, arm={len(arm_joints)} joints")


def send_gripper_command(pose_config: Dict, action_client, logger):
    """Send gripper command via action client

    The ParallelGripperCommand uses a JointState message with:
    - name: list with single joint name (finger_1_joint)
    - position: list with single position value (meters)
    - effort: list with single max effort value (optional)

    The parallel gripper controller only controls finger_1_joint and expects
    exactly 1 position value. The SRDF may define both finger joints, but
    we only send the first one (finger_1_joint).
    """
    # Extract only the first joint (finger_1_joint) and its value
    joint_name = pose_config["joints"][0]  # Should be "finger_1_joint"
    joint_value = pose_config["values"][0]  # Position in meters (0.0 to 0.03)

    # Create goal with JointState command (single joint)
    goal_msg = ParallelGripperCommand.Goal()
    goal_msg.command.name = [joint_name]
    goal_msg.command.position = [joint_value]
    goal_msg.command.effort = [DEFAULT_GRIPPER_EFFORT]

    # Send goal asynchronously
    logger.info(f"Sending gripper command: joint={joint_name}, position={joint_value:.4f}m")
    action_client.send_goal_async(goal_msg)
    logger.info("Gripper command sent to action server")
