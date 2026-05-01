#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""High-level launch file for G1 agile locomotion + bimanual IK + finger control.

See README.md for full usage instructions.
"""

from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description."""
    declared_arguments = [
        DeclareLaunchArgument(
            "input_mode",
            default_value="teleop",
            description=(
                "Input source for IK end-effector targets and locomotion command."
                " 'teleop': subscribes to xr_teleop/* topics published by the teleop app."
                " 'markers': RViz interactive markers (ik_controller_marker.py)."
            ),
            choices=["teleop", "markers"],
        ),
        # Hardware
        DeclareLaunchArgument(
            "hardware_type",
            default_value="mujoco",
            description="Hardware type: 'mujoco' for simulation, 'real' for physical G1.",
            choices=["mujoco", "real"],
        ),
        DeclareLaunchArgument(
            "enable_viewer",
            default_value="true",
            description="[MuJoCo only] Enable MuJoCo viewer GUI.",
        ),
        DeclareLaunchArgument(
            "network_interface",
            default_value="eno1",
            description="[Real hardware only] Network interface for G1 communication.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz for visualization. Defaults to true when input_mode=markers.",
        ),
        DeclareLaunchArgument(
            "use_foxglove",
            default_value="false",
            description="Start Foxglove Studio bridge.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context: LaunchContext) -> list[Any]:
    """Resolve arguments and build the node list."""
    input_mode = context.launch_configurations.get("input_mode", "teleop")
    use_markers = input_mode == "markers"
    use_teleop = input_mode == "teleop"

    bringup_share = Path(get_package_share_directory("unitree_g1_bringup"))
    controller_manager_launch = str(
        bringup_share / "launch/unitree_g1_controller_manager.launch.py"
    )

    use_rviz = "true" if use_markers else context.launch_configurations.get("use_rviz", "false")

    ik_reference_pose_topic = (
        "/xr_teleop/ee_poses" if use_teleop else "/ik_controller/reference_pose"
    )
    cmd_vel_topic = "/xr_teleop/root_twist" if use_teleop else ""

    controller_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controller_manager_launch),
        launch_arguments={
            "initial_controller_group": "agile_velocity_with_ik",
            "hardware_type": LaunchConfiguration("hardware_type"),
            "enable_viewer": LaunchConfiguration("enable_viewer"),
            "network_interface": LaunchConfiguration("network_interface"),
            "use_rviz": use_rviz,
            "use_foxglove": LaunchConfiguration("use_foxglove"),
            "ik_reference_pose_topic": ik_reference_pose_topic,
            "cmd_vel_topic": cmd_vel_topic,
        }.items(),
    )

    # Static TF publishers bridging ROS convention (x-forward, y-left, z-up) to
    # OpenXR convention (x-right, y-up, z-backward).
    _openxr_R_ros = ["-0.5", "0.5", "0.5", "0.5"]  # qx qy qz qw
    _ros_R_openxr = ["0.5", "-0.5", "-0.5", "0.5"]  # qx qy qz qw

    def _static_tf(
        parent: str,
        child: str,
        translation: list[float | str] | None = None,
        rotation: list[float | str] | None = None,
    ) -> Node:
        if translation is None:
            translation = [0, 0, 0]
        if rotation is None:
            rotation = [0, 0, 0, 1]
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"{parent}_to_{child}_tf".replace("/", "_"),
            arguments=[
                "--frame-id", parent,
                "--child-frame-id", child,
                "--x", str(translation[0]),
                "--y", str(translation[1]),
                "--z", str(translation[2]),
                "--qx", str(rotation[0]),
                "--qy", str(rotation[1]),
                "--qz", str(rotation[2]),
                "--qw", str(rotation[3]),
            ],
            output="screen",
        )

    nodes = [
        controller_manager,
        _static_tf("pelvis", "world_openxr", translation=[0, 0, -1], rotation=_ros_R_openxr),
        _static_tf("left_wrist_openxr", "left_wrist", rotation=_openxr_R_ros),
        _static_tf("right_wrist_openxr", "right_wrist", rotation=_openxr_R_ros),
        # Correction frames: children of the URDF EE frames, oriented in OpenXR
        # convention.  The IK controller looks up the purely static path
        # ee_command_frame → ee_frame (e.g. left_hand_palm_link_openxr → left_hand_palm_link).
        _static_tf("left_hand_palm_link", "left_hand_palm_link_openxr", rotation=_ros_R_openxr),
        _static_tf("right_hand_palm_link", "right_hand_palm_link_openxr", rotation=_ros_R_openxr),
    ]

    if use_markers:
        nodes.append(Node(
            package="isaac_ros_cumotion_controllers",
            executable="ik_controller_marker_node.py",
            name="ik_controller_marker",
            output="screen",
        ))
    else:
        teleop_share = Path(get_package_share_directory("isaac_ros_teleop"))
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(teleop_share / "launch/isaac_ros_teleop.launch.py")
            ),
            # Select frame where teleop app uses for reference poses
            launch_arguments={
                "world_frame": "world_openxr",
                "right_wrist_frame": "right_wrist_openxr",
                "left_wrist_frame": "left_wrist_openxr",
            }.items(),
        ))

    return nodes
