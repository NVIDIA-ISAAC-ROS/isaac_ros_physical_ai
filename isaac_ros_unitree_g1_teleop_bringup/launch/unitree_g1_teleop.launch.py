#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


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

    nodes = [
        controller_manager,
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
            launch_arguments={
                "world_frame": "pelvis",
                "right_wrist_frame": "right_wrist",
                "left_wrist_frame": "left_wrist",
                # XR world origin is 1 m below the pelvis frame on the G1.
                "transform_translation": "[0.0, 0.0, -1.0]",
            }.items(),
        ))

    # RealSense D435 camera driver — only on real hardware.
    # MuJoCo simulation already publishes its own camera topic.
    # The driver publishes on /realsense_d435_rgb/color/image_raw,
    # matching the MuJoCo URDF sensor config so sim and real share one topic.
    hardware_type = context.launch_configurations.get("hardware_type", "mujoco")
    if hardware_type == "real":
        bringup_share_self = Path(
            get_package_share_directory("isaac_ros_unitree_g1_teleop_bringup")
        )
        realsense_config = str(bringup_share_self / "config" / "realsense_d435.yaml")
        nodes.append(
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container_mt",
                name="realsense_container",
                namespace="",
                composable_node_descriptions=[
                    ComposableNode(
                        package="realsense2_camera",
                        plugin="realsense2_camera::RealSenseNodeFactory",
                        name="realsense_d435_rgb",
                        namespace="",
                        parameters=[realsense_config],
                    ),
                ],
                output="screen",
            )
        )

    return nodes
