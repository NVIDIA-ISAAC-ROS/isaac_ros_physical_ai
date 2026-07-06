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

"""Launch file for Unitree G1 running GR00T with (optional) AGILE locomotion.

Wraps `unitree_g1_bringup/launch/unitree_g1_inference_graph.launch.py` with the
`gr00t_n17_apple_to_plate` controller group selected by default, and adds the
GR00T-specific pieces the generic launch file doesn't carry:

- The initial-noise publisher node (diffusion-policy seed).
- The RealSense D435 ComposableNode loaded into the inference pipeline
  container on real hardware.
"""

import os
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


GR00T_INSTALL_PACKAGE = 'isaac_ros_gr00t_unitree_g1_install'
GR00T_ASSET_SUBDIR = Path(
    'isaac_ros_assets/models/gr00t_unitree_g1/n17_apple_to_plate_0.0.2')


def _load_controller_groups() -> dict:
    bringup_share = Path(get_package_share_directory('unitree_g1_bringup'))
    return yaml.safe_load(
        (bringup_share / 'config/controller_groups.yaml').read_text())


def _workspace_from_package_share(package_share: Path) -> Path | None:
    """Infer the workspace root from a package share directory under install/."""
    for parent in package_share.parents:
        if parent.name == 'install':
            return parent.parent
    return None


def _candidate_asset_roots(package_share: Path) -> list[Path]:
    """Return possible roots that may contain isaac_ros_assets."""
    roots = []
    if os.environ.get('ISAAC_ROS_WS'):
        roots.append(Path(os.environ['ISAAC_ROS_WS']))

    workspace_from_share = _workspace_from_package_share(package_share)
    if workspace_from_share:
        roots.append(workspace_from_share)
        if workspace_from_share.name == 'ros_ws':
            roots.append(workspace_from_share.parent)

    return list(dict.fromkeys(roots))


def _resolve_group_config_path(group_config: dict[str, Any]) -> str:
    """Resolve policy config, preferring installed GR00T assets when available."""
    data_package = group_config.get('data_package', GR00T_INSTALL_PACKAGE)
    config_filename = group_config['config']

    if data_package == GR00T_INSTALL_PACKAGE:
        override_dir = os.environ.get('ISAAC_ROS_GR00T_UNITREE_G1_ASSET_DIR')
        package_share = Path(get_package_share_directory(data_package))
        candidates = []
        if override_dir:
            candidates.append(Path(override_dir) / config_filename)
        for root in _candidate_asset_roots(package_share):
            candidates.append(root / GR00T_ASSET_SUBDIR / config_filename)
        for candidate in candidates:
            if candidate.exists():
                return str(candidate)
        expected_path = package_share / 'data' / config_filename
        checked_paths = ', '.join(str(path) for path in [*candidates, expected_path])
        raise FileNotFoundError(
            f'Could not find installed GR00T config {config_filename}. '
            f'Run `ros2 run {GR00T_INSTALL_PACKAGE} install_gr00t_unitree_g1.sh --eula` '
            f'or set ISAAC_ROS_GR00T_UNITREE_G1_ASSET_DIR. Checked: {checked_paths}.')

    policy_share = Path(get_package_share_directory(data_package))
    return str(policy_share / 'data' / config_filename)


def generate_launch_description() -> LaunchDescription:
    """Declare GR00T-wrapper args + pass-through args, then defer to launch_setup."""
    declared_arguments = [
        DeclareLaunchArgument(
            'use_agile',
            default_value='true',
            description='If true, run AGILE legs alongside GR00T upper body '
            '(group `gr00t_n17_apple_to_plate`); otherwise upper body only '
            '(group `gr00t_n17_apple_to_plate_no_agile`). Ignored when '
            'initial_controller_group is set.',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'initial_controller_group',
            default_value='',
            description='Override the controller group (entry in '
            'controller_groups.yaml). Empty string means derive from '
            'use_agile.',
        ),
        # Pass-through args forwarded to unitree_g1_inference_graph.launch.py.
        DeclareLaunchArgument(
            'hardware_type',
            default_value='mujoco',
            description="Hardware type: 'mujoco' or 'real'.",
            choices=['mujoco', 'real'],
        ),
        DeclareLaunchArgument(
            'enable_viewer',
            default_value='true',
            description='[MuJoCo only] Enable the MuJoCo viewer GUI.',
        ),
        DeclareLaunchArgument(
            'mujoco_model_path',
            default_value='',
            description='[MuJoCo only] Absolute path to the MuJoCo scene XML.',
        ),
        DeclareLaunchArgument(
            'use_foxglove',
            default_value='true',
            description='Start Foxglove Studio bridge for visualization.',
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='5.0',
            description='Rate at which InputBuilderNode publishes (Hz).',
        ),
        DeclareLaunchArgument(
            'visualize_commands',
            default_value='true',
            description='Publish commanded joint positions as a ghost robot.',
        ),
        DeclareLaunchArgument(
            'network_interface',
            default_value='eno1',
            description='[Real hardware only] Network interface for G1 communication.',
        ),
        DeclareLaunchArgument(
            'gr00t_leapp_yaml_path',
            default_value='',
            description=(
                'Absolute path to your LEAPP GR00T policy YAML. ONNX files stay in '
                "the same directory as the YAML. Empty uses controller_groups "
                '`config` paths (bundled demo policy unless you customize the group).'
            ),
        ),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context: LaunchContext) -> list[Any]:
    """Resolve arguments and assemble actions at launch time."""
    override = context.perform_substitution(
        LaunchConfiguration('initial_controller_group'))
    if override:
        controller_group = override
    else:
        use_agile = context.perform_substitution(
            LaunchConfiguration('use_agile')) == 'true'
        controller_group = (
            'gr00t_n17_apple_to_plate' if use_agile
            else 'gr00t_n17_apple_to_plate_no_agile'
        )

    group_config = _load_controller_groups()[controller_group]
    leapp_yaml = context.perform_substitution(
        LaunchConfiguration('gr00t_leapp_yaml_path')).strip()
    if leapp_yaml:
        config_path = str(Path(leapp_yaml).expanduser().resolve())
    else:
        config_path = _resolve_group_config_path(group_config)

    # Base inference-graph launch — drives controllers, InputBuilder, Triton,
    # OutputBuilder, and ghost-robot visualization.
    inference_graph_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('unitree_g1_bringup'),
                'launch',
                'unitree_g1_inference_graph.launch.py',
            ])
        ),
        launch_arguments={
            'initial_controller_group': controller_group,
            'hardware_type': LaunchConfiguration('hardware_type'),
            'enable_viewer': LaunchConfiguration('enable_viewer'),
            'mujoco_model_path': LaunchConfiguration('mujoco_model_path'),
            'use_foxglove': LaunchConfiguration('use_foxglove'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'visualize_commands': LaunchConfiguration('visualize_commands'),
            'network_interface': LaunchConfiguration('network_interface'),
            'inference_config_path': config_path,
        }.items(),
    )

    # GR00T requires a diffusion seed — run the initial-noise publisher in the
    # inference_graph namespace so InputBuilder picks it up on `initial_noise`.
    # The yaml drives the noise tensor shape, which is policy-specific.
    initial_noise_publisher = GroupAction([
        PushRosNamespace('inference_graph'),
        Node(
            package='isaac_ros_deploy_converters',
            executable='initial_noise_publisher_node',
            name='initial_noise_publisher_node',
            parameters=[{
                'config_path': config_path,
                'publish_rate': LaunchConfiguration('publish_rate'),
                'output_topic': 'initial_noise',
            }],
            output='screen',
        ),
    ])

    actions: list[Any] = [inference_graph_launch, initial_noise_publisher]

    # On real hardware, load the RealSense D435 into the inference pipeline
    # container so images stay in-process with InputBuilder and Triton.
    hardware_type = context.perform_substitution(LaunchConfiguration('hardware_type'))
    if hardware_type == 'real':
        realsense_camera = ComposableNode(
            package='realsense2_camera',
            plugin='realsense2_camera::RealSenseNodeFactory',
            name='realsense_d435_rgb',
            namespace='',
            parameters=[{
                'enable_color': True,
                'enable_depth': False,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_gyro': False,
                'enable_accel': False,
                'enable_pointcloud': False,
                'rgb_camera.color_profile': '640x480x30',
            }],
        )
        actions.append(LoadComposableNodes(
            target_container='/inference_graph/inference_pipeline_container',
            composable_node_descriptions=[realsense_camera],
        ))

    return actions
