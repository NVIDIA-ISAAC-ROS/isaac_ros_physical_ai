# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

# flake8: noqa: F403,F405
import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut


def create_rectify_node(camera_name: str, side: str, args: lu.ArgumentContainer) -> lut.ComposableNode:
    """Create a single rectification node for a camera side."""
    return lut.ComposableNode(
        name=f'rectify_{camera_name}_{side}_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        parameters=[{
            'output_width': args.rectify_output_width,
            'output_height': args.rectify_output_height,
        }],
        remappings=[
            ('image_raw', f'/{camera_name}/{side}/image_raw'),
            ('camera_info', f'/{camera_name}/{side}/camera_info'),
            ('image_rect', f'/{camera_name}/{side}/image_rect'),
            ('camera_info_rect', f'/{camera_name}/{side}/camera_info_rect'),
        ],
    )


def add_rectification_nodes(args: lu.ArgumentContainer):
    """Add rectification nodes for camera images."""
    actions = []

    if not args.enable_rectification or not args.camera_name:
        return actions

    # Create rectification nodes for all cameras
    rectify_nodes = []
    for side in ['left', 'right']:
        rectify_nodes.append(create_rectify_node(args.camera_name, side, args))

    # Load all rectification nodes into the container
    # Note: Container should exist either because camera.launch.py created it
    # (when launch_container=True) or because navigation.launch.py created it
    if rectify_nodes:
        actions.append(
            lu.load_composable_nodes(
                'nova_container',
                rectify_nodes,
            ))

    return actions


def add_camera_sensors(args: lu.ArgumentContainer):
    """Add camera and sensor-related launch configurations."""
    actions = []

    # OAK camera support (RealSense driver is launched separately by gr00t_agile
    # for the policy camera, so this launch only handles OAK for localization).
    if args.camera_name.startswith('oak'):
        oak_camera_list = args.camera_name.split(',')
        for oak_camera in oak_camera_list:
            if args.enable_oak_driver:
                actions.append(
                    lu.include(
                        'isaac_ros_unitree_g1_cloud_control_bringup',
                        'launch/sensors/oak.launch.py',
                        launch_arguments={
                            'camera_name': oak_camera,
                            'oak_config_file': f'{oak_camera}/params.yaml',
                        },
                    ))

            if args.publish_urdf:
                # Reference URDF from navigator package
                actions.append(
                    lu.include(
                        'isaac_ros_unitree_g1_cloud_control_bringup',
                        'launch/sensors/publish_urdf.launch.py',
                        launch_arguments={
                            'urdf_file': f'g1_{oak_camera}.urdf',
                        },
                    ))

    # Foxglove bridge is started by gr00t_agile (use_foxglove arg), not here.

    if args.launch_container:
        actions.append(lu.component_container('nova_container'))

    # Add rectification nodes if enabled
    actions.extend(add_rectification_nodes(args))

    return actions


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()

    # Camera sensor configuration for G1
    args.add_arg('enable_oak_driver', False, cli=True)
    args.add_arg('camera_name', 'oak_cam8', cli=True)

    args.add_arg('launch_container', False, cli=True)

    args.add_arg('publish_urdf', False, cli=True)

    # Rectification options
    args.add_arg('enable_rectification', False, cli=True)
    args.add_arg('rectify_output_width', 1280, cli=True)
    args.add_arg('rectify_output_height', 720, cli=True)

    args.add_opaque_function(add_camera_sensors)

    return lut.LaunchDescription(args.get_launch_actions())
