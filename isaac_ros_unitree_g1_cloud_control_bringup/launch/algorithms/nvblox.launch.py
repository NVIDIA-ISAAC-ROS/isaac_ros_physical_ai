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

from typing import Any, List, Tuple

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut


def get_oak_remappings(camera_name: str) -> List[Tuple[str, str]]:
    """Get topic remappings for OAK-D cameras."""
    remappings = [
        ('camera_0/depth/image', f'/{camera_name}/stereo/image_raw'),
        ('camera_0/depth/camera_info', f'/{camera_name}/stereo/camera_info'),
        ('camera_0/color/image', f'/{camera_name}/rgb/image_raw'),
        ('camera_0/color/camera_info', f'/{camera_name}/rgb/camera_info'),
    ]
    return remappings


def get_realsense_remappings(camera_name: str) -> List[Tuple[str, str]]:
    """Get topic remappings for RealSense cameras with depth enabled."""
    remappings = [
        ('camera_0/depth/image', f'/{camera_name}/depth/image_rect_raw'),
        ('camera_0/depth/camera_info', f'/{camera_name}/depth/camera_info'),
        ('camera_0/color/image', f'/{camera_name}/color/image_raw'),
        ('camera_0/color/camera_info', f'/{camera_name}/color/camera_info'),
    ]
    return remappings


def get_nvblox_params(global_frame: str) -> List[Any]:
    """Get nvblox parameters."""
    parameters = []
    parameters.append(lu.get_path('nvblox_examples_bringup', 'config/nvblox/nvblox_base.yaml'))
    parameters.append(
        lu.get_path('nvblox_examples_bringup',
                    'config/nvblox/specializations/nvblox_dynamics.yaml'))
    # Add humanoid-specific nvblox parameters (ESDF slice heights, etc.)
    parameters.append(
        lu.get_path('isaac_ros_unitree_g1_cloud_control_bringup', 'params/nvblox_perceptor.yaml'))
    parameters.append({'num_cameras': 1})
    parameters.append({'global_frame': global_frame})
    return parameters


def add_nvblox(args: lu.ArgumentContainer) -> List[lut.Action]:
    camera_name = args.camera_name

    # Select remappings based on camera type
    if camera_name.startswith('oak'):
        remappings = get_oak_remappings(camera_name)
    elif camera_name.startswith('realsense'):
        remappings = get_realsense_remappings(camera_name)
    else:
        # Default to OAK remappings
        remappings = get_oak_remappings(camera_name)

    parameters = get_nvblox_params(args.nvblox_global_frame)

    # Add the nvblox node
    nvblox_node = lut.ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=remappings,
        parameters=parameters,
    )

    actions = []
    actions.append(lu.load_composable_nodes(args.container_name, [nvblox_node]))
    actions.append(
        lu.log_info(["Enabling nvblox for camera '", camera_name, "'"]))

    return actions


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('container_name', 'nova_container')
    args.add_arg('camera_name', 'oak_cam8')
    args.add_arg('nvblox_global_frame', 'horizontal_frame')

    args.add_opaque_function(add_nvblox)
    return lut.LaunchDescription(args.get_launch_actions())
