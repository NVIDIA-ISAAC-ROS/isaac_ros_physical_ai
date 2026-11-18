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
import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut


def _create_image_decompressor_node(
    camera_name: str, identifier: str, rectified: bool
) -> lut.ComposableNode:
    node_name = f'image_decompressor_{camera_name}_{identifier}'

    if rectified:
        input_topic = f'/{camera_name}/{identifier}/image_rect/compressed'
        output_topic = f'/{camera_name}/{identifier}/image_rect'
    else:
        input_topic = f'/{camera_name}/{identifier}/image_raw/compressed'
        output_topic = f'/{camera_name}/{identifier}/image_raw'

    return lut.ComposableNode(
        name=node_name,
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageDecompressorNode',
        parameters=[{
            'input_topic': input_topic,
            'output_topic': output_topic,
        }],
    )


def _add_image_decompression(args: lu.ArgumentContainer) -> list[lut.Action]:
    camera_names = args.enabled_stereo_cameras.split(',')
    actions = []

    decompressor_nodes = []
    for camera_name in camera_names:
        left = _create_image_decompressor_node(
            camera_name, 'left', args.decompress_rectified_images)
        right = _create_image_decompressor_node(
            camera_name, 'right', args.decompress_rectified_images)
        decompressor_nodes.extend([left, right])

    actions.append(lu.load_composable_nodes(args.container_name, decompressor_nodes))
    actions.append(lu.log_info([
        "Enabling image decompression for cameras '",
        args.enabled_stereo_cameras,
        "'"
    ]))

    return actions


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('container_name', 'nova_container')
    args.add_arg('enabled_stereo_cameras')
    args.add_arg('decompress_rectified_images', False, cli=True)

    args.add_opaque_function(_add_image_decompression)

    return lut.LaunchDescription(args.get_launch_actions())
