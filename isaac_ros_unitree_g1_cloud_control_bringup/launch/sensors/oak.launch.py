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


def launch_oak(args: lu.ArgumentContainer) -> list[lut.Node]:

    oak_config_file = lu.get_path(
        'isaac_ros_unitree_g1_cloud_control_bringup', f'params/{args.oak_config_file}')

    actions = []
    oak_node = lut.ComposableNode(
        package="depthai_ros_driver",
        plugin="depthai_ros_driver::Camera",
        name=args.camera_name,
        parameters=[
            oak_config_file,
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    actions.append(lu.load_composable_nodes(args.container_name, [oak_node]))
    actions.append(lu.log_info(f'Using OAK config file: {oak_config_file}'))
    return actions


def generate_launch_description() -> lu.LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('container_name', 'nova_container')
    args.add_arg('camera_name', 'oak')
    args.add_arg('oak_config_file', '', cli=True)
    args.add_opaque_function(launch_oak)

    return lut.LaunchDescription(args.get_launch_actions())
