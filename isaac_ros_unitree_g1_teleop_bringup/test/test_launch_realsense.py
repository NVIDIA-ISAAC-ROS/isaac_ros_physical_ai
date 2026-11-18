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

"""Tests that the teleop launch file includes the RealSense node conditionally."""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import OpaqueFunction
from launch_ros.actions import ComposableNodeContainer


def _load_launch_module():
    """Load the launch file as a Python module from the installed share directory."""
    share = Path(get_package_share_directory("isaac_ros_unitree_g1_teleop_bringup"))
    launch_file = share / "launch" / "unitree_g1_teleop.launch.py"
    spec = importlib.util.spec_from_file_location("unitree_g1_teleop_launch", launch_file)
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load launch file: {launch_file}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _resolve_launch(hardware_type: str) -> list:
    """Load the launch module and resolve the OpaqueFunction with given hardware_type."""
    launch_mod = _load_launch_module()
    ld = launch_mod.generate_launch_description()

    # Find the OpaqueFunction in the launch description entities.
    opaque_fns = [
        e for e in ld.entities
        if isinstance(e, OpaqueFunction)
    ]
    assert len(opaque_fns) == 1, f"Expected 1 OpaqueFunction, found {len(opaque_fns)}"

    # Build a context with the desired hardware_type.
    context = LaunchContext()
    context.launch_configurations['hardware_type'] = hardware_type
    context.launch_configurations['input_mode'] = 'teleop'
    context.launch_configurations['enable_viewer'] = 'true'
    context.launch_configurations['network_interface'] = 'eno1'
    context.launch_configurations['use_rviz'] = 'false'
    context.launch_configurations['use_foxglove'] = 'false'

    # Execute the OpaqueFunction to get the resolved actions.
    return opaque_fns[0].execute(context)


def _composable_containers(actions: list) -> list[ComposableNodeContainer]:
    """Return all ``ComposableNodeContainer`` actions in the given list.

    Uses a structural check rather than introspecting each container's
    private ``_composable_node_descriptions`` list, which has no public
    accessor and varies across ``launch_ros`` versions. The launch file
    under test creates a single container only when ``hardware_type='real'``,
    so presence/absence is a reliable signal.
    """
    return [a for a in actions if isinstance(a, ComposableNodeContainer)]


def test_realsense_launched_on_real_hardware():
    """When hardware_type=real, a ComposableNodeContainer must be present."""
    actions = _resolve_launch('real')
    containers = _composable_containers(actions)
    assert len(containers) == 1, (
        f"Expected exactly 1 ComposableNodeContainer for RealSense on real "
        f"hardware, found {len(containers)}"
    )


def test_realsense_not_launched_on_mujoco():
    """When hardware_type=mujoco, no ComposableNodeContainer should be present."""
    actions = _resolve_launch('mujoco')
    containers = _composable_containers(actions)
    assert len(containers) == 0, (
        f"Did not expect any ComposableNodeContainer on mujoco hardware, "
        f"found {len(containers)}"
    )
