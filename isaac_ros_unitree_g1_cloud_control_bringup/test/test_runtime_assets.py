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

"""Static checks for cloud-control bringup runtime assets."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_runtime_asset_directories_are_present():
    """The installed share tree must include all launch-time asset directories."""
    for relative_path in [
        "foxglove_layouts",
        "launch",
        "params",
        "urdf",
    ]:
        assert (PACKAGE_ROOT / relative_path).is_dir(), f"Missing {relative_path}"


def test_top_level_launch_references_installed_include_launch_files():
    """Included launch files referenced by the top-level bringup must be packaged."""
    launch_file = PACKAGE_ROOT / "launch" / "unitree_g1_cloud_control.launch.py"
    assert launch_file.is_file(), "Missing launch/unitree_g1_cloud_control.launch.py"
    launch_text = launch_file.read_text(encoding="utf-8")

    for relative_path in [
        "launch/sensors/camera.launch.py",
        "launch/include/perception_include.launch.py",
    ]:
        assert (PACKAGE_ROOT / relative_path).is_file(), f"Missing {relative_path}"
        expected_reference = f"'/{relative_path}'"
        assert expected_reference in launch_text, f"Missing include reference {expected_reference}"
