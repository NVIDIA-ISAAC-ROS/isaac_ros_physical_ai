#!/usr/bin/env python3

# Copyright 2026 NVIDIA CORPORATION & AFFILIATES
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

"""Unit tests for unitree_g1_gr00t_agile.launch.py argument forwarding."""

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]


def _launch_tree():
    launch_file = PACKAGE_ROOT / "launch" / "unitree_g1_gr00t_agile.launch.py"
    return ast.parse(launch_file.read_text())


def _contains_string(node, value):
    return any(
        isinstance(child, ast.Constant) and child.value == value
        for child in ast.walk(node)
    )


def _dict_keys(node):
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
        if node.func.attr == "items":
            node = node.func.value
    if not isinstance(node, ast.Dict):
        return set()
    return {
        key.value
        for key in node.keys
        if isinstance(key, ast.Constant) and isinstance(key.value, str)
    }


def _declared_launch_arguments(tree):
    names = set()
    for call in [node for node in ast.walk(tree) if isinstance(node, ast.Call)]:
        if getattr(call.func, "id", "") != "DeclareLaunchArgument":
            continue
        if not call.args:
            continue
        name = call.args[0]
        if isinstance(name, ast.Constant) and isinstance(name.value, str):
            names.add(name.value)
    return names


def test_gr00t_wrapper_declares_and_forwards_triton_cpu_models():
    tree = _launch_tree()

    assert "triton_cpu_models" in _declared_launch_arguments(tree)

    for call in [node for node in ast.walk(tree) if isinstance(node, ast.Call)]:
        if getattr(call.func, "id", "") != "IncludeLaunchDescription":
            continue
        if not _contains_string(call, "unitree_g1_inference_graph.launch.py"):
            continue
        launch_arguments = next(
            keyword.value
            for keyword in call.keywords
            if keyword.arg == "launch_arguments"
        )

        assert "triton_cpu_models" in _dict_keys(launch_arguments)
        return

    raise AssertionError("unitree_g1_inference_graph.launch.py include not found")
