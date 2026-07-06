#!/usr/bin/env bash
#####################################################################################
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
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
#####################################################################################

# Start the NVIDIA CUDA MPS (Multi-Process Service) daemon.
#
# MPS allows multiple CUDA processes to share the GPU with guaranteed thread
# partitioning.  This is required when running a lightweight policy (e.g. AGILE
# at 200 Hz) alongside a heavy model (e.g. GR00T) so that the heavy model does
# not starve the low-latency one.
#
# Usage:
#   sudo ./setup_mps.sh          # start MPS
#   sudo ./setup_mps.sh --stop   # stop MPS
#
# This script is safe to run multiple times — it skips if MPS is already active.

set -euo pipefail

export CUDA_MPS_PIPE_DIRECTORY=/tmp/nvidia-mps
export CUDA_MPS_LOG_DIRECTORY=/tmp/nvidia-log

if [[ ${1:-} == "--stop" ]]; then
  echo "Stopping MPS daemon..."
  echo quit | nvidia-cuda-mps-control 2>/dev/null || true
  echo "MPS stopped."
  exit 0
fi

# Check if MPS is already running.
if echo get_server_list | nvidia-cuda-mps-control 2>/dev/null; then
  echo "MPS is already running."
  exit 0
fi

mkdir -p "$CUDA_MPS_PIPE_DIRECTORY" "$CUDA_MPS_LOG_DIRECTORY"

echo "Starting MPS daemon..."
nvidia-cuda-mps-control -d
echo "MPS daemon started."
echo "  Pipe directory: $CUDA_MPS_PIPE_DIRECTORY"
echo "  Log directory:  $CUDA_MPS_LOG_DIRECTORY"
