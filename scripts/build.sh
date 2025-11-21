#!/usr/bin/env bash

# Usage: ./scripts/build.sh [DOCKER_CI_DIR]

set -e
set -o pipefail

DOCKER_CI_DIR=$1

$DOCKER_CI_DIR/scripts/build.sh sonia_common_ros2

cd rs485_port_manager

source /build/sonia_common_ros2/INSTALL_BASE/setup.sh

colcon build --cmake-force-configure --install INSTALL_BASE
