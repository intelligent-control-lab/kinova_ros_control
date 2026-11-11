#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /opt/kinova_ws/install/setup.bash
source /opt/camera_ws/install/setup.bash
exec "$@"
