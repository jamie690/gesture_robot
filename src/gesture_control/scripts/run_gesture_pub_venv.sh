#!/usr/bin/env bash
set -e

source ~/mp_venv/bin/activate
source /opt/ros/humble/setup.bash
source ~/gesture_ws/install/setup.bash

python3 -m gesture_control.gesture_pub
