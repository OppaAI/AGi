#!/bin/bash
set -e

source /home/oppa-ai/AGi/AuRoRA/.venv/bin/activate
source /opt/ros/humble/setup.bash
source /home/oppa-ai/AGi/AuRoRA/install/setup.bash

# Your Zenoh/ROS env vars (use IP, not hostname)
export ROS_DOMAIN_ID=34
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CONFIG_URI=/home/oppa-ai/AGi/AuRoRA/custom_router_config.json5
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.0.16:7447"]'
export ZENOH_ROUTER_CHECK_ATTEMPTS=-1

exec ros2 run scs igniter
