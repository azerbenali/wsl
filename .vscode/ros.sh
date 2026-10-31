#!/bin/bash
# Runs a command with ROS 2 Jazzy sourced, plus this workspace's overlay once it
# has been built. Used by tasks.json: .vscode/ros.sh <command> [args...]
source /opt/ros/jazzy/setup.bash
if [ -f "$(dirname "$0")/../install/setup.bash" ]; then
  source "$(dirname "$0")/../install/setup.bash"
fi
exec "$@"
