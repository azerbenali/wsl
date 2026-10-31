#!/bin/bash
# One-time setup when the container is created: dependencies, then a build.
set -e
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
apt-get update
rosdep update --rosdistro jazzy
# -r: keep going if a key cannot be installed, so one missing package does not
# block the rest; colcon reports anything that is then actually missing.
rosdep install --from-paths src --ignore-src -y -r --rosdistro jazzy \
  || echo "WARNING: some dependencies could not be installed (listed above)."
apt-get install -y ros-jazzy-teleop-twist-keyboard || echo "teleop_twist_keyboard not installed (optional)"
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
line="[ -f $PWD/install/setup.bash ] && source $PWD/install/setup.bash"
grep -qxF "$line" /etc/bash.bashrc || echo "$line" >> /etc/bash.bashrc
echo
echo "WSL is built. Run the simulation with:  ros2 launch wsl_sim warehouse.launch.py"
echo "and watch it at http://localhost:6080/vnc.html?autoconnect=1&resize=scale"
