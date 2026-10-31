# ROS 2 Jazzy on Ubuntu 24.04, plus debugging/lint tools and a browser-viewable
# desktop for Gazebo and RViz. The official ros image is multi-arch, so it runs
# natively on Apple Silicon (arm64) as well as x86_64; Gazebo, RViz and RTAB-Map
# are installed by rosdep in post-create.sh.
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      gdb clang-format clang-tidy python3-flake8 \
      libegl-mesa0 libgl1-mesa-dri mesa-utils \
      xvfb fluxbox x11vnc novnc websockify xterm \
    && rm -rf /var/lib/apt/lists/*

COPY start-desktop.sh /usr/local/bin/start-desktop
RUN chmod +x /usr/local/bin/start-desktop \
    && echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc

# Gazebo and RViz render on the virtual display with Mesa; no GPU required.
ENV DISPLAY=:1 \
    LIBGL_ALWAYS_SOFTWARE=1
