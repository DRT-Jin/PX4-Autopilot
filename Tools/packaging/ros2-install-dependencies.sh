#!/usr/bin/env bash
set -euo pipefail

rm -f /etc/apt/apt.conf.d/docker-clean
apt-get update
apt-get install -y --no-install-recommends ca-certificates curl
curl -fsSL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb" \
    -o /ros2-apt-source.deb
apt-get install -y --no-install-recommends /ros2-apt-source.deb
rm /ros2-apt-source.deb
apt-get update
apt-get install -y --no-install-recommends \
    build-essential ccache cmake git ninja-build \
    python3-empy ros-dev-tools ros-jazzy-ros-base

rosdep init
rosdep update --rosdistro jazzy
if [ "$#" -gt 0 ]; then
    # ROS setup scripts are not nounset-safe.
    set +u
    source /opt/ros/jazzy/setup.bash
    rosdep install --from-paths "$1" --ignore-src --rosdistro jazzy -y
fi
