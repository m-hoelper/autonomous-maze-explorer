# Checks that the package builds on a plain ROS 2 Jazzy image.
# The robot vendor packages are not included, so only nodes without those imports can start here.
FROM ros:jazzy
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-numpy python3-opencv python3-scipy \
    python3-networkx python3-transforms3d \
    ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-apriltag-msgs \
    ros-jazzy-cv-bridge ros-jazzy-tf-transformations ros-jazzy-tf2-geometry-msgs \
 && pip3 install --break-system-packages numba \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY ros2_ws/src/searchforce src/searchforce
RUN . /opt/ros/jazzy/setup.sh && colcon build --packages-select searchforce
