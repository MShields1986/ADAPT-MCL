ARG ROS_DISTRO=noetic
# FROM ros:${ROS_DISTRO}-ros-base AS base
FROM osrf/ros:$ROS_DISTRO-desktop-full AS base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
  && apt-get install -y \
  ros-${ROS_DISTRO}-sensor-msgs \
  ros-${ROS_DISTRO}-nav-msgs \
  ros-${ROS_DISTRO}-geometry-msgs \
  ros-${ROS_DISTRO}-visualization-msgs \
  ros-${ROS_DISTRO}-tf2-ros \
  ros-${ROS_DISTRO}-tf2-geometry-msgs \
  ros-${ROS_DISTRO}-message-filters \
  ros-${ROS_DISTRO}-map-server \
  ros-${ROS_DISTRO}-amcl \
  ros-${ROS_DISTRO}-rosbag \
  # ros-${ROS_DISTRO}-rviz \
  libeigen3-dev \
  libtbb-dev \
  python3-pip \
  python3-catkin-tools \
  && rm -rf /var/lib/apt/lists/*

# Python deps for scripts and data inspection
RUN pip3 install --no-cache-dir numpy scipy typer pyyaml matplotlib rosbags

# Build workspace
COPY src /catkin_ws/src
WORKDIR /catkin_ws

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && rosdep update \
 && rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y \
 && catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc \
 && echo 'source /catkin_ws/devel/setup.bash' >> /root/.bashrc

RUN sed -i '/exec "$@"/d' /ros_entrypoint.sh \
 && echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /ros_entrypoint.sh \
 && echo 'source /catkin_ws/devel/setup.bash' >> /ros_entrypoint.sh \
 && echo 'exec "$@"' >> /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

ENV DEBIAN_FRONTEND=dialog
