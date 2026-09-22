# ROS 2 Jazzy + Gazebo Harmonic, sized for this package only.
FROM osrf/ros:jazzy-desktop-full

ARG USERNAME=dev
ARG UID=1000
ARG GID=1000

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-nav2-bringup \
      ros-jazzy-navigation2 \
      ros-jazzy-slam-toolbox \
      ros-jazzy-robot-localization \
      ros-jazzy-ros-gz \
      ros-jazzy-rplidar-ros \
      ros-jazzy-joint-state-publisher \
      ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-xacro \
      python3-serial \
      python3-colcon-common-extensions \
      python3-pip \
      git vim less \
 && rm -rf /var/lib/apt/lists/*

# Match the host UID so files written in the mounted workspace stay yours,
# and so the X11 socket is readable.
RUN groupadd -g ${GID} ${USERNAME} 2>/dev/null || true \
 && useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} \
 && usermod -aG dialout,video ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /ws

RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
 && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> ~/.bashrc

CMD ["/bin/bash"]
