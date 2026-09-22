# ROS 2 Jazzy + Gazebo Harmonic, sized for this package only.
FROM osrf/ros:jazzy-desktop-full

ARG USERNAME=dev
ARG HOST_UID=1000
ARG HOST_GID=1000

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
# and so the X11 socket is readable. Ubuntu Noble already ships a `ubuntu`
# user at UID 1000, so evict whoever holds the id before claiming it.
RUN if getent passwd ${HOST_UID} >/dev/null; then \
        userdel -r "$(getent passwd ${HOST_UID} | cut -d: -f1)" 2>/dev/null || true; \
    fi \
 && if getent group ${HOST_GID} >/dev/null; then \
        groupdel "$(getent group ${HOST_GID} | cut -d: -f1)" 2>/dev/null || true; \
    fi \
 && groupadd -g ${HOST_GID} ${USERNAME} \
 && useradd -m -u ${HOST_UID} -g ${HOST_GID} -s /bin/bash ${USERNAME} \
 && usermod -aG dialout,video ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

# Create the workspace as the dev user before switching to it. WORKDIR would
# create it root-owned, and the build/install named volumes inherit whatever
# ownership these directories have when the volume is first populated.
RUN mkdir -p /ws/src /ws/build /ws/install /ws/log \
 && chown -R ${HOST_UID}:${HOST_GID} /ws

USER ${USERNAME}
WORKDIR /ws

RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
 && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> ~/.bashrc

CMD ["/bin/bash"]
