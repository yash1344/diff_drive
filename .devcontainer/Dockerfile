FROM osrf/ros:jazzy-desktop

RUN apt-get update \
    && apt-get install -y xxd build-essential \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y lsb-release gnupg \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y tree \
    && rm -rf /var/lib/apt/lists/*

# RUN apt-get update \
#     && apt-get install -y ros-jazzy-ros-ign-bridge \
#     && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y ros-jazzy-moveit ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y ros-jazzy-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-ros2-controllers-test-nodes \
    ros-jazzy-rqt-joint-trajectory-controller \
    && rm -rf /var/lib/apt/lists/*


RUN apt-get update \
    && apt-get install -y ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y ros-jazzy-gz-ros2-control \
    ros-jazzy-gz-ros2-control-demos \
    && rm -rf /var/lib/apt/lists/*
 
# RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# RUN apt-get update \
#     && apt-get install -y ignition-fortress \
#     && rm -rf /var/lib/apt/lists/*

ARG USERNAME=ros
ARG USER_UID=1001
ARG USER_GID=${USER_UID}

# creates group
# create and add user to group (-m : make home dir for user)
# create .config
# change ownership of .config to new user
RUN groupadd --gid ${USER_GID} ${USERNAME} \    
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && mkdir /home/${USERNAME}/.config \
    && chown $USER_UID:$USER_GID /home/${USERNAME}/.config


# set up Sudo
RUN apt-get update \
    && apt-get install -y sudo \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && rm -rf /var/lib/apt/lists/*

# Source ROS2 setup for the user
USER ${USERNAME}
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc

USER root
# COPY .devcontainer/entrypoint.sh /entrypoint.sh

COPY .devcontainer/bashrc /tmp/new.bashrc
RUN cat /tmp/new.bashrc >> /home/ros/.bashrc && \
    rm /tmp/new.bashrc && \
    chown ros:ros /home/ros/.bashrc

CMD [ "bash" ]