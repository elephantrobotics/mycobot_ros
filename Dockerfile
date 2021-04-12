ARG UBUNTU_VERSION
FROM nvidia/opengl:1.1-glvnd-runtime-ubuntu${UBUNTU_VERSION}

# Add ROS ppa
RUN apt-get update && apt-get install -y --no-install-recommends gnupg2 lsb-release && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# DEBIAN_FRONTEND is so tzdata doesn't require user input during the apt install
ENV DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO
RUN apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-desktop-full --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# Add ROS env vars to the bashrc
ENV BASH_ENV="/root/launch.sh"
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/bin/bash", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> $BASH_ENV

# Install build dependencies
RUN apt-get update && \
    apt-get install -y \
        # ROS Build dependencies
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
        build-essential \
        # Project-specific build dependencies
        python-pip \
        ros-${ROS_DISTRO}-serial \
        ros-${ROS_DISTRO}-joint-state-publisher-gui && \
    rm -rf /var/lib/apt/lists/*

# Install python dependencies
ARG PYMYCOBOT_VERSION
RUN pip install "pymycobot $PYMYCOBOT_VERSION" --user

# Build the project
WORKDIR /catkin_ws/src
ADD . myCobotROS
WORKDIR /catkin_ws
RUN catkin_make

# Let ROS know about the projects launch options
RUN echo "source /catkin_ws/devel/setup.bash" >> $BASH_ENV
