ARG BASE_IMAGE

FROM ${BASE_IMAGE}

# For this build, we pull the entire ros image, and then merge the filesystem
# with the nvidia/opengl image, so that displaying to the screen on GPU
# (or without GPU) works through docker.
# I was unable to use the ROS_DISTRO variable here due to this issue:
# https://github.com/docker/for-mac/issues/2155
COPY --from=osrf/ros:noetic-desktop-full / /

# Add ROS env vars to the bashrc
ENV BASH_ENV="/root/launch.sh"
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/bin/bash", "-c"]
ARG ROS_DISTRO
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> $BASH_ENV

# Install build dependencies
RUN apt update && \
    apt install -y \
        # ROS Build dependencies
        python3 curl \
        build-essential

RUN curl -kL https://bootstrap.pypa.io/get-pip.py | python3

# Install python dependencies
ARG PYMYCOBOT_VERSION
RUN pip install "pymycobot $PYMYCOBOT_VERSION" --user

# Build the project
WORKDIR /catkin_ws/src
ADD . myCobotROS
WORKDIR /catkin_ws
RUN rosdep update && rosdep install -i -y --from-paths src && catkin_make

# Let ROS know about the projects launch options
RUN echo "source /catkin_ws/devel/setup.bash" >> $BASH_ENV
