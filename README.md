# mycobot_ros

[![jaywcjlove/sb](https://jaywcjlove.github.io/sb/lang/chinese.svg)](https://www.elephantrobotics.com/docs/myCobot/3-development/4-ros&moveit/)
[![jaywcjlove/sb](https://jaywcjlove.github.io/sb/lang/english.svg)](https://www.elephantrobotics.com/docs/myCobot-en/3-development/4-ros&moveit/)

[中文文档](https://www.elephantrobotics.com/docs/myCobot/3-development/4-ros&moveit/) | [English Document](https://www.elephantrobotics.com/docs/myCobot-en/3-development/4-ros&moveit/)

**Notes**:

<!-- This is the mycobot ROS package designed by Zhang Lijun([lijun.zhang@elephantrobotics.com]()) -->

* Make sure that `Atom` is flashed into the top Atom and `Transponder` or `minirobot` is flashed into the base Basic .The tool download address: [https://github.com/elephantrobotics/myCobot/tree/main/Software](https://github.com/elephantrobotics/myCobot/tree/main/Software)
* Supported ROS versions:
   * Ubuntu 16.04 / ROS Kinetic
   * Ubuntu 18.04 / ROS Melodic
   * Ubuntu 20.04 / ROS Noetic

<!-- **If your `Atom` is 2.3 or before, or `pymycobot` is 1.\*, Please check branch [before](https://github.com/elephantrobotics/myCobotRos/tree/before)** -->

## Installation
### Option 1: Docker
There are two ways to run this project. The first is by running the project in a container, and this requires
[installing docker](https://docs.docker.com/engine/install/ubuntu/) and
[installing docker-compose](https://docs.docker.com/compose/install/). The benefit of running in the container is that you can run the project in any version of linux, as long as your kernel
is new enough.

Once docker is installed, run the following command, and the project should show up.

without NVIDIA GPU:

```
docker-compose build ros && xhost +local:root && docker-compose up ros
```

with NVIDIA GPU

```
docker-compose build nvidia-ros && xhost +local:root && docker-compose up nvidia-ros
```

This command does three things:
1) `docker-compose build ros`

   This builds the project in a container. That means nothing is installed on your host machine!
   The first time this runs, this command will take a long while. After running it once, caching
   will allow this command to run quickly.

2) `xhost +local:root`

   This command gives X the ability to display GUI's from within the docker container

3) `docker-compose up ros`

   This runs the image specified in the `docker-compose.yml`, which by default runs
   the command `roslaunch mycobot_320 mycobot_320_slider.launch` within the container.


### Option 2: Local
#### 1.1 Pre-Requriements

For using this package, the [Python api](https://github.com/elephantrobotics/pymycobot.git) library should be installed first.

```bash
pip install pymycobot --user
```

#### 1.2 Package Download and Install

Install ros package in your src folder of your Catkin workspace.

```bash
$ cd ~/catkin_ws/src
$ git clone --depth 1 https://github.com/elephantrobotics/mycobot_ros.git
$ cd ~/catkin_ws
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
$ sudo echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
```

#### 1.3 Test Python API

```bash
cd ~/catkin_ws/src/mycobot_ros
python test.py
```

## Screenshot

![Demo](./demo_img/Screenshot-1.png)

![Demo](./demo_img/Screenshot-2.png)

![Demo](./demo_img/Screenshot-3.png)

![Demo](./demo_img/Screenshot-4.png)

![Demo](./demo_img/Screenshot-5.png)

![Demo](./demo_img/320_slider.png)

![Demo](./demo_img/320_moveit.png)
