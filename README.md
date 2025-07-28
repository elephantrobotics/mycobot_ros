# mycobot_ros
<!-- ALL-CONTRIBUTORS-BADGE:START - Do not remove or modify this section -->
[![All Contributors](https://img.shields.io/badge/all_contributors-17-orange.svg?style=flat-square)](#contributors-)
<!-- ALL-CONTRIBUTORS-BADGE:END -->

[![jaywcjlove/sb](https://jaywcjlove.github.io/sb/lang/chinese.svg)](https://docs.elephantrobotics.com/docs/gitbook/12-ApplicationBaseROS/)
[![jaywcjlove/sb](https://jaywcjlove.github.io/sb/lang/english.svg)](https://docs.elephantrobotics.com/docs/gitbook-en/12-ApplicationBaseROS/)

[中文文档](https://docs.elephantrobotics.com/docs/gitbook/12-ApplicationBaseROS/) | [English Documentation](https://docs.elephantrobotics.com/docs/gitbook-en/12-ApplicationBaseROS/)

Control or simulate myCobot series robots in ROS.

![Demo](./demo_img/Screenshot-1.png)

**Notes**:

* Make sure that `Atom` is flashed into the top Atom and `Transponder` or `minirobot` is flashed into the base Basic .The tool download address: [https://github.com/elephantrobotics/myCobot/tree/main/Software](https://github.com/elephantrobotics/myCobot/tree/main/Software)
* Supported ROS versions:
   * Ubuntu 16.04 / ROS Kinetic
   * Ubuntu 18.04 / ROS Melodic
   * Ubuntu 20.04 / ROS Noetic

* The urdf model of a single gripper in this article is applicable to all machines that support this accessory.
<!-- **If your `Atom` is 2.3 or before, or `pymycobot` is 1.\*, Please check branch [before](https://github.com/elephantrobotics/myCobotRos/tree/before)** -->

## Installation
### Option 1: Docker
There are two ways to run this project. The first is by running the project in a container, and this requires
[installing docker](https://docs.docker.com/engine/install/ubuntu/) and
[installing docker-compose](https://docs.docker.com/compose/install/). The benefit of running in the container is that you can run the project in any version of linux, as long as your kernel
is new enough.

Once docker is installed, run the following command, and the project should show up.

#### Without NVIDIA GPU:

**ROS Noetic**:

```
docker-compose build ros-noetic && xhost +local:root && docker-compose up ros-noetic
```

**ROS Melodic**:

```
docker-compose build ros && xhost +local:root && docker-compose up ros
```

#### With NVIDIA GPU

**ROS Noetic**:

```
docker-compose build nvidia-ros-noetic && xhost +local:root && docker-compose up nvidia-ros-noetic
```

**ROS Melodic**:

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

To run other tutorials, set the LAUNCH_TARGET environment variable. For example, to the run the MoveIt tutorial run:

```
export LAUNCH_TARGET=mycobot_320_moveit mycobot320_moveit.launch
docker-compose up ros
```

### Option 2: Local
#### 2.1 Pre-Requriements

For using this package, the [Python api](https://github.com/elephantrobotics/pymycobot.git) library should be installed first.

```bash
pip install pymycobot --user
```

#### 2.2 Package Download and Install

Install ros package in your src folder of your Catkin workspace.

```bash
$ cd ~/catkin_ws/src
$ git clone --depth 1 https://github.com/elephantrobotics/mycobot_ros.git
$ cd ~/catkin_ws
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
$ sudo echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
```

#### 2.3 Test Python API

```bash
cd ~/catkin_ws/src/mycobot_ros
python test.py
```

## Where to get help

There is documentation in the [ElephantRobotics Docs](https://docs.elephantrobotics.com/docs/gitbook-en/). Please check the ROS chapter.

## Important Links & Docs

* [User Guide](https://docs.elephantrobotics.com/docs/gitbook-en/12-ApplicationBaseROS/)

## Contributing

Contributions are always welcome!

See [CONTRIBUTING.md](CONTRIBUTING.md) for ways to get started.

Please adhere to this project's [code of conduct](CODE_OF_CONDUCT.md).

## Screenshots

![Demo](./demo_img/Screenshot-2.png)

![Demo](./demo_img/Screenshot-3.png)

![Demo](./demo_img/Screenshot-4.png)

![Demo](./demo_img/Screenshot-5.png)

![Demo](./demo_img/320_slider.png)

![Demo](./demo_img/320_moveit.png)

## URDF Model Graph

[mycobot 280 m5](./mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5.urdf)

![280 m5](./demo_img/280m5/280_m5.png)

[mycobot 280 m5 gripper](./mycobot_description/urdf/mycobot_280_m5/mycobot_280m5_with_gripper_parallel.urdf)

![280 m5 gripper](./demo_img/280m5/280_m5_gripper.png)

[mycobot 280 m5 pump](./mycobot_description/urdf/mycobot_280_m5/mycobot_280m5_with_pump.urdf)

![280 m5 pump](./demo_img/280m5/280_m5_pump.png)

[mycobot 280 m5 camera flange](./mycobot_description/urdf/mycobot_280_m5/mycobot_280m5_with_camera_flange.urdf)

![280 m5 camera flange](./demo_img/280m5/280_m5_camera_flange.png)

[mycobot 280 m5 camera flange & pump](./mycobot_description/urdf/mycobot_280_m5/mycobot_280m5_with_camera_flange_pump.urdf)

![280 m5 camera flange & pump](./demo_img/280m5/280_m5_camera_flange_pump.png)

[mycobot 280 pi](./mycobot_description/urdf/mycobot_280_pi/mycobot_280_pi.urdf)

![280 pi](./demo_img/280pi/280_pi.png)

[mycobot 280 pi pump](./mycobot_description/urdf/mycobot_280_pi/mycobot_280pi_with_pump.urdf)

![280 pi pump](./demo_img/280pi/280_pi_pump.png)

[mycobot 280 pi camera flange](./mycobot_description/urdf/mycobot_280_pi/mycobot_280pi_with_camera_flange.urdf)

![280 pi camera flange](./demo_img/280pi/280_pi_camera_flange.png)

[mycobot 280 pi camera flange & pump](./mycobot_description/urdf/mycobot_280_pi/mycobot_280pi_with_camera_flange_pump.urdf)

![280 pi camera flange pump](./demo_img/280pi/280_pi_camera_flange_pump.png)

[mycobot 280 JetsonNano](./mycobot_description/urdf/mycobot_280_jn/mycobot_280_jn.urdf)

![280 jn](./demo_img/280jn/280jn.png)

[mycobot 280 JetsonNano Adaptive gripper](./mycobot_description/urdf/mycobot_280_jn/mycobot_280_jn_adaptive_gripper_parallel.urdf)

![280 jn](./demo_img/280jn/mycobot_280jn_adaptive_gripper.png)

[mycobot 280 JetsonNano Parallel gripper](./mycobot_description/urdf/mycobot_280_jn/mycobot_280_jn_parallel_gripper.urdf)

![280 jn](./demo_img/280jn/mycobot_280jn_parallel_gripper.png)

[mycobot 280 Arduino](./mycobot_description/urdf/mycobot_280_arduino/mycobot_280_arduino.urdf)

![280 ar](./demo_img/280arduino/280_arduino.png)

[mechArm 270 m5](./mycobot_description/urdf/mecharm_270_m5/mecharm_270_m5.urdf)

![270 m5](./demo_img/270m5/270m5.png)

[mechArm 270 pi](./mycobot_description/urdf/mecharm_270_pi/mecharm_270_pi.urdf)

![270 pi](./demo_img/270pi/270pi.png)

[myPalletizer 260 m5](./mycobot_description/urdf/mypalletizer_260_m5/mypalletizer_260_m5.urdf)

![260 m5](./demo_img/260m5/260m5.png)

[myPalletizer 260 pi](./mycobot_description/urdf/mypalletizer_260_pi/mypalletizer_260_pi.urdf)

![260 pi](./demo_img/260pi/260pi.png)

[mycobot 320 m5 2020](./mycobot_description/urdf/mycobot_320_m5_2020/mycobot_pro_320_m5_2020.urdf)

![320 m5 2020](./demo_img/320m5_2020/320m5_2020.png)

[mycobot 320 m5 2022](./mycobot_description/urdf/mycobot_320_m5_2022/new_mycobot_pro_320_m5_2022.urdf)

![320 m5 2022](./demo_img/320m5_2022/320m5_2022.png)

[mycobot 320 m5 2022 gripper](./mycobot_description/urdf/mycobot_320_m5_2022/new_mycobot_pro_320_m5_2022_gripper.urdf)

![320 m5 2022 gripper](./demo_img/320m5_2022/320m5_gripper_2022.png)

[mycobot 320 m5 2022 force control gripper](./mycobot_description/urdf/mycobot_320_m5_2022/pro_320_m5_2022_force_gripper.urdf)

![320 m5 2022 force control gripper](./demo_img/320m5_2022/320m5_force_gripper_2022.png)

[mycobot 320 pi 2022](./mycobot_description/urdf/mycobot_320_pi_2022/new_mycobot_pro_320_pi_2022.urdf)

![320 pi 2022](./demo_img/320pi_2022/320pi_2022.png)

[mycobot 320 pi 2022 gripper](./mycobot_description/urdf/mycobot_320_pi_2022/new_mycobot_pro_320_pi_2022_gripper.urdf)

![320 m5 2022 gripper](./demo_img/320m5_2022/320m5_gripper_2022.png)

[mycobot 320 pi 2022 force control gripper](./mycobot_description/urdf/mycobot_320_pi_2022/pro_320_pi_2022_force_gripper.urdf)

![320 m5 2022 force control gripper](./demo_img/320m5_2022/320m5_force_gripper_2022.png)

[ultraArm P340](./mycobot_description/urdf/ultraArm_p340/ultraArm_p340.urdf)

![p340](./demo_img/ultraArm_p340/ultraArmp340.png)

[mycobot 280 pi aikit](./mycobot_description/urdf/mycobot_280_pi/mycobot_280pi_with_vision_v2.urdf)

![aikit 280pi](./demo_img/aikit/aikit_280pi.png)

[mycobot 320 pi aikit](./mycobot_description/urdf/mycobot_320_pi_2022/new_mycobot_vision_v2_2022.urdf)

![aikit 320pi](./demo_img/aikit/aikit_320pi.png)

[mybuddy](./mycobot_description/urdf/mybuddy/mybuddy.urdf)

![mybuddy](./demo_img/mybuddy/mybuddy.png)

[mycobot pro 600](./mycobot_description/urdf/mycobot_pro_600/mycobot_pro_600.urdf)

![pro600](./demo_img/pro600/pro600.png)

[myArm 300 Pi](./mycobot_description/urdf/myarm_300_pi/myarm_300_pi.urdf)

![myarm](./demo_img/myarm300/myarm300.png)

[mycobot pro 630](./mycobot_description/urdf/mycobot_pro_630/mycobot_pro_630.urdf)

![pro630](./demo_img/pro630/pro630.png)

## Single Gripper URDF Model Graph

>> **This urdf model is applicable to all machines that support this accessory**

[mycobot Adaptive gripper](./mycobot_description/urdf/adaptive_gripper/mycobot_adaptive_gripper.urdf)

![280 jn](./demo_img/single_gripper/mycobot_adaptive_gripper.png)

[mycobot Parallel gripper](./mycobot_description/urdf/parallel_gripper/mycobot_parallel_gripper.urdf)

![280 jn](./demo_img/single_gripper/mycobot_parallel_gripper.png)

[mycobot Pro Adaptive gripper](./mycobot_description/urdf/pro_adaptive_gripper/mycobot_pro_adaptive_gripper.urdf) - old version

![280 jn](./demo_img/single_gripper/mycobot_pro_adaptive_gripper.png)

[mycobot Pro Adaptive gripper](./mycobot_description/urdf/pro_adaptive_gripper/new_gripper/mycobot_pro_adaptive_gripper_new.urdf) (new version - Added more details)

![320 gripper](./demo_img/single_gripper/mycobot_pro_adaptive_gripper.png)

## MyCobot_280_m5-Gazebo User Guide
1. Slider Control
The control of the robot arm model's pose in Gazebo through the sliders of joint_state_publisher_gui has now been achieved. Moreover, the pose of the robot arm model in Gazebo and the real robot arm can be controlled simultaneously via the sliders.
After confirming that the real robot arm is connected to the computer, check the port to which the robot arm is connected: 

```bash
ls /dev/tty*
/dev/ttyACM0 or /dev/ttyUSB0
```

The following output results are obtained: 

```bash
/dev/tty    /dev/tty26  /dev/tty44  /dev/tty62      /dev/ttyS20
/dev/tty0   /dev/tty27  /dev/tty45  /dev/tty63      /dev/ttyS21
/dev/tty1   /dev/tty28  /dev/tty46  /dev/tty7       /dev/ttyS22
/dev/tty10  /dev/tty29  /dev/tty47  /dev/tty8       /dev/ttyS23
/dev/tty11  /dev/tty3   /dev/tty48  /dev/tty9       /dev/ttyS24
/dev/tty12  /dev/tty30  /dev/tty49  /dev/ttyACM0   (/dev/ttyUSB0)
/dev/tty13  /dev/tty31  /dev/tty5   /dev/ttyprintk  /dev/ttyS26
/dev/tty14  /dev/tty32  /dev/tty50  /dev/ttyS0      /dev/ttyS27
/dev/tty15  /dev/tty33  /dev/tty51  /dev/ttyS1      /dev/ttyS28
/dev/tty16  /dev/tty34  /dev/tty52  /dev/ttyS10     /dev/ttyS29
/dev/tty17  /dev/tty35  /dev/tty53  /dev/ttyS11     /dev/ttyS3
/dev/tty18  /dev/tty36  /dev/tty54  /dev/ttyS12     /dev/ttyS30
/dev/tty19  /dev/tty37  /dev/tty55  /dev/ttyS13     /dev/ttyS31
/dev/tty2   /dev/tty38  /dev/tty56  /dev/ttyS14     /dev/ttyS4
/dev/tty20  /dev/tty39  /dev/tty57  /dev/ttyS15     /dev/ttyS5
/dev/tty21  /dev/tty4   /dev/tty58  /dev/ttyS16     /dev/ttyS6
/dev/tty22  /dev/tty40  /dev/tty59  /dev/ttyS17     /dev/ttyS7
/dev/tty23  /dev/tty41  /dev/tty6   /dev/ttyS18     /dev/ttyS8
/dev/tty24  /dev/tty42  /dev/tty60  /dev/ttyS19     /dev/ttyS9
/dev/tty25  /dev/tty43  /dev/tty61  /dev/ttyS2
```

Open communication and Chmod

```bash
sudo chmod -R 777 /dev/ttyACM0  or sudo chmod -r 777 /dev/ttyUSB0
sudo chmod -R 777 280m5_gripper_gazebo/280m5_gazebo_gripper/scripts/follow_display_gazebo.py
sudo chmod -R 777 280m5_gripper_gazebo/280m5_gazebo_gripper/scripts/slider_control_gazebo.py
sudo chmod -R 777 280m5_gripper_gazebo/280m5_gazebo_gripper/scripts/teleop_keyboard_gazebo.py
roscore
```

After confirming the port, open a terminal and enter the following command. Note that you should replace "port" with the value you found in the previous step. 

```bash
source devel/setup.bash
roslaunch 280m5_gazebo_gripper slider.launch _port:=/dev/ttyACM0 _baud:=115200
```


Then open another terminal and enter the following command: 

```bash
source devel/setup.bash
rosrun 280m5_gazebo_gripper slider_control_gazebo.py _port:=/dev/ttyACM0 _baud:=115200
```


Also remember to modify the port number to the one queried in the previous step. If the operation is successful, you will see the following terminal prompt: 

```bash
('/dev/ttyACM0', 115200)
spin ...
```


At this point, you can control the poses of both the mechanical arm model in Gazebo or the real mechanical arm simultaneously by manipulating the sliders in the joint_state_publisher_gui. 

2. Gazebo Model Following
The following command can be used to make the model in Gazebo change its pose in accordance with the movement of the actual robotic arm. First, run the launch file: 

```bash
source devel/setup.bash
roslaunch 280m5_gazebo_gripper follower.launch _port:=/dev/ttyACM0
```


If the program runs successfully, the Gazebo interface will successfully load the robotic arm model, and all joints of the robotic arm model will be in the original pose, that is, [0, 0, 0, 0, 0, 0]. After that, we open the second terminal and run: 

```bash
source devel/setup.bash
rosrun 280m5_gazebo_gripper follow_display_gazebo.py _port:=/dev/ttyACM0 _baud:=115200
```


Now when we control the pose of the actual robotic arm, we can see that the robotic arm in Gazebo will also move to the same pose together. 

3. Keyboard Control
We can also use keyboard input to simultaneously control the pose of the robotic arm model in Gazebo and the actual robotic arm. First, open a terminal and enter: 

```bash
source devel/setup.bash
roslaunch 280m5_gazebo_gripper teleop_keyboard.launch _port:=/dev/ttyACM0 _baud:=115200
```


As in the previous part, we will see the robotic arm model loaded into Gazebo, and all joints are at their initial poses. Then we open another terminal and enter: 

```bash
source devel/setup.bash
rosrun 280m5_gazebo_gripper teleop_keyboard_gazebo.py _port:=/dev/ttyACM0 _baud:=115200
```


If the operation is successful, we will see the following output information in the terminal: 

```shell
Mycobot_280_m5_gripper Teleop Keyboard Controller
---------------------------
Movimg options (control the angle of each joint):
w: joint2_to_joint1++   s: joint2_to_joint1--
e: joint3_to_joint2++   d: joint3_to_joint2--
r: joint4_to_joint3++   f: joint4_to_joint3--
t: joint5_to_joint4++   g: joint5_to_joint4--
y: joint6_to_joint5++   h: joint6_to_joint5--
u: joint6output_to_joint6++ j: joint6output_to_joint6--
o:open gripper          p:close gripper
Other:
1 - Go to home pose
q - Quit
```

You can find out which interfaces pymycobot provides in `README.md`.



> Note: Version v3.6.0 differentiates interfaces by model. Starting from this version, the MyCobot class will no longer be maintained. For new usage, please refer to the document: 

![jaywcjlove/sb](https://jaywcjlove.github.io/sb/lang/chinese.svg)   ![jaywcjlove/sb](https://jaywcjlove.github.io/sb/lang/english.svg)

[MyCobot 280 m5 gazebo中文操作](./mycobot_280/280m5_gazebo_gripper/READMECN.md)

## Contributors

Thanks goes to these people ([Emoji Key](https://allcontributors.org/docs/en/emoji-key)):
<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tbody>
    <tr>
      <td align="center" valign="top" width="14.28%"><a href="https://www.youtube.com/user/Apockill"><img src="https://avatars.githubusercontent.com/u/1740412?v=4?s=100" width="100px;" alt="Alex Thiele"/><br /><sub><b>Alex Thiele</b></sub></a><br /><a href="https://github.com/elephantrobotics/mycobot_ros/commits?author=apockill" title="Code">💻</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/nisshan-x"><img src="https://avatars.githubusercontent.com/u/67353276?v=4?s=100" width="100px;" alt="Shinya Nishimoto"/><br /><sub><b>Shinya Nishimoto</b></sub></a><br /><a href="https://github.com/elephantrobotics/mycobot_ros/commits?author=nisshan-x" title="Code">💻</a> <a href="https://github.com/elephantrobotics/mycobot_ros/commits?author=nisshan-x" title="Documentation">📖</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://www.smilerobotics.com"><img src="https://avatars.githubusercontent.com/u/207142?v=4?s=100" width="100px;" alt="Takashi Ogura"/><br /><sub><b>Takashi Ogura</b></sub></a><br /><a href="https://github.com/elephantrobotics/mycobot_ros/commits?author=OTL" title="Code">💻</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/toyoshi"><img src="https://avatars.githubusercontent.com/u/188394?v=4?s=100" width="100px;" alt="Ryuichiro Toyoshi"/><br /><sub><b>Ryuichiro Toyoshi</b></sub></a><br /><a href="https://github.com/elephantrobotics/mycobot_ros/commits?author=toyoshi" title="Documentation">📖</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://ry0.github.io/"><img src="https://avatars.githubusercontent.com/u/8924325?v=4?s=100" width="100px;" alt="Ryo Kabutan"/><br /><sub><b>Ryo Kabutan</b></sub></a><br /><a href="#design-Ry0" title="Design">🎨</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/dieu-detruit"><img src="https://avatars.githubusercontent.com/u/27790373?v=4?s=100" width="100px;" alt="Takafumi Watanabe"/><br /><sub><b>Takafumi Watanabe</b></sub></a><br /><a href="https://github.com/elephantrobotics/mycobot_ros/commits?author=dieu-detruit" title="Code">💻</a> <a href="https://github.com/elephantrobotics/mycobot_ros/commits?author=dieu-detruit" title="Documentation">📖</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/graziegrazie"><img src="https://avatars.githubusercontent.com/u/11900858?v=4?s=100" width="100px;" alt="Yoshiaki Watanabe"/><br /><sub><b>Yoshiaki Watanabe</b></sub></a><br /><a href="https://github.com/elephantrobotics/mycobot_ros/commits?author=graziegrazie" title="Code">💻</a></td>
    </tr>
    <tr>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/mertcookimg"><img src="https://avatars.githubusercontent.com/u/58113372?v=4?s=100" width="100px;" alt="Masato Kobayashi"/><br /><sub><b>Masato Kobayashi</b></sub></a><br /><a href="https://github.com/elephantrobotics/mycobot_ros/commits?author=mertcookimg" title="Code">💻</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://profile.tiryoh.com"><img src="https://avatars.githubusercontent.com/u/3256629?v=4?s=100" width="100px;" alt="Daisuke Sato"/><br /><sub><b>Daisuke Sato</b></sub></a><br /><a href="https://github.com/elephantrobotics/mycobot_ros/commits?author=Tiryoh" title="Documentation">📖</a> <a href="https://github.com/elephantrobotics/mycobot_ros/commits?author=Tiryoh" title="Code">💻</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://www.eacousineau.com/"><img src="https://avatars.githubusercontent.com/u/2806276?v=4?s=100" width="100px;" alt="Eric Cousineau"/><br /><sub><b>Eric Cousineau</b></sub></a><br /><a href="https://github.com/elephantrobotics/mycobot_ros/commits?author=eacousineau" title="Code">💻</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/thandal"><img src="https://avatars.githubusercontent.com/u/2613832?v=4?s=100" width="100px;" alt="thandal"/><br /><sub><b>thandal</b></sub></a><br /><a href="https://github.com/elephantrobotics/mycobot_ros/commits?author=thandal" title="Code">💻</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/nakano16180"><img src="https://avatars.githubusercontent.com/u/36945685?v=4?s=100" width="100px;" alt="nakano16180"/><br /><sub><b>nakano16180</b></sub></a><br /><a href="https://github.com/elephantrobotics/mycobot_ros/commits?author=nakano16180" title="Documentation">📖</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://twitter.com/comocc"><img src="https://avatars.githubusercontent.com/u/843396?v=4?s=100" width="100px;" alt="Akihiro Komori"/><br /><sub><b>Akihiro Komori</b></sub></a><br /><a href="https://github.com/elephantrobotics/mycobot_ros/commits?author=comoc" title="Code">💻</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/mostlyjason"><img src="https://avatars.githubusercontent.com/u/6370704?v=4?s=100" width="100px;" alt="Jason Skowronski"/><br /><sub><b>Jason Skowronski</b></sub></a><br /><a href="https://github.com/elephantrobotics/mycobot_ros/commits?author=mostlyjason" title="Code">💻</a></td>
    </tr>
    <tr>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/2929ss"><img src="https://avatars.githubusercontent.com/u/51234222?v=4?s=100" width="100px;" alt="2929ss"/><br /><sub><b>2929ss</b></sub></a><br /><a href="https://github.com/elephantrobotics/mycobot_ros/commits?author=2929ss" title="Code">💻</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://medium.com/@tks/takasu-profile-c50feee078ac"><img src="https://avatars.githubusercontent.com/u/1667148?v=4?s=100" width="100px;" alt="TAKASU Masakazu"/><br /><sub><b>TAKASU Masakazu</b></sub></a><br /><a href="#promotion-takasumasakazu" title="Promotion">📣</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://scrapbox.io/saitotetsuya/"><img src="https://avatars.githubusercontent.com/u/114928?v=4?s=100" width="100px;" alt="SAITO, Tetsuya"/><br /><sub><b>SAITO, Tetsuya</b></sub></a><br /><a href="#promotion-3110" title="Promotion">📣</a></td>
    </tr>
  </tbody>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://allcontributors.org/)
specification. Contributions of any kind are welcome!

## License

Licensed under standard three-clause BSD license (same as ROS Core), 
Copyright 2020-2023 Elephant Robotics. [Copy of the license](LICENSE).
