# ROS1 280m5 Gazebo使用说明

## 1. 安装与配置

### 1.1 环境搭建与基础配置

本项目基于Ubuntu18.04及其对应的ROS版本ROS-melodic进行开发，具体的安装过程请参考[大象机器人官网安装指导](https://docs.elephantrobotics.com/docs/gitbook/12-ApplicationBaseROS/12.1-ROS1/12.1.2-%E7%8E%AF%E5%A2%83%E6%90%AD%E5%BB%BA.html)进行安装和配置。

为确认安装与配置是否正确，在终端命令行中输入

```bash
cd ~/catkin_ws
roslaunch mycobot_280 test.launch
```

如果能看到如下所示的画面并能够通过滑块操控Rviz中机械臂模型的位姿，则说明安装与配置都一切正常

<img src="./image/2.png" title="" alt="2.png" data-align="center">

### 1.2 安装和配置Gazebo

在安装`gazebo`的各种依赖包之前，请先确认`gazebo`是否能正常运行和打开，打开终端命令行并输入

```bash
gazebo
```

如果一切正常，Gazebo将会加载一个空的世界，如下图

<img src="./image/10.png" title="" alt="10.png" data-align="center">

你还可以通过`gazebo --version`命令查看其版本，你将得到输出

```bash
Gazebo multi-robot simulator, version 9.0.0
Copyright (C) 2012 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org


Gazebo multi-robot simulator, version 9.0.0
Copyright (C) 2012 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org
```

至此，你的环境配置已经支持Gazebo的运行了，紧接着你需要安装Gazebo在ROS中的一些依赖包（gazebo_ros_pkgs），在终端输入

```bash
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
```

本说明使用的是`melodic`版本，如果使用的是其它版本的ROS，请将命令中的`melodic`替换成对应的版本名称

紧接着你需要为`.bashrc`文件添加`setup`脚本源，输入命令：

```bash
source /opt/ros/molodic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

## 2. 使用

### 2.1 滑块控制

`Ctrl + Alt + T`打开一个终端，进入到`catkin_ws`这个路径下，对项目进行编译

```bash
catkin_make
```

随后启动`Gazebo`以及滑块控制界面

```bash
roslaunch mycobot_280_gazebo_moveit slider_control_gazebo.launch
```

你将会看到如下图所示的界面

<img src="./image/11.png" title="" alt="11.png" data-align="center">

另外打开一个终端，同样地进入到`catkin_ws`路径下并输入

```bash
rosrun mycobot_280_gazebo_moveit slider_control_gazebo.py 
```

终端将会输出`spin ...`，此时说明脚本已经开始监听滑块的数值并且将这些数值发送给`Gazebo`中的机械臂模型了，此时拖动滑块，你就可以看到机械臂模型的运动，至此，就完成了Moveit滑块对机械臂模型的操控

### 2.2 模型跟随

`Ctrl + Alt + T`打开一个终端，进入到`catkin_ws`这个路径下，对项目进行编译

```bash
catkin_make
```

随后启动`Gazebo`并加载机械臂模型

```bash
roslaunch mycobot_280_gazebo_moveit mycobot_follow_gazebo.launch
```

Gazebo将会加载机械臂模型，随后打开另外一个终端并输入

```bash
rosrun mycobot_280_gazebo_moveit follow_display_gazebo.py _port:=/dev/ttyACM0 _baud:=115200
```

注意这里的端口号并不一定是ACM0，可以通过命令`'ls /dev/tty*`进行查询

如果是在虚拟机中运行，可能无法找到对应的端口，这可能是因为该虚拟机没有开启USB，为了解决这个问题，可以先关闭虚拟机，然后在设置中找到`USB设备`，并勾选`启用USB控制器`选项，选择`USB 3.0`，并点击右侧第二个按钮添加280m5机械臂对应的USB设备

这里还有可能遇到`Permission Denied`的问题，以/dev/ttyACM0这个端口为例，需要输入以下这行命令

```bash
sudo chmod 777 /dev/ttyACM0
```

如果端口是其它名字，直接进行替换即可。再次运行

```bash
rosrun mycobot_280_gazebo_moveit follow_display_gazebo.py _port:=/dev/ttyACM0 _baud:=115200
```

就可以看到Gazebo机械臂模型跟随机械臂运动了

### 2.3 键盘控制

你还可以使用键盘输入的方式同时操控Gazebo中机械臂模型的位姿，首先打开一个终端并输入：

```bash
roslaunch mycobot_280_gazebo_moveit teleop_keyboard_gazebo.launch同上一部分相同，我们会看到机械臂模型被加载到Gazebo中，并且所有关节都在初始的位姿上，紧接着我们打开另外一个终端并输入：
```

```bash
rosrun mycobot_280_gazebo_moveit teleop_keyboard_gazebo.py
```

如果运行成功，我们将在终端看到如下的输出信息：

```shell
Mycobot_280_m5 Teleop Keyboard Controller
---------------------------
Movimg options (control the angle of each joint):
    w: joint2_to_joint1++   s: joint2_to_joint1--
    e: joint3_to_joint2++   d: joint3_to_joint2--
    r: joint4_to_joint3++   f: joint4_to_joint3--
    t: joint5_to_joint4++   g: joint5_to_joint4--
    y: joint6_to_joint5++   h: joint6_to_joint5--
    u: joint6output_to_joint6++ j: joint6output_to_joint6--

Other:
    1 - Go to init pose
    2 - Go to home pose
    3 - Resave home pose
    q - Quit
```

根据上面的提示我们可以知道如何操控机械臂运动了，这里我设置每点击一下机械臂与Gazebo中的机械臂模型会运动1角度，这个运动是不明显的，可以尝试长按上述键位中的其中一个键来到达某一位姿。
