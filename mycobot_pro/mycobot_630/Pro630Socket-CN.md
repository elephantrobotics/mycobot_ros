# Pro630基于Socket通信的ROS操控指南

## 1. 安装

### 1.1 Ubuntu与ROS1的安装与配置

首先请根据[大象机器人官网](https://docs.elephantrobotics.com/docs/gitbook/12-ApplicationBaseROS/12.1-ROS1/12.1.2-%E7%8E%AF%E5%A2%83%E6%90%AD%E5%BB%BA.html)提供的指南完成Ubuntu的安装与配置、ROS1的安装与配置以及MoveIt！的安装与配置。

本项目推荐使用Ubuntu18.04版本，对应的ROS版本为ros-melodic，为了验证以上的配置是否一切正常，用户可以在终端输入指令：

```bash
roslaunch mycobot_280 slider_control.launch
```

如果上述的安装与配置都正常的话，将会出现如下的界面，通过拖动滑块，可以操控机械臂各个关节的位姿。

<img src="file:///C:/Users/12174/Desktop/Pro630/image/1.png" title="" alt="1.png" data-align="center">

### 1.2 pymycobot库的安装与配置

有两种方式来安装pymycobot库，第一种是通过pip install的方法，另外一种是通过克隆源码的方式完成安装，接下来会逐一介绍

#### 1.2.1 pip install 安装

打开终端输入命令

```bash
pip install pymycobot==3.5.0a3
```

想要验证是否安装成功，可以在命令行输入

```bash
pip list | grep pymycobot
```

如果能够正常安装，终端会输出`pymycobot`及其对应的版本

#### 1.2.2 通过克隆源码安装

首先进入需要安装pymycobot库的路径下，在终端输入

```bash
cd /home/<username>/catkin_ws/src/mycobot_ros/mycobot_pro/mycobot_630/scripts
```

将其中的`<username>`替换为当前用户的的用户名，用户名可以通过在命令行中输入`who`指令来获得。

进入该路径后，通过克隆源码的方式安装pymycobot库

```bash
git clone https://github.com/elephantrobotics/pymycobot.git
```

进入克隆的文件夹

```bash
cd /home/<username>/catkin_ws/src/mycobot_ros/mycobot_pro/mycobot_630/scripts/pymycobot
```

同样地，需要将其中的`<username>`替换为当前用户的的用户名。进入该文件夹后，切换git的分支

```bash
git checkout pro630_esp32
```

如果一切正常，终端将会有如下的输出

```bash
branch 'pro630_esp32' set up to track 'origin/pro630_esp32'.
Switched to a new branch 'pro630_esp32'
```

最后通过命令安装这个库

```bash
sudo python2 setup.py install
```

## 2. 运行

首先在PC端开启一个终端并运行

```bash
roslaunch mycobot_630 mycobot_630_slider.launch
```

将会弹出`RViz`的可视化窗口以及`jont_state_publisher_gui`的滑块界面。

在保证树莓派端服务器的代码正常运行的前提下，再在PC端开启另外一个终端并运行

**注意：** 机械臂刚开始的位姿是与地面平行的，请务必在运行下面这行命令前保证机械臂有**足够**的`运动空间`并保证在使用过程中如发生意外情况使用者能随时按下`急停`按钮！

```bash
rosrun mycobot_630 pro630_slider_control.py
```
