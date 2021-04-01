# myCobotROS

[![jaywcjlove/sb](https://jaywcjlove.github.io/sb/lang/english.svg)](README.md)

[English](README.md) | [中文](READMEcn.md)

![Demo](./Screenshot-1.png)

**注意**:

<!-- This is the mycobot ROS package designed by Zhang Lijun([lijun.zhang@elephantrobotics.com]()) -->

> 请确保顶部的 Atom 烧入 `Atom`，底部的 Basic 烧入 `Transponder`。烧录工具的下载地址: [https://github.com/elephantrobotics/myCobot/tree/main/Software](https://github.com/elephantrobotics/myCobot/tree/main/Software)<br>
> 该包的测试环境：<br> &nbsp;&nbsp;&nbsp;&nbsp; ubuntu: 16.04LTS<br> &nbsp;&nbsp;&nbsp;&nbsp; ros version: 1.12.17

**如果你的`Atom` 是 2.3 或更早的, 或者 `pymycobot` 是 1.\*, 请查看分支 [before](https://github.com/elephantrobotics/myCobotRos/tree/before)**

ROS 下载地址：[http://wiki.ros.org/ROS/Installation](http://wiki.ros.org/ROS/Installation)

## 1. 安装

### 1.1 前提

要使用该包，请确保 [python api](https://github.com/elephantrobotics/pymycobot.git) 已正确安装。

```bash
pip install pymycobot --user
```

### 1.2 Ros 包的下载和安装

将该 ros 包安装到 Catkin 的 src 文件夹中。

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/elephantrobotics/myCobotROS.git
$ cd ~/catkin_ws
$ catkin_make
```

### 1.3 你可以选择测试 Python API

```bash
cd ~/catkin_ws/src/myCobotROS
python scripts/test.py
```

## 2. 模块

### 2.1 节点

- `display` 展示节点。 同步展示 myCobot 的姿态到仿真的模型。
- `control_slider` 通过滑动条控制 myCobot。
- `control_marker` 通过可交互的标记控制 myCobot。

### 2.2 主题

- `joint_states` - 控制和记录 myCobot 的状态.

  ```
  Message_type: std_msgs/JointState
  Data: position[float, float, float, float, float, float]
  ```

## 3. RViz 可视化

### 3.1 功能

- 可视化 -- `display.launch`: This function will display robot arm movement in realtime when you manually move mycobot.

- 控制 -- `control.launch`: This function will allow you use slider bar to control movement of the robot arm.

### 3.2 启动和运行

- 使用滑块控制

  - 启动 ros 和 rviz

  ```
  roslaunch myCobotROS control_slider.launch
  ```

  - 运行 python 脚本

  ```
  rosrun myCobotROS control_slider.py
  ```

- 仿真模型同步机械臂

  - 启动 ros 和 rviz

  ```
  roslanuch myCobotROS mycobot.launch
  ```

  - 运行 python 脚本

  ```
  rosrun myCobotROS display.py
  ```

## Q & A

**Q: error[101]**

**A**: 请确保的你的串口没有被占用，以及 Basic 和 Atom 烧入了正确的固件。
