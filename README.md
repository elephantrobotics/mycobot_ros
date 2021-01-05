# myCobotROS

[![jaywcjlove/sb](https://jaywcjlove.github.io/sb/lang/chinese.svg)](READMEcn.md)

[English](README.md) | [中文](READMEcn.md)

![Demo](./Screenshot-1.png)

**Notes**:

<!-- This is the mycobot ROS package designed by Zhang Lijun([lijun.zhang@elephantrobotics.com]()) -->

> Make sure that `Atom2.1alpha` is flashed into the top Atom and `Transponder` is flashed into the base Basic .The tool download address: [https://github.com/elephantrobotics/myCobot/tree/main/Software](https://github.com/elephantrobotics/myCobot/tree/main/Software)<br>
> ubuntu: 16.04LTS<br>
> ros version: 1.12.17

## 1. Installation

### 1.1 Pre-Requriements

For using this package, the [api]() library should be installed first.(demo comes with)

If you want to use api alone, you can look here [https://github.com/elephantrobotics/myCobotROS/blob/main/scripts/pythonAPI/README.md](https://github.com/elephantrobotics/myCobotROS/blob/main/scripts/pythonAPI/README.md)

### 1.2 Package Download and Install

Install ros package in your src folder of your Catkin workspace.

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/elephantrobotics/myCobotROS.git
$ cd ~/catkin_ws
$ catkin_make
```

### 1.3 Test Python API

```bash
cd ~/catkin_ws/src/myCobotROS
python3 scripts/test.py
```

## 2. Package Modules

### 2.1 Nodes

- `display` is a display node. When the node is running, the model of ROS will show the movement of mycobot synchronously.
- `control_slider` is the node which slider bar control.
- `control_marker` is the node which use interactive marker control.

### 2.2 Topics

- `joint_states` - control mycobot status.

  ```
  Message_type: std_msgs/JointState
  Data: position[float, float, float, float, float, float]
  ```

## 3. Visualization in RViz

### 3.1 Functions

- Visualization -- display.launch: This function will display robot arm movement in realtime when you manually move mycobot.

- Control -- control.launch: This function will allow you use slider bar to control movement of the robot arm.

### 3.2 Lanuch and Run

-**Step 1**: In one terminal, open the core.

```bash
roscore #open another tab
```

-**Step 2**: Launch

a) For display or marker control, in second terminal, run:

```bash
roslaunch myCobotROS mycobot.launch
```

b) For slider bar control, in second terminal, run:

```
roslaunch myCobotROS control.launch
```

-**Step 3**: Open rviz to view robot

```bash
rosrun rviz rviz
```

If you use the above command, then you may need to manually add some model components. If you don't want to be so troublesome, you can use the following command to load a saved **myCobot** model.

```bash
rosrun rviz rviz -d rospack find myCobotROS/config/mycobot.rviz
```

-**Step 4**: Run python script

a) For display

```bash
rosrun myCobotROS display.py
```

b) For slider bar.

```bash
rosrun myCobotROS control_slider.py
```

c) For marker control

```bash
rosrun myCobotROS control_marker.py
```

## Q & A

**Q: error[101]**

**A:** Make sure that the serial port is not occupied, and that the correct firmware is burned in for atom and basic
