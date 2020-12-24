# myCobotROS

<!-- This is the mycobot ROS package designed by Zhang Lijun([lijun.zhang@elephantrobotics.com]()) -->

## 1. Installation

### 1.1 Pre-Requriements

For using this package, the [pymycobot]() library should be installed first.

### 1.2 Package Download and Install

Install ros package in your src folder of your Catkin workspace.

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/elephantrobotics/myCobotROS.git
$ cd ~/catkin_ws
$ catkin_make
```

## 2. Package Modules

### 2.1 Nodes

- `display` is a display node. When the node is running, When the node is running, the model of ROS will show the movement of mycobot synchronously.
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
rocore #open another tab
```

-**Step 2**: Launch

a) For display or marker control, in second terminal, run:

```bash
roslaunch myCobotROS display.launch
```

b) For slider bar control, in second terminal, run:

```
roslaunch myCobotROS control.launch
```

-**Step 3**: Open rviz to view robot

```bash
rosrun rviz rviz
```

-**Step 4**: Run python script

a) For display

```bash
rosrun myCobotRos display.py
```

b) For slider bar.

```bash
rosrun myCobotRos control_slider.py
```

c) For marker control

```bash
rosrun myCobots control_marker.py
```



