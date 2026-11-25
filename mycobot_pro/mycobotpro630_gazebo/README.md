# Pro630 Gazebo（ROS 1）

此包针对 Ubuntu 20.04、ROS Noetic、Gazebo Classic 11 和 MoveIt 1。模型、网格、控制程序和设备适配均随包安装，运行不依赖 ROS 2 工作区。

在现有 catkin 工作区安装依赖并编译：

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
rosdep install --from-paths src/mycobot_ros/mycobot_pro/mycobotpro630_gazebo --ignore-src -r -y
catkin_make --pkg mycobotpro630_gazebo
source devel/setup.bash
```

实机模式另外需要安装含 `ElephantRobot` 的 `pymycobot` SDK（使用与原 ROS 2 工程相同的版本）。纯仿真不需要机械臂或 SDK。

## 公共环境

```bash
roslaunch mycobotpro630_gazebo slider.launch environment:=simulation with_gripper:=true
roslaunch mycobotpro630_gazebo slider.launch environment:=real with_gripper:=true pro630_ip:=192.168.1.159 pro630_port:=5001
```

任选其一。默认使用纯仿真，无夹爪；`gazebo_gui:=false` 只启动 Gazebo 服务端。实机模式先在唯一的 SDK 桥接中获取只读稳定姿态，地面检查通过后用它初始化 Gazebo。Gazebo 的关节位置通过真实反馈持续同步。MoveIt 使用精细碰撞网格，Gazebo 使用独立的凸包网格。实机模式下 MoveIt 的轨迹控制器仍指向 Gazebo，不向 SDK 下发轨迹。

`allow_collision:=true` 仅可用于实机公共环境，允许地面初始碰撞的姿态启动。关节命令仍经过碰撞校验。它不是碰撞检查总开关。

## 控制入口

公共环境启动后，在另一终端任选一个控制功能。各功能互斥，继承公共环境的夹爪安装配置。

| 功能 | 命令 |
|---|---|
| 仿真滑块、随机数 | `roslaunch mycobotpro630_gazebo slider_control_gazebo.launch environment:=simulation` |
| 真机滑块、随机数 | `roslaunch mycobotpro630_gazebo slider_control_gazebo.launch environment:=real` |
| 真机键盘 | `roslaunch mycobotpro630_gazebo teleop_keyboard.launch` |
| 真机拖动跟随 | `roslaunch mycobotpro630_gazebo drag_follow_gazebo.launch` |
| 真机数字输入 GUI | `roslaunch mycobotpro630_gazebo gui_control_gazebo.launch` |
| RViz | `roslaunch mycobotpro630_gazebo moveit_rviz.launch` |

滑块保留 `show_force_execute:=true` 开关，真机会拒绝强制执行。键盘保留坐标点动、松键停止、Init/Home、会话内保存 Home 与软锁。拖动跟随使用控制器自由拖动模式。数字输入 GUI 支持关节、坐标、Home、停止以及装夹爪时的开度控制。带夹爪时，工程在 SDK 适配层沿法兰 +Z 补偿 165 mm；原生键盘旋转点动仍使用设备自身参考点。

辅助入口包括 `move_group.launch`、`rsp.launch`、`setup_assistant.launch` 和 `spawn_controllers.launch`。公共环境运行时，RViz 直接使用已启动的 MoveIt 和模型。ROS 1 轨迹控制器的命令话题为 `/arm_controller/command` 和 `/pro_gripper_controller/command`。

Setup Assistant 会为当前带夹爪或无夹爪配置生成临时编辑副本。可在窗口单击 Load Files，或用 `setup_assistant.launch auto_load:=true` 自动加载。导出时需选择持久化输出目录。

ROS 1 版以 Gazebo 仿真时钟和实测关节反馈确认滑块轨迹完成。MoveIt 1 使用显式的仿真规划加速度限值 (1.0 rad/s²)，供 Pilz 规划和 Gazebo 轨迹执行使用。物理机械臂的 SDK 运动由独立控制链路下发。
