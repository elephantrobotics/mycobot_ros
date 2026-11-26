# Pro630 Gazebo ROS 1

## 安装依赖

使用该工程前，需要安装最新的 pymycobot Python API 库：

```bash
pip install pymycobot --user
```

创建工作空间并拉取代码：

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --branch gazebo https://github.com/elephantrobotics/mycobot_ros.git
```

编译并加载 ROS 1 环境：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg mycobotpro630_gazebo
# 或全量编译
# catkin_make
source devel/setup.bash
```

## 前置准备

使用前需要确认有线网关正常才能跟机械臂正常通信，使用 `ip a` 查看有线网卡名称，然后调用：

```bash
# 确认电脑地址 192.168.1.171 未被占用
sudo ip link set <你的有线网卡名称> up
sudo ip addr replace 192.168.1.171/32 dev <你的有线网卡名称>
sudo ip route replace 192.168.1.159/32 dev <你的有线网卡名称> src 192.168.1.171

ip route get 192.168.1.159
ping -c 3 -I <你的有线网卡名称> 192.168.1.159
nc -vz -w 3 192.168.1.159 5001
```

使用本工程前需要参考[前置上电准备](https://docs.elephantrobotics.com/docs/pro630-cn/5-BasicApplication/5.2-ApplicationUse.html#roboflow)，执行完 1.1 系统上电 即可继续使用本工程

## 公共环境

如果机械臂选择末端装有myGripper F100 力控夹爪，则调用以下指令拉起公共环境：

```bash
# 纯仿真环境 带夹爪
roslaunch mycobotpro630_gazebo slider.launch environment:=simulation with_gripper:=true

# 真机环境 带夹爪
roslaunch mycobotpro630_gazebo slider.launch environment:=real with_gripper:=true
```

如果机械臂选择末端没有myGripper F100 力控夹爪，则调用以下指令拉起公共环境：

```bash
# 纯仿真环境 不带夹爪
roslaunch mycobotpro630_gazebo slider.launch environment:=simulation with_gripper:=false

# 真机环境 不带夹爪
roslaunch mycobotpro630_gazebo slider.launch environment:=real with_gripper:=false
```

- 启动成功后，会拉起 Gazebo 仿真环境，并实时跟踪真机机械臂姿态，期间可切换控制器来控制机械臂。
- 真机环境：启动时会读取一次真机姿态快照，并映射到 Gazebo 环境中。使用前需确认机械臂、末端、夹爪的固件版本为最新，并确保机械臂和夹爪通信正常。
- 纯仿真环境：默认从零位开始。

如果坐标控制期间机械臂本体陷入水平面以下，则调用以下指令临时强制拉起公共环境：

```bash
# 真机环境 带夹爪
roslaunch mycobotpro630_gazebo slider.launch environment:=real with_gripper:=true allow_collision:=true

# 真机环境 不带夹爪
roslaunch mycobotpro630_gazebo slider.launch environment:=real with_gripper:=false allow_collision:=true
```

- 启动后的初始阶段，角度控制会被拒绝。需通过键盘控制、GUI 控制沿 Z 轴上移，或通过拖动跟随，使机械臂本体回到水平面以上，再改用正常指令重新拉起公共环境。
- `allow_collision:=true` 仅用于应急：坐标运动导致机械臂本体低于水平面的特殊情况。该参数默认为 `false`，且仅支持真机，即 `environment:=real`。
- 该模式仅临时用于坐标运动导致机械臂本体陷入水平面以下的特殊情况。脱离异常区域后，请切换回正常公共环境使用机械臂。
- 本工程的坐标运动仅限制末端不陷入水平面以下。具体见键盘控制的操作说明。

## 滑块与随机目标

公共环境就绪后，另开终端：

```bash
roslaunch mycobotpro630_gazebo slider_control_gazebo.launch environment:=real
# 纯仿真时使用 environment:=simulation
```

- 启动成功后，会拉起用于`滑块和随机数操作`的 Qt 界面，界面和实际功能会自动适配公共环境是否有 myGripper F100 力控夹爪的情况。
- `Randomize Target` 按钮用于生成六轴与夹爪的随机角度。
- `Zero Target` 按钮用于生成机械臂的零位关节角度。
- 移动滑块、生成随机目标、回零生成关节角度后，均需点击 `Execute` 按钮，机械臂才会运动。
- `Stop` 按钮用于停止正在运动的机械臂。
- `environment` 参数需要与公共环境一致。检测到路径碰撞时，会显示 `Rejected: collision detected` 并拒绝执行。

## 键盘控制

公共环境就绪后，另开终端：

```bash
roslaunch mycobotpro630_gazebo teleop_keyboard.launch
```

- 启动成功后，会拉起用于`键盘控制`的 Qt 界面，界面和实际功能会自动适配公共环境是否有 myGripper F100 力控夹爪的情况。
- 装有 myGripper F100 力控夹爪时，机械臂反馈的坐标信息会延伸到夹爪末端，但进行坐标运动的末端参考依然是link6末端。

仅支持真机。

聚焦窗口后：

- `M`：解锁控制
- `N` 或 `Space`：停止并锁定
- `Q`：关闭窗口并退出
- 按住 `w` / `s`、`a` / `d`、`z` / `x` 分别控制位置 X、Y、Z；按住 `u` / `j`、`i` / `k`、`o` / `l` 分别控制姿态角 RX、RY、RZ
- 按住 `g` / `h` 控制夹爪开合，未安装夹爪时无此功能
- `1`：回到机械臂零位
- `2`：回到坐标控制 Home 位
- `3`：自定义坐标控制 Home 位，将当前位置设置为坐标运动 Home 位，下次启动键盘控制时需重新设置
- 松开按键即停止，窗口失焦即停止
- 108 键布局下，主键盘区的 `-`（`_`）、`+`（`=`），以及小键盘区的 `-` / `+`，均可切换速度档位
- 键盘控制进入奇异位姿时将无法继续控制，需按 `2` 回到坐标控制 Home 位方可重新使用键盘控制。
- 按键控制运动期间，请等待当前运动结束后再开始下一次运动；否则会触发软锁，需再次按 `M` 解锁后方可继续控制。
- 控制期间不允许运动到水平面以下。末端距地面 20 mm 时会触发保护机制，需先长按 `X` 向上运动并脱离触发区域，才能继续其他运动按键；此时仍可调整速度档位。
- 本工程的保护机制仅限制末端，不能保证机械臂本体不与地面碰撞，操作时需谨慎。
- 按键控制不区分大小写。
- 界面持续显示可操作按键、当前设定速度、速度档位、当前有效按键、提示信息等。
- 630 进入奇异位置时，不会改变 Atom 的颜色。如果机械臂静止但六轴坐标跳变，或调用坐标运动时打印 `The current pose may be singular. Please use joint control to move away`，即当前为奇异姿态，需要用角度运动脱离。可以参考[奇异姿态典例](https://docs.elephantrobotics.com/docs/pro630-cn/2-ProductFeature/2.5-CoordinateSystem.html)

坐标运动导致关节超限时：

- 此时机械臂的关节角度运动和坐标运动均会失效。请进入机械臂桌面，使用 RoboFlow 通过关节角度运动脱离超限位置，再回到本工程继续使用机械臂。具体操作可参考 [RoboFlow 使用](https://docs.elephantrobotics.com/docs/pro630-cn/5-BasicApplication/5.2-ApplicationUse.html#13%E5%BF%AB%E9%80%9F%E7%A7%BB%E5%8A%A8) 中的 1.2 新建工程和 1.3 快速移动章节。

## 拖动跟随

公共环境就绪后，另开终端：

```bash
roslaunch mycobotpro630_gazebo drag_follow_gazebo.launch
```

仅支持真机。

- 启动成功后，会拉起用于`拖动跟随`的 Qt 界面，界面和实际功能会自动适配公共环境是否有 myGripper F100 力控夹爪的情况。
- `M`：控制机械臂进入自由拖动模式。进入后机械臂 link5 的 Atom 变为黄色，此时按下 Atom 并听到两声咔咔声即可开始自由拖动；松开 Atom 后机械臂锁紧，但不会退出自由拖动模式。再次按下 `M` 退出自由拖动模式，锁紧六轴和夹爪。
- `Q`：关闭窗口退出。
- 退出拖动跟随界面前，请将机械臂运动到零位附近，并确认关节未超限。程序退出时会自动清错并退出自由拖动模式。

## GUI 控制

公共环境就绪后，另开终端：

```bash
roslaunch mycobotpro630_gazebo gui_control_gazebo.launch
```

仅支持真机。

- 启动成功后，会拉起用于 `GUI 控制` 的 Qt 界面，界面和实际功能会自动适配公共环境是否有 myGripper F100 力控夹爪。
- 装有 myGripper F100 力控夹爪时，该控制器进行坐标运动的末端会自动延伸至夹爪末端。
- 坐标控制需先点击 `Home`，进入坐标控制 Home 位后再开始坐标运动。该按钮也可用于脱离奇异位姿。
- 输入速度、目标角度或坐标后，点击 `Set Gripper`、`Set Joints` 或 `Set Coordinates`，执行相应运动。
- `Stop` 按钮用于停止正在运动的机械臂。
- 角度运动的路径检测到碰撞时，会显示 `Rejected: collision detected` 并拒绝执行。
- 若当前位姿为奇异位姿，在 Qt 界面中调用坐标控制或角度控制时，会提示 `The current pose may be singular. Please use joint control to move away.`。此时无法进行坐标运动，需点击 `Home` 回到坐标控制 Home 位方可恢复运动。

- 630 进入奇异位置时，不会改变 Atom 的颜色。如果机械臂静止但六轴坐标跳变，或调用坐标运动时打印 `The current pose may be singular. Please use joint control to move away`，即当前为奇异姿态，需要用角度运动脱离。可以参考[奇异姿态典例](https://docs.elephantrobotics.com/docs/pro630-cn/2-ProductFeature/2.5-CoordinateSystem.html)

坐标运动导致关节超限时：

- 此时机械臂的关节角度运动和坐标运动均会失效。请进入机械臂桌面，使用 RoboFlow 通过关节角度运动脱离超限位置，再回到本工程继续使用机械臂。具体操作可参考 [RoboFlow 使用](https://docs.elephantrobotics.com/docs/pro630-cn/5-BasicApplication/5.2-ApplicationUse.html#13%E5%BF%AB%E9%80%9F%E7%A7%BB%E5%8A%A8) 中的 1.2 新建工程和 1.3 快速移动章节。
