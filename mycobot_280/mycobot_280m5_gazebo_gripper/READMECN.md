### 本地 1.操作流程
#### 1.1 安装前提

要使用此包，需先安装[Python api](https://github.com/elephantrobotics/pymycobot.git)库。

```bash
pip install pymycobot --user
ros1 noetic
```

#### 1.2 包的下载与安装

下载包到你的ros工作空间中

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/jiaweilong66/280m5_gripper_gazebo.git
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
```
MyCobot_280_m5-Gazebo使用说明
1. 滑块控制
现已实现通过joint_state_publisher_gui的滑块控制机械臂模型在Gazebo中的位姿
确认将真实的机械臂连接到电脑以后，查看机械臂连接的端口：

```bash
ls /dev/tty*
/dev/ttyACM0 or /dev/ttyUSB0
```

得到如下的输出结果：

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

打开通信，给脚本添加执行权限

```bash
sudo chmod -R 777 /dev/ttyACM0  or sudo chmod -r 777 /dev/ttyUSB0
sudo chmod -R 777 mycobot_280/mycobot_280m5_gazebo_gripper/scripts/follow_display_gazebo.py
sudo chmod -R 777 mycobot_280/mycobot_280m5_gazebo_gripper/scripts/slider_control_gazebo.py
sudo chmod -R 777 mycobot_280/mycobot_280m5_gazebo_gripper/scripts/teleop_keyboard_gazebo.py
roscor
```

确认好端口后，打开一个终端输入以下命令，注意port改成上一步查询到的值

```bash
source devel/setup.bash
roslaunch mycobot_280m5_gazebo_gripper slider.launch _port:=/dev/ttyACM0 _baud:=115200
```

接着打开另外一个终端，输入如下命令：

```bash
source devel/setup.bash
rosrun mycobot_280m5_gazebo_gripper slider_control_gazebo.py _port:=/dev/ttyACM0 _baud:=115200
```

同样记得把端口号修改成上一步查询到的端口号。如果运行成功将会看到如下的终端提示：

```bash
('/dev/ttyACM0', 115200)
spin ...
```

此时便可通过操控joint_state_publisher_gui的滑块来操控Gazebo或者机械臂模型的位姿了。

2. Gazebo模型跟随
通过如下的命令可以实现Gazebo中的模型跟随实际机械臂的运动而发生位姿的改变，首先运行launch文件：

```bash
source devel/setup.bash
roslaunch mycobot_280m5_gazebo_gripper follower.launch _port:=/dev/ttyACM0
```

如果程序运行成功，Gazebo界面将成功加载机械臂模型，机械臂模型的所有关节都处于原始位姿，即[0,0,0,0,0,0]. 此后我们打开第二个终端并运行：

```bash
source devel/setup.bash
rosrun mycobot_280m5_gazebo_gripper follow_display_gazebo.py _port:=/dev/ttyACM0 _baud:=115200
```

现在当我们操控实际机械臂的位姿，我们可以看到Gazebo中的机械臂也会跟着一起运动到相同的位姿。

3. 键盘控制
我们还可以使用键盘输入的方式同时操控Gazebo中机械臂模型与实际机械臂的位姿，首先打开一个终端并输入：

```bash
source devel/setup.bash
roslaunch mycobot_280m5_gazebo_gripper teleop_keyboard.launch _port:=/dev/ttyACM0 _baud:=115200
```

同上一部分相同，我们会看到机械臂模型被加载到Gazebo中，并且所有关节都在初始的位姿上，紧接着我们打开另外一个终端并输入：

```bash
source devel/setup.bash
rosrun mycobot_280m5_gazebo_gripper teleop_keyboard_gazebo.py _port:=/dev/ttyACM0 _baud:=115200
```

如果运行成功，我们将在终端看到如下的输出信息：

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

根据上面的提示我们可以知道如何操控机械臂运动了，这里我设置每点击一下机械臂与Gazebo中的机械臂模型会运动1角度，可以尝试长按上述键位中的其中一个键来到达某一位姿。
