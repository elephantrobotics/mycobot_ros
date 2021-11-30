Tips：
1、请使用与机械臂相同型号的文件名文件。

2、请先执行以下操作:
1）打开一个新的终端
2）输入命令
chmod +x /home/h/catkin_mycobot/src/mycobot_ros/mycobot_communication/scripts/xxx.py
								(此处为各个新增文件的文件名）
3、jetson nano的文件还没有使用机械臂进行过测试，可能存在问题。

4、数莓派版本的使用:
1）打开VScode,新建一个文件，复制以下内容（请确保电脑与数莓派机械臂已经连接）并运行

from pymycobot import MyCobotSocket

mc = MyCobotSocket("192.168.10.10","9000")
mc.connect()

如果缺少包或版本还未更新请自己安装更新最新版本

2）
打开“网络与internet”设置
更改适配器选项
右键打开数莓派的以太网属性
打开 “internet协议版本4” 的属性
选择 “使用下面的IP地址”
IP地址为 ：  192.168.10.100 （最后一位非10都可）
子网掩码为： 255.255.255.0
确认







