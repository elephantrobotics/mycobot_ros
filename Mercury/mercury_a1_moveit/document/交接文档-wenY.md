交接文档的link：

https://github.com/YWpepper/mycobot_ros/tree/demo_gazebo/Mercury/mercury_a1_moveit/document

目前电脑中主要有两个环境，其中ubuntu1804是18的环境，其中ubuntu64位是20.04的环境，其中目前只在1804环境下运行gazebo和ros，，密码都为Elephant

![img](https://virginia-pepper.oss-cn-guangzhou.aliyuncs.com/img/blog/202404281510050.jpg)

 

为了实现固定操作，此处修改代码用于固定base世界坐标系的标签

![img](https://virginia-pepper.oss-cn-guangzhou.aliyuncs.com/img/blog/202404281510071.jpg)

 

项目参考gitbook链接：

[https://docs.elephantrobotics.com/docs/gitbook/12-ApplicationBaseROS/12.2-ROS2/12.2.1-ROS2%E7%9A%84%E5%AE%89%E8%A3%85.html](https://docs.elephantrobotics.com/docs/gitbook/12-ApplicationBaseROS/12.2-ROS2/12.2.1-ROS2的安装.html)

 

  项目运行的方式：

roslaunch mercury_a1_moveit demo_gazebo.launch

其中demo.launch文件是ros+rviz的仿真

其中gazebo.launch文件是gazebo的仿真文件

其中demo_gazebo.launch文件是实现两个同步仿真

![img](https://virginia-pepper.oss-cn-guangzhou.aliyuncs.com/img/blog/202404281510121.jpg)

操作过程：

选择目标位置 + plan ，我们在rviz中看见机器完成动作组的规划后，可以使用execute命令发送动作组给到gazebo进行仿真，便完成了操作。

![img](https://virginia-pepper.oss-cn-guangzhou.aliyuncs.com/img/blog/202404281510124.jpg)