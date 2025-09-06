# LLM for Robotics

设备与环境：
* franka机械臂 FR3
* ubuntu 22.04
* ROS2 Humble


## ROS2 action 控制机械臂

该程序主要有三个控制器，分别是
* `ik_impedance_controller`: 用于逆运动学解算，通过moveit求解，可以将机械臂从一个点移动到另一个点;
* `gripper_controller`: 用于控制夹爪夹取物体
* `cartesian_pose_controller`: 用于末端保持旋转姿态，在笛卡尔坐标系下进行平动，即改变其位置。

> 注意，`ik_impedance_controller`和`cartesian_pose_controller`不可同时启动，两个控制器启动之后自带夹爪控制器，后续会将两个控制器合并。

根据ros2 action的调试指令，可快速指定具体动作：

```bash
ros2 action send_goal <action_name> <action_type> <values>
```

### 逆运动学控制器

启动逆运动学解算控制器
```bash
cd ~/llm_robotics_ws
source install/setup.bash
ros2 launch ik_impedance_controller start.py
```

发送特定坐标指令
```bash
ros2 action send_goal /ik_controller communication_interfaces/action/Target "{target_point: [0.442466, 0.268252, 0.306255, -0.025996, 0.999311, 0.025939, 0.005396]}"
```
显然，该七维向量为机械臂的目标点，分别是三个x,y,z位置坐标和表示旋转的四元数坐标；

该机械臂会往该目标点进行移动。

通过`ros2 topic echo可以实时读取末端的位置`

使用默认控制器启动
```bash
ros2 launch franka_bringup move_to_start_example_controller.launch.py robot_ip:=192.168.8.2
```

新建另一个终端，通过指令可以看到当前的末端坐标，可用作定位。

```bash
ros2 topic echo /franka_robot_state_broadcaster/current_pose
```

要注意的是，输入控制器的坐标顺序是`[x, y, z, w, x, y, z]`，前三个是坐标，后四个是四元数

### 夹爪控制器

在启动`ik_impedance_controller`或`cartesian_pose_controller`之后，新建一个终端，输入以下指令

```bash
ros2 action send_goal /fr3_gripper/move franka_msgs/action/Move "{width: 0.06, speed: 0.01}"
```

其中`width`为宽度（0.00～0.08）, `speed`为速度（0.01~0.03）即可

### 笛卡尔控制器

启动控制器
```bash
cd ~/llm_robotics_ws
source install/setup.bash
ros2 launch cartesian_pose_controller start.py
```

发送特定的指令
```bash
ros2 action send_goal /cartesian_pose_controller communication_interfaces/action/Cartesian "{cartesian_move: [0.05, 0.00, 0.00, 3]}"
```
`[x, y, z, time]`前三个参数代表三个轴的位移（m），后面一个为时间（s），一般设置2-3s


