# CANopen 联调命令速查

更新时间：2026-03-25  
适用包：`Eyou_Canopen_Master`

## 1. CAN 启动与检查

```bash
# 1) 加载内核模块
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev

# 2) 配置并拉起 can0（1Mbps）
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 3) 查看状态
ip -details -statistics link show can0
```

抓包：

```bash
# 全量抓包
candump can0

# 关键帧抓包（SYNC + PDO + Heartbeat + EMCY）
candump can0,080:7FF,180:7FF,200:7FF,280:7FF,700:7FF
```

## 2. 赋权命令（可选）

推荐直接用 `sudo` 配置 CAN。  
如需减少 sudo 频率，可给工具加 capability（按需使用）：
```bash
# 先找到节点二进制
realpath ~/Robot24_catkin_ws/devel/lib/Eyou_Canopen_Master/canopen_hw_ros_node

# 给节点提权（打开 RAW CAN 一般需要 NET_RAW；加 NET_ADMIN 更保险）
sudo setcap cap_net_raw,cap_net_admin+eip ~/Robot24_catkin_ws/devel/lib/Eyou_Canopen_Master/canopen_hw_ros_node

# 检查
getcap ~/Robot24_catkin_ws/devel/lib/Eyou_Canopen_Master/canopen_hw_ros_node
```

若 USB-CAN 走串口设备（`/dev/ttyUSB*`）：

```bash
sudo usermod -aG dialout $USER
newgrp dialout
```

## 3. 生成 DCF

```bash
cd ~/Robot24_catkin_ws/src/Eyou_Canopen_Master/config
dcfgen -S -r -d . master.yaml
```

## 4. 编译

```bash
cd ~/Robot24_catkin_ws/src/Eyou_Canopen_Master
cmake --build build -j
```

或 catkin：

```bash
cd ~/Robot24_catkin_ws
catkin_make --pkg Eyou_Canopen_Master
source devel/setup.bash
```

## 5. 启动节点

```bash
cd ~/Robot24_catkin_ws
source devel/setup.bash
roslaunch Eyou_Canopen_Master bringup.launch auto_init:=false
```

常用参数：

```bash
roslaunch Eyou_Canopen_Master bringup.launch \
  dcf_path:=/home/dianhua/Robot24_catkin_ws/src/Eyou_Canopen_Master/config/master.dcf \
  joints_path:=/home/dianhua/Robot24_catkin_ws/src/Eyou_Canopen_Master/config/joints.yaml \
  loop_hz:=200.0 auto_init:=false auto_enable:=false auto_release:=false
```

启用 IP 轨迹执行器：

```bash
roslaunch Eyou_Canopen_Master bringup.launch \
  use_ip_executor:=true \
  ip_executor_action_ns:=arm_position_controller/follow_joint_trajectory

# 可选：覆盖执行器频率
rosparam set /canopen_hw_node/ip_executor_rate_hz 100.0
```

## 6. Service 调用

列出服务：

```bash
rosservice list | grep canopen_hw_node
```

生命周期：

```bash
rosservice call /canopen_hw_node/init "{}"
rosservice call /canopen_hw_node/enable "{}"
rosservice call /canopen_hw_node/disable "{}"
rosservice call /canopen_hw_node/halt "{}"
rosservice call /canopen_hw_node/resume "{}"
rosservice call /canopen_hw_node/recover "{}"
rosservice call /canopen_hw_node/shutdown "{}"
```

状态语义（当前）：
- `init`: `Configured -> Armed`
- `enable`: `Standby -> Armed`
- `disable`: `Running/Armed/Standby -> Standby`
- `halt`: `Running -> Armed`
- `resume`: `Armed -> Running`
- `recover`: `Faulted -> Standby`（不自动上电）
- `shutdown`: `* -> Configured`

模式切换（白名单：`7/8/9/10`）：

```bash
# 指定轴切换到 IP
rosservice call /canopen_hw_node/set_mode "{axis_index: 0, mode: 7}"

# 全轴切换到 CSV
for i in 0 1 2 3 4 5; do
  rosservice call /canopen_hw_node/set_mode "{axis_index: $i, mode: 9}"
done
```

模式定义：
- `7`: IP
- `8`: CSP
- `9`: CSV
- `10`: CST

## 7. Controller 切换命令

```bash
# 查询
rosservice call /controller_manager/list_controllers

# 切到速度控制器（先停 position，再启 velocity）
rosrun controller_manager controller_manager stop arm_position_controller
rosrun controller_manager controller_manager start arm_velocity_controller

# 切回位置控制器
rosrun controller_manager controller_manager stop arm_velocity_controller
rosrun controller_manager controller_manager start arm_position_controller
```

## 8. Topic 查看与发布

查看：

```bash
rostopic list | grep -E "joint_states|diagnostics|controller"
rostopic echo /joint_states
rostopic echo /joint_states/position
rostopic echo /joint_states/velocity
rostopic echo /diagnostics
rostopic hz /joint_states
```

只看单关节（`joint_1`）位置/速度反馈：

```bash
rostopic echo /joint_states | grep -E "name|position|velocity"
```

发布位置轨迹（单关节示例）：

```bash
rostopic pub -1 /arm_position_controller/command trajectory_msgs/JointTrajectory \
"{joint_names: ['joint_1'], points: [{positions: [0.2], time_from_start: {secs: 2, nsecs: 0}}]}"
```

发布速度轨迹（先切到 CSV）：

```bash
rostopic pub -1 /arm_velocity_controller/command trajectory_msgs/JointTrajectory \
"{joint_names: ['joint_1'], points: [{velocities: [0.5], time_from_start: {secs: 1, nsecs: 0}}]}"
```

发送多轴 `FollowJointTrajectory`（IP executor / MoveIt 路径）：

```bash
rostopic pub -1 /arm_position_controller/follow_joint_trajectory/goal \
control_msgs/FollowJointTrajectoryActionGoal \
"{
  goal: {
    trajectory: {
      joint_names: ['joint_1','joint_2'],
      points: [
        {positions: [0.2, -0.1], time_from_start: {secs: 1, nsecs: 0}},
        {positions: [0.4,  0.0], time_from_start: {secs: 2, nsecs: 0}}
      ]
    }
  }
}"
```

## 9. 一键联调顺序（推荐）

```bash
# 1) 拉起 CAN
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up

# 2) 生成 DCF
cd ~/Robot24_catkin_ws/src/Eyou_Canopen_Master/config
dcfgen -S -r -d . master.yaml

# 3) 启动 ROS 节点
cd ~/Robot24_catkin_ws
source devel/setup.bash
roslaunch Eyou_Canopen_Master bringup.launch auto_init:=false

# 4) 初始化
rosservice call /canopen_hw_node/init "{}"

# 5) （可选）切到 IP
rosservice call /canopen_hw_node/disable "{}"
rosservice call /canopen_hw_node/set_mode "{axis_index: 0, mode: 7}"
rosservice call /canopen_hw_node/enable "{}"
rosservice call /canopen_hw_node/resume "{}"
```
