# 项目总览：CANopen 主站控制栈

更新时间：2026-03-25  
范围：`Eyou_Canopen_Master`

---

## 1. 当前架构结论

项目已不是“纯独立程序”形态，当前同时支持：

1. 核心库层：`canopen_core`（不依赖 ROS）
2. ROS 适配层：`canopen_ros_adapter`
3. ROS 入口节点：`canopen_hw_ros_node`

核心运行链路是：

`CanopenMaster/AxisDriver/AxisLogic`  
`<-> SharedState <->`  
`CanopenRobotHw/CanopenRobotHwRos/OperationalCoordinator/ServiceGateway`

---

## 2. 主要模块

### 2.1 canopen_core（底层与状态机）

- `CanopenMaster`：Lely I/O 与主站生命周期、轴驱动创建、优雅停机。
- `AxisDriver`：单轴 Lely 回调接入。
- `AxisLogic` + `CiA402StateMachine`：状态机推进、控制字与目标输出策略。
- `SharedState`：跨线程数据面（反馈、命令、intent、故障闩锁、同步序列）。
- `LifecycleManager`：资源所有权与生命周期（配置、初始化、停机）。
- `OperationalCoordinator`：系统级模式编排（Configured/Standby/Armed/Running/Faulted）。

### 2.2 canopen_ros_adapter（ROS 接线）

- `CanopenRobotHwRos`：`hardware_interface::RobotHW` 适配。
- `ServiceGateway`：`~/init ~/enable ~/disable ~/halt ~/resume ~/recover ~/shutdown` 服务桥接。
- `IpFollowJointTrajectoryExecutor`：IP 模式轨迹执行器（多轴、动态 DOF）。

### 2.3 节点与控制器

- 节点：`canopen_hw_ros_node`
- 控制器管理：`controller_manager`
- 默认保留 `arm_position_controller`/`arm_velocity_controller` 配置；
  启用 `use_ip_executor` 时由自定义 executor 接管 `FollowJointTrajectory` action 执行。

---

## 3. 线程与循环模型

### 3.1 Lely 事件线程

负责 CANopen 事件循环与 PDO/EMCY/Heartbeat 回调，更新 `SharedState.feedback`。

### 3.2 ROS 控制循环线程

`canopen_hw_ros_node` 主循环（典型 200Hz）顺序：

1. `coordinator.UpdateFromFeedback()`
2. `coordinator.ComputeIntents()`
3. `robot_hw_ros.read()`
4. `controller_manager.update()`
5. `ip_executor.update()`（启用时）
6. `robot_hw_ros.write()`
7. `diagnostics` 更新

---

## 4. 数据协议（关键字段）

### 4.1 命令侧（AxisCommand）

- `target_position/target_velocity/target_torque`
- `mode_of_operation`
- `valid`
- `arm_epoch`

### 4.2 快照侧（SharedSnapshot）

- `feedback[]`
- `commands[]`
- `global_fault`
- `all_axes_halted_by_fault`
- `intents[]`
- `intent_sequence`
- `command_sync_sequence`

`command_sync_sequence` 用于声明“旧命令源失效，需要重新对齐”。

---

## 5. IP 轨迹执行器位置

执行器当前代码位于：

- `include/canopen_hw/controllers/ip_follow_joint_trajectory_executor.hpp`
- `src/controllers/ip_follow_joint_trajectory_executor_validation.cpp`
- `src/controllers/ip_follow_joint_trajectory_executor_runtime.cpp`
- `src/controllers/ip_follow_joint_trajectory_executor_ros.cpp`

特性：

1. 单个 action goal 覆盖全部配置轴（MoveIt 标准语义）。
2. `Ruckig<0>` 动态 DOF，轴数来自 `joints.yaml`。
3. 每轴约束来自 `joints.yaml` 的 `ip_max_*` 和 `ip_goal_tolerance`。

详见：`docs/ip_follow_joint_trajectory_executor.md`。

---

## 6. 配置来源

### 6.1 `master.yaml/master.dcf`

- `master.yaml` 由 `dcfgen` 生成 `master.dcf`。
- `master.dcf` 是启动硬依赖（缺失会拒绝启动）。

### 6.2 `joints.yaml`

运行时参数入口：

- CAN 接口与 loop 频率
- 每轴 node id、缩放、限幅
- IP 插补周期与 executor 约束

---

## 7. 当前文档入口

1. `docs/usage.md`：部署与联调步骤
2. `docs/api_reference.md`：接口速查
3. `docs/yaml_config_guide.md`：配置字段说明
4. `docs/2026-03-25_现行安全行为规范.md`：安全行为基线
