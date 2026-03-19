# API Quick Reference

## LifecycleManager

| 方法 | 说明 |
|------|------|
| `Init(dcf_path, joints_path)` | 加载配置并启动主站，进入 Active |
| `Init(CanopenMasterConfig)` | 同上，直接传配置结构体 |
| `Halt()` | 优雅停机，进入 Configured |
| `Recover()` | 从 Configured 重启主站，回到 Active |
| `Shutdown()` | 完全关闭，释放所有资源 |
| `robot_hw()` | 返回 CanopenRobotHw 指针 |

## CanopenRobotHw

| 方法 | 说明 |
|------|------|
| `ReadFromSharedState()` | 从 SharedState 读取全轴反馈快照 |
| `WriteToSharedState()` | 将目标命令写入 SharedState |
| `ApplyConfig(config)` | 应用轴配置（单位换算参数等） |
| `SetJointPositionCommand(axis, rad)` | 设置单轴目标位置（弧度） |
| `SetJointVelocityCommand(axis, rad_s)` | 设置单轴目标速度（rad/s） |
| `SetJointTorqueCommand(axis, nm)` | 设置单轴目标力矩（Nm） |
| `SetModeOfOperation(axis, mode)` | 设置单轴运动模式（CSP/CSV/CST） |
| `GetJointPosition(axis)` | 获取单轴实际位置（弧度） |
| `GetJointVelocity(axis)` | 获取单轴实际速度（rad/s） |
| `GetJointTorque(axis)` | 获取单轴实际力矩（Nm） |

## AxisLogic

| 方法 | 说明 |
|------|------|
| `ProcessRpdo(sw, pos, vel, torque, mode)` | RPDO 周期处理：反馈→状态机→写总线 |
| `ProcessEmcy(eec, er)` | EMCY 处理，递增健康计数 |
| `ProcessHeartbeat(lost)` | 心跳丢失/恢复处理 |
| `Configure(threshold, max_resets, hold)` | 配置状态机参数 |
| `SetRosTarget(pos)` | 设置目标位置 |
| `SetRosTargetVelocity(vel)` | 设置目标速度 |
| `SetRosTargetTorque(torque)` | 设置目标力矩 |
| `SetTargetMode(mode)` | 设置运动模式 |
| `RequestEnable()` / `RequestDisable()` | 使能/去使能 |
| `ResetFault()` | 复位故障 |

## BusIO (接口)

| 方法 | CiA 402 对象 |
|------|-------------|
| `WriteControlword(cw)` | 0x6040 |
| `WriteTargetPosition(pos)` | 0x607A |
| `WriteTargetVelocity(vel)` | 0x60FF |
| `WriteTargetTorque(torque)` | 0x6071 |
| `WriteModeOfOperation(mode)` | 0x6060 |

## SdoAccessor

| 方法 | 说明 |
|------|------|
| `Read(index, subindex)` | 同步 SDO 读（返回 SdoResult） |
| `Write(index, subindex, data)` | 同步 SDO 写 |
| `WriteU8/U16/U32(index, sub, val)` | 类型化同步写 |

## DiagnosticsCollector

| 方法 | 说明 |
|------|------|
| `Collect()` | 收集全轴诊断信息（健康计数、状态等） |

## RealtimeLoop

| 方法 | 说明 |
|------|------|
| `Run(tick)` | 执行周期循环，tick 返回 false 时退出 |
| `stats()` | 返回 LoopStats（max/avg/last jitter, iterations） |

## 运动模式常量

| 常量 | 值 | 说明 |
|------|----|------|
| `kMode_CSP` | 8 | Cyclic Synchronous Position |
| `kMode_CSV` | 9 | Cyclic Synchronous Velocity |
| `kMode_CST` | 10 | Cyclic Synchronous Torque |
