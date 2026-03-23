# API Quick Reference

## LifecycleManager

| 方法 | 说明 |
|------|------|
| `Init(dcf_path, joints_path)` | 加载配置并启动主站，进入 Active |
| `Init(CanopenMasterConfig)` | 同上，直接传配置结构体 |
| `Configure(config)` | 仅构造对象，进入 Configured |
| `InitMotors()` | 启动通信与驱动流程（Configured -> Active） |
| `Halt()` | 轻量停转：置 Halt bit，保持 Active |
| `Resume()` | 清 Halt bit 恢复运动（global fault latch 期间会被拒绝） |
| `Recover()` | 仅对 fault 轴执行复位（不重启通信、不自动使能；需后续 `Resume()`） |
| `StopCommunication()` | 执行 402 降级 + 停通信，置 `require_init=true` |
| `Shutdown()` | 完全关闭，释放所有资源 |
| `robot_hw()` | 返回 CanopenRobotHw 指针 |

## CanopenMaster

| 方法 | 说明 |
|------|------|
| `GracefulShutdown(detail)` | 402 降级 + NMT Stop；超时返回 false 并给 detail |
| `HaltAll()` / `ResumeAll()` | 全轴置/清 Halt bit（保持 Operation Enabled） |
| `RecoverFaultedAxes(detail)` | 仅对 fault 轴复位，超时返回 false 并给 detail |
| `EnableAxis/DisableAxis/ResetAxisFault` | 单轴手动控制接口 |
| `GetAxisFeedback(i, out)` | 读取单轴反馈快照 |

## CanopenRobotHw

| 方法 | 说明 |
|------|------|
| `ReadFromSharedState()` | 从 SharedState 读取全轴反馈快照 |
| `WriteToSharedState()` | 将目标命令写入 SharedState |
| `ApplyConfig(config)` | 应用轴配置（单位换算参数等） |
| `SetJointCommand(axis, rad)` | 设置单轴目标位置（弧度） |
| `SetJointVelocityCommand(axis, rad_s)` | 设置单轴目标速度（rad/s） |
| `SetJointTorqueCommand(axis, nm)` | 设置单轴目标力矩（Nm） |
| `SetJointMode(axis, mode)` | 设置单轴运动模式（CSP/CSV/CST） |
| `SetCommandReady(axis, ready)` | 写入命令有效标志（AxisCommand.valid） |
| `SetCommandEpoch(axis, epoch)` | 写入命令会话号（AxisCommand.arm_epoch） |
| `arm_epoch(axis)` | 读取反馈会话号（AxisFeedback.arm_epoch） |
| `all_axes_halted_by_fault()` | 读取全轴故障连带停机标志 |

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
| `SetExternalCommand(cmd)` | 写入上层命令包（target/mode/valid/epoch） |
| `SetGlobalFault(fault)` | 注入全局故障闩锁输入 |
| `RequestEnable()` / `RequestDisable()` | 使能/去使能 |
| `RequestHalt()` / `RequestResume()` | 置/清 Halt bit |
| `ResetFault()` | 复位故障 |

## SharedState 协议字段

`AxisCommand`：

| 字段 | 说明 |
|------|------|
| `target_position/velocity/torque` | 上层期望目标（工程量已换算为设备单位） |
| `mode_of_operation` | 目标模式（CSP/CSV/CST） |
| `valid` | 上层命令源是否完成重同步并声明可用 |
| `arm_epoch` | 目标所属使能会话号（`0` 永远无效） |

`SharedSnapshot`：

| 字段 | 说明 |
|------|------|
| `global_fault` | 任一轴故障后置位的全局闩锁 |
| `all_axes_halted_by_fault` | 全轴因故障被连带冻结标志 |

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
| `kMode_IP` | 7 | Interpolated Position |
| `kMode_CSP` | 8 | Cyclic Synchronous Position |
| `kMode_CSV` | 9 | Cyclic Synchronous Velocity |
| `kMode_CST` | 10 | Cyclic Synchronous Torque |
