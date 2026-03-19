# 项目文档：CANopen 主站控制栈

日期：2026-03-17  
范围：当前仓库代码与文档（含 PDO 映射验证）

---

## 1. 项目简介

本项目实现面向 6 轴谐波关节的 CANopen 主站控制栈，基于 Lely-core，支持 CSP/CSV/CST 三种运动模式。系统由三层核心模块组成：

- CANopen 主站层：负责 Lely I/O 初始化、AsyncMaster 事件循环与从站驱动管理。
- 轴驱动层：每轴一个驱动，通过 AxisLogic + BusIO 接口接入 CiA402 状态机与 PDO/SDO 交互。
- 上层接口层：以 `SharedState` 作为线程间共享数据面，连接控制算法或 ROS 侧。

当前版本为独立可运行程序（`canopen_hw_node`），未接入 ROS。主循环以 100Hz 通过 RealtimeLoop（clock_nanosleep 绝对时间等待）读写 `SharedState`，与 Lely 事件线程并行运行。

---

## 2. 目标与设计原则

目标：
- 启动时自动完成 DCF 配置并进入 CSP 控制流程。
- 支持启动时 PDO 映射验证，发现不一致即阻断该轴进入 Operational。
- 以可观测性与稳定性为优先，避免隐藏失败状态。

设计原则：
- 事件驱动在 Lely 线程，控制算法在应用线程。
- 所有运行期交换通过 `SharedState`，避免跨线程直接操作。
- 不在回调线程做阻塞 I/O。
- SYNC 由主站产生，控制周期与应用线程对齐。

---

## 3. 代码结构

```
include/
  canopen_hw/
    axis_driver.hpp           # Lely 适配器，实现 BusIO，委托 AxisLogic
    axis_logic.hpp            # 单轴核心逻辑（状态机、反馈、健康计数）
    bus_io.hpp                # 总线写入抽象接口
    canopen_master.hpp        # 主站与 Lely 生命周期管理
    canopen_robot_hw.hpp      # 机器人硬件抽象层（读写 SharedState）
    canopen_robot_hw_ros.hpp  # ROS1 ros_control 适配层
    cia402_state_machine.hpp  # CiA402 状态机（CSP/CSV/CST）
    lifecycle_manager.hpp     # 生命周期管理（Init/Halt/Recover/Shutdown）
    joints_config.hpp         # joints.yaml 解析
    pdo_mapping.hpp           # PDO 读取/比对
    realtime_loop.hpp         # 周期性实时循环（clock_nanosleep）
    shared_state.hpp          # 共享状态
src/
  axis_driver.cpp
  axis_logic.cpp
  canopen_master.cpp
  canopen_robot_hw.cpp
  canopen_robot_hw_ros.cpp    # ROS 适配层实现
  canopen_hw_ros_node.cpp     # ROS 节点入口
  cia402_state_machine.cpp
  joints_config.cpp
  lifecycle_manager.cpp
  pdo_mapping.cpp
  realtime_loop.cpp
  shared_state.cpp
  main.cpp                    # 独立模式入口
srv/
  SetMode.srv                 # 运动模式切换 service 定义
config/
  joints.yaml
  master.yaml
  master.dcf
  ros/controllers.yaml        # ros_control controller 配置
launch/
  canopen_hw.launch           # ROS 一键启动
```

---

## 4. 核心模块说明

### 4.1 CanopenMaster

职责：初始化 Lely I/O，创建 `AsyncMaster` 与事件线程，管理轴驱动生命周期。

关键流程：
- 创建 `IoGuard/Context/Poll/Loop/Timer`。
- 打开 `CanController/CanChannel`。
- 创建 `AsyncMaster`，加载 DCF。
- `CreateAxisDrivers()`：创建 6 轴驱动。
- 启动 `ev::Loop` 线程，并 `Reset()` 触发配置流程。

### 4.2 AxisDriver / AxisLogic / BusIO

AxisDriver 是 Lely 适配器，继承 `BasicDriver` 并实现 `BusIO` 接口。核心逻辑委托给 `AxisLogic`。

- `AxisDriver`：Lely 回调转发 + PDO 读写 + PDO 验证 + SDO 转发
- `AxisLogic`：状态机驱动、反馈缓存、健康计数、安全目标计算
- `BusIO`：纯虚接口（WriteControlword/Position/Velocity/Torque/Mode），可 mock 测试

关键功能：
- `OnRpdoWrite()`：读取 RPDO 反馈，委托 AxisLogic::ProcessRpdo 推进状态机并写安全目标
- `OnBoot()`：触发 PDO 映射验证；失败则阻断该轴 Operational
- `OnEmcy()`/`OnHeartbeat()`：委托 AxisLogic 处理，更新健康计数和 SharedState
- PDO 下行：支持 0x607A（位置）、0x60FF（速度）、0x6071（力矩）、0x6060（模式）

### 4.3 SharedState

职责：线程安全的状态交换。Lely 回调写入反馈，应用线程读取。应用线程写入目标，Lely 线程读取。

同步机制：
- `SharedState` 内部使用 `std::mutex` 保护读写。
- `Snapshot()` 在锁内拷贝 6 轴数据，调用方后续使用不持锁。

核心字段：
- `AxisFeedback`：状态字、位置/速度/力矩、模式等。
- `AxisCommand`：目标位置等。
- `all_operational`：全轴是否可运行。

### 4.4 PdoMappingReader & Diff

职责：
- `PdoMappingReader` 通过 SDO 读回 PDO 映射（COB-ID + 映射条目）。
- `DiffPdoMapping` 对比 DCF 期望映射与读回结果，输出差异明细。

约束：
- 只比对 COB-ID 和映射内容，不比对传输类型、事件定时等。

---

## 5. 数据流向

### 5.1 线程与数据流

```
应用线程 (100Hz)
  CanopenRobotHw::ReadFromSharedState()  <-  SharedState::Snapshot
  CanopenRobotHw::WriteToSharedState()   ->  SharedState::UpdateCommand

Lely 事件线程
  AxisDriver::OnRpdoWrite() -> 读取 RPDO-mapped 反馈
  AxisDriver::InjectFeedback() -> 状态机更新 -> SharedState::UpdateFeedback
  SharedState::RecomputeAllOperational()
```

### 5.2 启动时序 (关键步骤)

```
1) 启动 canopen_hw_node
2) 创建 Lely AsyncMaster
3) Lely OnConfig 自动下发 DCF
4) 从站进入 Operational -> OnBoot
5) 可选：PDO 映射验证
6) 状态机推进至 OPERATION_ENABLED
```

### 5.3 SYNC 与应用周期

- SYNC 由主站通过 `ev::Loop` 定时器产生（历史设计见 `docs/archive/canopenplan.md`）。
- 目标周期：10ms（与应用线程 100Hz 对齐）。
- 若未来需要与外部时钟对齐，应明确时间戳与采样对齐策略。

---

## 6. PDO 映射验证机制

- 配置项：`joints[].canopen.verify_pdo_mapping`
- 验证流程：
  1) 启动时加载 DCF 为期望映射
  2) OnBoot 触发后通过 SDO 读回实际映射
  3) 对比不一致则该轴 `is_operational=false`

超时策略：
- SDO 读回超时（默认 2s）直接判失败

---

## 7. 配置、构建与运行

### 7.1 关键配置

- `config/master.yaml` -> 生成 `master.dcf`
- `config/joints.yaml` -> 每轴参数与验证开关

最小 joints.yaml 示例：
```yaml
joints:
  - name: joint_1
    counts_per_rev: 1000
    rated_torque_nm: 10
    velocity_scale: 1.0
    torque_scale: 1.0
    canopen:
      node_id: 1
      verify_pdo_mapping: true
```

### 7.2 构建

```
cmake -S /home/dianhua/robot_test -B /home/dianhua/robot_test/build
cmake --build /home/dianhua/robot_test/build -j
```

### 7.2 启动命令

```
/home/dianhua/robot_test/build/canopen_hw_node \
  --dcf /home/dianhua/robot_test/config/master.dcf \
  --joints /home/dianhua/robot_test/config/joints.yaml
```

注意：
- `--dcf` 不存在会直接退出。
- `--joints` 不存在会报警但继续运行。

---

## 8. 关机流程

程序退出时执行：
1) `Disable Operation` (0x6040=0x0007)
2) 等待 `SwitchedOn`（2s）
3) `Shutdown` (0x6040=0x0006)
4) 等待 `ReadyToSwitchOn`（1s）
5) `NMT Stop`

---

## 9. 错误处理与风险矩阵

| 场景 | 当前行为 | 备注 |
|------|----------|------|
| CAN 口未启动/不存在 | 启动失败并退出 | `CanopenMaster::Start()` 捕获异常 |
| DCF 文件不存在 | 启动前直接退出 | `main.cpp` 预检查 |
| 从站 PDO 映射不一致 | 该轴不进入 Operational | 日志输出差异明细 |
| SDO 读回超时 | 该轴验证失败 | 超时 2s |
| 心跳超时 | 标记该轴 fault，上报 SharedState | AxisLogic::ProcessHeartbeat |
| EMCY | 记录 EEC，递增健康计数 | AxisLogic::ProcessEmcy |

---

## 10. 性能与延迟预算（待补充）

- 目标：SYNC 周期 10ms，端到端反馈到应用线程延迟 < 1 周期。
- 现状：未做 profiling；建议加入时间戳采集后实测。

---

## 11. 当前限制与待办

限制：
- 尚未接入 ROS controller_manager。
- 不做 EMCY 自动复位（仅记录和计数）。
- 轴数由 `joints.yaml` 配置决定，上限 16 轴。

建议后续：
- 增加上电/OnBoot 超时与失败恢复策略。
- 将 `SharedState` 对接 ROS 控制器。
- SCHED_FIFO 需 root 或 CAP_SYS_NICE，生产部署需配置权限。

---

## 12. 版本与变更

- 2026-03-19：补齐 CSV/CST PDO 下行链路；RealtimeLoop 替代 sleep_for；抽取 BusIO/AxisLogic 提升可测试性。
- 2026-03-17：完成 PDO 映射验证 + 超时兜底 + Lely 主站真实启动流程。

---

## 13. 参考文档

- `docs/README.md`：文档索引与归档规则
- `docs/quality_9_score_plan.md`：冲 9 分执行清单
- `docs/usage.md`：运行与联调说明
- `docs/archive/canopenplan.md`：完整设计规格（历史）
- `docs/archive/pdo_sdo_design.md`：PDO/SDO 设计与验证（历史）
- `docs/archive/change_report_pdo_verify.md`：实现变更报告（历史）
