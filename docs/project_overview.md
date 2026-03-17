# 项目文档：CANopen 主站控制栈

日期：2026-03-17  
范围：当前仓库代码与文档（含 PDO 映射验证）

---

## 1. 项目简介

本项目实现面向 6 轴谐波关节的 CANopen 主站控制栈，基于 Lely-core 与 CSP 模式。系统由三层核心模块组成：

- CANopen 主站层：负责 Lely I/O 初始化、AsyncMaster 事件循环与从站驱动管理。
- 轴驱动层：每轴一个驱动，接入 CiA402 状态机与 PDO/SDO 交互。
- 上层接口层：以 `SharedState` 作为线程间共享数据面，连接控制算法或 ROS 侧。

当前版本为独立可运行程序（`canopen_hw_node`），未接入 ROS。主循环以 100Hz 读写 `SharedState`，与 Lely 事件线程并行运行。

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
    axis_driver.hpp           # 单轴驱动，回调接入、状态机入口
    canopen_master.hpp        # 主站与 Lely 生命周期管理
    canopen_robot_hw.hpp      # 机器人硬件抽象层（读写 SharedState）
    cia402_state_machine.hpp  # CiA402 状态机
    joints_config.hpp         # joints.yaml 解析
    pdo_mapping.hpp           # PDO 读取/比对
    shared_state.hpp          # 共享状态
src/
  axis_driver.cpp
  canopen_master.cpp
  canopen_robot_hw.cpp
  cia402_state_machine.cpp
  joints_config.cpp
  pdo_mapping.cpp
  shared_state.cpp
  main.cpp
config/
  joints.yaml
  master.yaml
  master.dcf
  *.eds
_docs/
  usage.md
  canopenplan.md
  pdo_sdo_design.md
  change_report_pdo_verify.md
  review_report_and_fix_plan.md
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

### 4.2 AxisDriver

职责：单轴驱动，继承 `lely::canopen::BasicDriver`，处理回调并驱动 CiA402 状态机。

关键功能：
- `OnRpdoWrite()`：读取 PDO 映射对象，推进状态机。
- `OnBoot()`：触发 PDO 映射验证；失败则阻断该轴 Operational。
- `InjectFeedback()`：统一反馈入口。
- `SendControlword()`：下发控制字。

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

- SYNC 由主站通过 `ev::Loop` 定时器产生（见 `docs/canopenplan.md`）。
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
| 心跳超时 | 未处理 | TODO（需上报 SharedState） |
| EMCY | 仅占位 | TODO（需日志/映射） |

---

## 10. 性能与延迟预算（待补充）

- 目标：SYNC 周期 10ms，端到端反馈到应用线程延迟 < 1 周期。
- 现状：未做 profiling；建议加入时间戳采集后实测。

---

## 11. 当前限制与待办

限制：
- 尚未接入 ROS controller_manager。
- 不做 EMCY 映射与自动复位。
- 仅支持固定 6 轴配置（由 `axis_count` 决定）。

建议后续：
- 增加上电/OnBoot 超时与失败恢复策略。
- 增加对心跳超时的上报。
- 将 `SharedState` 对接 ROS 控制器。

---

## 12. 版本与变更

- 2026-03-17：完成 PDO 映射验证 + 超时兜底 + Lely 主站真实启动流程。

---

## 13. 参考文档

- `docs/usage.md`：运行与联调说明
- `docs/canopenplan.md`：完整设计规格
- `docs/pdo_sdo_design.md`：PDO/SDO 设计与验证
- `docs/change_report_pdo_verify.md`：实现变更报告
