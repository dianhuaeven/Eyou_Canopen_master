# CANopen 主站代码评审报告与修复提交路线

版本: v0.1  
日期: 2026-03-17

## 1. 结论摘要

本次外部评审结论整体可信，方向与项目现状一致。当前代码属于“骨架完整、闭环未打通”阶段：

- 架构分层正确（状态机纯逻辑、共享状态清晰、测试可独立运行）
- 关键功能尚未落地（Lely 启动、PDO 接入、全轴运行态判定、配置加载、优雅关机）

综合结论：**可以继续按现有设计推进，但必须尽快完成控制闭环主路径。**

## 2. 评审意见采纳情况

### 2.1 完全采纳

1. `CMakeLists.txt` 与当前源码结构不匹配（严重）
2. `OnRpdoWrite()` 未接入反馈字段（功能缺失）
3. `all_operational` 未实现周期计算（逻辑缺口）
4. `CanopenMaster::Start()` 未完成真实 Lely 初始化（功能缺失）
5. `joints.yaml` 未加载（配置未接入）
6. `master_dcf_path` 相对路径风险（可部署性问题）
7. 关机流程未按规格书 8.3 落地（安全缺口）

### 2.2 部分采纳（需澄清）

1. `ResetFaultFlowContext()` 条件是否为硬 bug：
   - 当前实现将 `fault_reset_count` 定义为“复位尝试次数”，在 `SendEdge` 计数
   - 该实现并非必然错误，但可读性与语义存在争议
   - 处理策略：修复时同步重构复位计数语义并补测试，避免歧义

2. `DecodeState()` 掩码问题：
   - 逻辑判定可工作
   - 主要问题是常量命名重复导致理解成本高
   - 处理策略：整理掩码命名并补注释，不作为单独阻塞项

## 3. 当前风险分级

- P0（必须优先修）
  - PDO 未接入（无法闭环）
  - Master 未真实启动（无总线控制能力）
- P1（高优先）
  - `all_operational` 未计算（上层控制无法进入正常写入）
  - CMake 目标错误（工程构建不稳定）
  - 关机流程未落实（安全收敛不足）
- P2（中优先）
  - joints.yaml 未加载（参数未工程化）
  - dcf/路径依赖工作目录（部署一致性不足）
  - 状态机常量命名可读性问题

## 4. 修复总原则

1. 保持“初始化可分配、运行循环零动态分配”约束
2. 每个 commit 只做一件可验证的小事
3. 每个 commit 提交前必须完成完整编译回归
4. 高风险改动必须配套最小单元测试或集成验证点

## 5. 小 Commit 修复路线

### Commit A: build: fix cmake target layout

目标:
- 修正 `CMakeLists.txt`，显式编译 `src/*.cpp`
- 增加测试目标（至少 state_machine/shared_state/unit_conversion）

验收:
- `cmake && make` 可生成主程序与测试

### Commit B: core: clarify cia402 reset semantics and masks

目标:
- 明确 `fault_reset_count` 语义（尝试次数/成功次数二选一并统一）
- 调整 `ResetFaultFlowContext()` 触发条件与注释
- 整理重复掩码命名，提升可读性

验收:
- `test_state_machine` 覆盖 FaultReaction->Fault->Reset 场景

### Commit C: canopen: implement lely master runtime bootstrap

目标:
- 在 `CanopenMaster::Start()` 中创建并启动真实 Lely 组件
- 创建 master 与 6 轴 driver，进入事件循环线程

验收:
- 程序启动后可观测到总线心跳/基础事件（至少无崩溃）

### Commit D: canopen: wire rpdo decode into axis state machine

目标:
- 在 `AxisDriver::OnRpdoWrite()` 读取关键对象字段
  - 0x6041, 0x6064, 0x6061, 0x606C, 0x6077
- 调用状态机推进并更新 SharedState

验收:
- 反馈路径可驱动状态机状态变化

### Commit E: canopen: compute all_operational each cycle

目标:
- 在 Lely 周期中汇总各轴 `is_operational && !position_locked`
- 写入 `SharedState::SetAllOperational()`

验收:
- `CanopenRobotHw::WriteToSharedState()` 仅在全轴就绪时写入命令

### Commit F: config: load joints.yaml into runtime conversions

目标:
- 增加配置加载模块，解析 `joints.yaml`
- 将 `counts_per_rev/rated_torque_nm/scale` 注入 `CanopenRobotHw`

验收:
- 单测覆盖每轴不同参数换算

### Commit G: app: harden dcf path and startup args

目标:
- `master_dcf_path` 改为参数化绝对路径或启动参数注入
- 启动时增加配置有效性检查与失败日志

验收:
- 从任意工作目录启动均可正确定位 DCF

### Commit H: safety: implement graceful shutdown sequence

目标:
- 实现 Disable Operation -> Shutdown -> NMT Stop 顺序
- 带超时等待与失败处理

验收:
- `SIGINT` 下可观察到有序停机流程

### Commit I: docs: update debug and bringup playbook

目标:
- 更新调试文档中的新命令、验证点、故障树
- 固化 D1~D9 实验记录模板

验收:
- 文档可直接用于上机联调

## 6. 执行顺序

建议顺序：`A -> B -> C -> D -> E -> F -> G -> H -> I`

其中 `C/D/E` 为主链路阻塞项，优先级最高。

## 7. 完成标准

满足以下条件视为“闭环完成”:

1. 主程序可稳定启动 Lely 主站并接收 6 轴反馈
2. 状态机可将轴推进到 `OPERATION_ENABLED`
3. `all_operational=true` 后上层命令可写入并被从站执行
4. `SIGINT` 可触发优雅关机序列
5. 单测全绿，且关键路径具备基础联调记录

