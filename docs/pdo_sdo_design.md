# PDO 映射验证与 SDO 降级控制 — 完整设计方案

---

## 一、背景与目标

CANopen 从站（电机驱动器）的 PDO 映射配置可能与主站 DCF 文件定义的期望不一致，导致 PDO 通信数据错位或失效。本方案在不自行改写 PDO 映射的前提下，利用 Lely 已有的 OnConfig 自动下发能力，增加验证环节，并在验证失败时提供 SDO 降级控制路径，保证系统在异常配置下仍可低速诊断运行。

### 设计原则

- DCF 是唯一权威配置来源
- 不自行写入 PDO 映射，由 Lely OnConfig 负责
- 验证失败不重试，直接降级
- 降级是诊断手段，不是生产替代
- 单轴故障不影响其他轴

---

## 二、Lely 已有能力与本方案边界

| 能力 | Lely 提供 | 本方案负责 |
|------|----------|-----------|
| DCF 解析与加载 | ✅ | — |
| OnConfig 自动下发 DCF 配置到从站 | ✅ | — |
| OnBoot 回调通知配置完成 | ✅ | — |
| PDO 收发 | ✅ | — |
| SDO 异步读写原语 | ✅ | — |
| 事件循环与定时器 | ✅ | — |
| 读回从站实际 PDO 映射并验证 | ❌ | ✅ |
| DCF 期望映射与实际映射对比 | ❌ | ✅ |
| 验证失败后降级到 SDO 轮询控制 | ❌ | ✅ |
| 降级模式切换 PP/PV 操作模式 | ❌ | ✅ |
| 周期性 SDO 轮询调度 | ❌ | ✅ |

前置任务：实现前必须核实当前 Lely 版本中 SDO 读写、DCF 解析、定时器的实际 API 签名。

---

## 三、配置结构

与现有 joints.yaml 结构统一，不引入新的顶层键。

```yaml
joints:
  - name: axis_1
    canopen:
      node_id: 2
      verify_pdo_mapping: true          # 是否在启动时验证 PDO 映射
      sdo_fallback:                     # 轴级降级参数（可选，覆盖全局）
        poll_interval_ms: 50

  - name: axis_2
    canopen:
      node_id: 3
      verify_pdo_mapping: true

  - name: axis_3
    canopen:
      node_id: 4
      verify_pdo_mapping: false         # 不验证，信任 Lely OnConfig

canopen:
  bus:
    interface: can0
    bitrate: 1000000
  sdo_fallback:                         # 全局降级默认参数
    poll_interval_ms: 100               # 轮询周期
    max_velocity_scale: 0.1             # 降级限速，正常最大速度的 10%
    sdo_timeout_ms: 50                  # 单次 SDO 超时
    max_consecutive_timeouts: 5         # 连续超时 N 次触发 WARNING
    fallback_mode: "pp"                 # 降级操作模式："pp" 或 "pv"
```

优先级规则：轴级 `sdo_fallback` 参数 > 全局 `canopen.sdo_fallback` 参数。

---

## 四、操作模式分析

| 模式 | 全称 | 依赖 SYNC | 依赖 PDO | SDO 降级可用 |
|------|------|----------|----------|-------------|
| CSP | 周期同步位置 | ✅ | ✅ | ❌ |
| CSV | 周期同步速度 | ✅ | ✅ | ❌ |
| CST | 周期同步力矩 | ✅ | ✅ | ❌ |
| IP | 周期插补位置 | ✅ | ✅ | ❌ |
| PP | 轮廓位置 | ❌ | ❌ | ✅ |
| PV | 轮廓速度 | ❌ | ❌ | ✅ |
| Homing | 回零 | ❌ | ❌ | ✅ |

所有周期同步类模式（CSP/CSV/CST/IP）均依赖 SYNC + PDO 配合，SDO 降级下不可用。降级模式使用 PP 或 PV。

---

## 五、运行模式层级

```
┌────────────────────────────────────────────────┐
│ 正常模式                                        │
│   PDO + CSP/CSV/CST/IP                         │
│   全速实时控制                                   │
├────────────────────────────────────────────────┤
│ 降级层级 1：SDO + PP 模式                        │
│   SDO 写 0x6060=0x01 切换 Profile Position      │
│   SDO 周期读写 controlword/statusword/position  │
│   限速运行（max_velocity_scale）                 │
│   WARNING 日志                                  │
├────────────────────────────────────────────────┤
│ 降级层级 2：SDO + PV 模式（备选）                  │
│   SDO 写 0x6060=0x03 切换 Profile Velocity      │
│   SDO 周期读写 controlword/statusword/velocity  │
│   限速运行                                      │
│   WARNING 日志                                  │
├────────────────────────────────────────────────┤
│ 降级层级 3：SDO 只读                              │
│   PP/PV 模式切换均失败                            │
│   仅读取 statusword、actual position             │
│   不控制运动                                     │
│   ERROR 日志                                    │
└────────────────────────────────────────────────┘
```

---

## 六、启动流程

```
1.  加载 joints.yaml
    ├─ 解析各轴 verify_pdo_mapping 开关
    └─ 解析 sdo_fallback 参数（全局 + 轴级合并）

2.  加载 DCF，初始化 Lely AsyncMaster

3.  从站上线，发送 boot-up 报文

4.  Lely OnConfig(id)
    └─ Lely 自动将 DCF 中全部配置（含 PDO 映射、COB-ID 等）
       通过 SDO 写入从站

5.  等待 OnBoot(id, state) 或超时
    │
    ├─ 超时未触发 OnBoot（T 秒）
    │   └─ 该轴 pdo_ok = false
    │   └─ WARNING: "轴 X: OnBoot 超时，OnConfig 可能失败"
    │
    └─ OnBoot 触发
        │
        ├─ OnConfig 本身报失败（state 异常）
        │   └─ 该轴 pdo_ok = false
        │   └─ WARNING: "轴 X: OnConfig 失败"
        │
        ├─ verify_pdo_mapping = false
        │   └─ 该轴 pdo_ok = true
        │   └─ INFO: "轴 X: 跳过 PDO 验证"
        │
        └─ verify_pdo_mapping = true
            │
            ├─ PdoMappingReader: SDO 读回从站实际配置
            │   ├─ RPDO COB-ID:  0x1400~0x1403 sub1
            │   ├─ RPDO 映射:    0x1600~0x1603 sub0~subN
            │   ├─ TPDO COB-ID:  0x1800~0x1803 sub1
            │   └─ TPDO 映射:    0x1A00~0x1A03 sub0~subN
            │
            ├─ PdoMappingDiff: 与 DCF 期望对比
            │   ├─ 比较 COB-ID
            │   ├─ 比较映射数量
            │   └─ 比较映射条目（index:subindex:bitlength）
            │
            ├─ 一致
            │   └─ 该轴 pdo_ok = true
            │   └─ INFO: "轴 X: PDO 映射验证通过"
            │
            └─ 不一致
                └─ 该轴 pdo_ok = false
                └─ WARNING: "轴 X: PDO 映射不一致"（含差异明细）

6.  各轴 AxisDriver 根据 pdo_ok 选择运行路径
    │
    ├─ pdo_ok = true
    │   └─ PDO 路径，正常模式（已有实现）
    │
    └─ pdo_ok = false
        │
        ├─ 尝试 SDO 写 0x6060 切换到 PP 模式
        │   ├─ 成功 → 降级层级 1（SDO + PP 控制）
        │   └─ 失败 →
        │       ├─ 尝试切换 PV 模式
        │       │   ├─ 成功 → 降级层级 2（SDO + PV 控制）
        │       │   └─ 失败 → 降级层级 3（SDO 只读）
        │
        └─ WARNING/ERROR 日志

7.  发送 NMT Start，进入 Operational
```

---

## 七、验证范围

### 读取内容

| 对象 | 子索引 | 含义 | 用途 |
|------|--------|------|------|
| 0x1400~0x1403 | sub 1 | RPDO COB-ID | COB-ID 校验 |
| 0x1600~0x1603 | sub 0 | RPDO 映射数量 | 数量对比 |
| 0x1600~0x1603 | sub 1~N | RPDO 映射条目 | 条目对比 |
| 0x1800~0x1803 | sub 1 | TPDO COB-ID | COB-ID 校验 |
| 0x1A00~0x1A03 | sub 0 | TPDO 映射数量 | 数量对比 |
| 0x1A00~0x1A03 | sub 1~N | TPDO 映射条目 | 条目对比 |

### 对比规则

| 对比项 | 是否比较 |
|--------|---------|
| COB-ID（0x1400/0x1800 sub1） | ✅ 比较 |
| 映射数量（0x1600/0x1A00 sub0） | ✅ 比较 |
| 映射条目（index:subindex:bitlength） | ✅ 比较 |
| 传输类型 | ❌ 不比较 |
| Inhibit time | ❌ 不比较 |
| Event timer | ❌ 不比较 |

### 判定逻辑

- 任何一个 PDO 通道的 COB-ID 不一致 → 失败
- 任何一个 PDO 通道的映射数量不一致 → 失败
- 任何一个映射条目的三元组不一致 → 失败
- 任何一次 SDO 读取超时或报错 → 整体失败
- 全部通道全部条目一致 → 通过

---

## 八、SDO 降级控制

### PP 模式控制序列

```
初始化阶段（一次性）：
  1. SDO 写 0x6060:00 = 0x01         ← 切换到 PP 模式
  2. SDO 读 0x6061:00                ← 确认模式已切换
  3. SDO 写 0x6081:00 = 限速值        ← Profile Velocity（限速）
  4. SDO 写 0x6083:00 = 加速度        ← Profile Acceleration
  5. SDO 写 0x6084:00 = 减速度        ← Profile Deceleration
  6. CiA 402 状态机使能（0x6040 序列）

周期轮询阶段（每 poll_interval_ms 执行一次）：
  1. SDO 读 0x6041:00                ← Statusword
  2. SDO 读 0x6064:00                ← Position Actual Value
  3. SDO 写 0x607A:00 = 目标位置      ← Target Position（如有新目标）
  4. SDO 写 0x6040:00 = controlword  ← 含 new setpoint bit
  5. 更新 SharedState
```

### PV 模式控制序列

```
初始化阶段（一次性）：
  1. SDO 写 0x6060:00 = 0x03         ← 切换到 PV 模式
  2. SDO 读 0x6061:00                ← 确认模式已切换
  3. SDO 写 0x6083:00 = 加速度
  4. SDO 写 0x6084:00 = 减速度
  5. CiA 402 状态机使能

周期轮询阶段：
  1. SDO 读 0x6041:00                ← Statusword
  2. SDO 读 0x6064:00                ← Position Actual Value
  3. SDO 写 0x60FF:00 = 目标速度      ← Target Velocity（限速后）
  4. SDO 写 0x6040:00 = controlword
  5. 更新 SharedState
```

### 多轴调度

多轴降级时，轮询交错执行以避免总线拥塞：

```
轴间偏移 = poll_interval_ms / 降级轴数

示例：3 个轴降级，poll_interval_ms = 100ms
  轴1: t=0,   100, 200, 300 ...
  轴2: t=33,  133, 233, 333 ...
  轴3: t=66,  166, 266, 366 ...
```

非硬实时保证，尽力而为。

### 超时处理

- 单次 SDO 超时：静默忽略，跳过本轮
- 连续 N 次超时（max_consecutive_timeouts）：WARNING 日志
- 连续超时不触发急停，仅告警
- 超时计数在成功通信后重置

---

## 九、模块设计

### 9.1 PdoMappingReader

- 职责：通过 SDO 读回一个从站的全部 RPDO/TPDO 的 COB-ID 与映射
- 输入：从站 node_id
- 输出：该从站完整的 PDO 配置数据结构
- 读取范围：
  - RPDO：0x1400~0x1403 sub1（COB-ID），0x1600~0x1603 sub0~subN（映射）
  - TPDO：0x1800~0x1803 sub1（COB-ID），0x1A00~0x1A03 sub0~subN（映射）
- 错误处理：任何一次 SDO 读取失败 → 返回失败状态
- 依赖：Lely SDO 异步读

### 9.2 PdoMappingDiff

- 职责：对比 DCF 期望配置与 Reader 读回的实际配置
- 输入：DCF 期望映射 + Reader 读回的实际映射
- 输出：是否一致（bool） + 差异明细列表
- 比较规则：见第七节
- 依赖：Lely DCF 解析 API（从 co_dev_t 中提取期望值）

### 9.3 SdoFallbackDriver

- 职责：PDO 不可用时，通过 SDO 轮询实现基本控制
- 触发条件：该轴 pdo_ok = false
- 生命周期：启动时确定，运行期间不切换回 PDO
- 行为：
  - 初始化：切换操作模式（PP 或 PV）、设置限速参数、使能状态机
  - 周期轮询：按配置的 poll_interval_ms 执行 SDO 读写
  - 状态更新：读回数据写入 SharedState，与 PDO 路径共用同一接口
- 降级层级切换：PP 失败尝试 PV，PV 失败退化为只读
- 依赖：Lely SDO 异步读写 + Lely 定时器

---

## 十、日志规范

```
[INFO]  轴1 (node 2): OnConfig 完成，开始 PDO 映射验证
[INFO]  轴1 (node 2): RPDO0 COB-ID 一致: 0x00000202
[INFO]  轴1 (node 2): RPDO0 映射一致 (2 entries)
[INFO]  轴1 (node 2): TPDO0 COB-ID 一致: 0x00000182
[INFO]  轴1 (node 2): TPDO0 映射一致 (2 entries)
[INFO]  轴1 (node 2): PDO 映射验证通过

[WARN]  轴2 (node 3): TPDO1 映射不一致
          期望: [6041:00/16, 6064:00/32]
          实际: [6041:00/16, 606C:00/32]
[WARN]  轴2 (node 3): PDO 验证失败，进入 SDO 降级模式
[INFO]  轴2 (node 3): 切换到 PP 模式成功
[WARN]  轴2 (node 3): 运行在 SDO 降级模式（非实时，限速 10%）

[INFO]  轴3 (node 4): verify_pdo_mapping=false，跳过验证

[ERROR] 轴4 (node 5): OnBoot 超时（5s），OnConfig 可能失败
[ERROR] 轴4 (node 5): PP/PV 模式切换均失败，进入只读模式
```

---

## 十一、明确不做的事情

1. 不自己写入 PDO 映射（Lely OnConfig 负责）
2. 不重试 OnConfig
3. 不重启从站
4. 不区分失败原因（超时/拒绝/不支持统一降级）
5. 不因单轴降级影响其他轴
6. 不改 COB-ID、传输类型、inhibit time、event timer
7. 不在运行期间从 SDO 切换回 PDO
8. 不保证 SDO 降级模式的实时性

---

## 十二、已知风险与约束

| 编号 | 风险 | 应对 |
|------|------|------|
| R1 | SDO 降级模式不是 PDO 的生产替代 | 文档明确：仅用于诊断和低速验证。生产环境 PDO 失败应停机排查 |
| R2 | SDO 轮询受总线负载影响，不保证时效性 | 单次超时静默忽略，连续超时告警，不触发急停 |
| R3 | 部分驱动器可能 PP/PV 模式切换也失败 | 退化为只读模式，ERROR 日志 |
| R4 | COB-ID 正确但映射错位 → 验证捕获；映射正确但 COB-ID 错 → 验证也捕获 | 两者都比较，已覆盖 |
| R5 | OnConfig 失败后 OnBoot 可能不触发 | 超时兜底机制 |
| R6 | 多轴交错调度依赖定时器精度 | 非 RT 系统下尽力而为，文档注明 |
| R7 | 验证环节增加启动时间 | 每轴约数百毫秒 SDO 交互，可接受 |
| R8 | 从站不支持 SDO 读取 PDO 参数 | 验证本身失败即触发降级，由用户保证从站能力 |
| R9 | Lely API 签名可能与设计中的名称不一致 | 实现前第一步核实 |

---

## 十三、实现任务清单

| 序号 | 任务 | 输入 | 输出 | 依赖 |
|------|------|------|------|------|
| 0 | 核实 Lely 版本 API 签名 | /usr/include/lely 头文件 | API 确认文档 | 无，最先做 |
| 1 | PdoMappingReader | Lely SDO 异步读 | 从站实际 PDO 配置数据 | 任务 0 |
| 2 | PdoMappingDiff | DCF 期望 + Reader 输出 | 一致性判定 + 差异明细 | 任务 0、1 |
| 3 | OnBoot 接入验证流程 + 超时兜底 | Reader + Diff | 各轴 pdo_ok 标志 | 任务 1、2 |
| 4 | SdoFallbackDriver（PP/PV 模式 + 周期轮询） | Lely SDO 读写 + 定时器 | 降级控制路径 | 任务 0 |
| 5 | AxisDriver 三路径分支 | pdo_ok 标志 | PDO / SDO+PP控制 / SDO只读 | 任务 3、4 |
| 6 | joints.yaml 配置解析扩展 | 现有 yaml 解析 | verify_pdo_mapping + sdo_fallback 参数 | 无 |
| 7 | 日志输出 | 已有日志框架 | 格式化日志 | 无 |

推荐实现顺序：`0 → 6 → 1 → 2 → 3 → 4 → 5 → 7`

---

## 十四、验收标准

1. verify_pdo_mapping=false 时，不触发任何额外 SDO 读取，行为与现有系统完全一致
2. verify_pdo_mapping=true 且 PDO 配置一致时，正常进入 PDO 模式，启动时间增加不超过 2 秒
3. PDO 配置不一致时，日志输出具体差异项，自动切换到 SDO+PP 降级模式
4. 降级模式下能完成低速点动验证
5. PP/PV 切换失败时退化为只读，ERROR 日志
6. 单轴降级不影响其他轴的 PDO 正常运行
7. 轴级 sdo_fallback 参数能正确覆盖全局默认值

