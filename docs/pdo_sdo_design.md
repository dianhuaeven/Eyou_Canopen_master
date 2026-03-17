# 完整设计方案：PDO 映射验证与 SDO 降级

本文档定义“以 DCF 为权威、PDO 映射验证、失败则降级 SDO 轮询”的完整设计方案。

---

## Lely 的 SDO 能力

**Lely 提供 SDO 读写原语，不提供轮询模式封装。**

| 能力 | Lely 提供 |
|---|---|
| SDO 异步读 `AsyncRead<T>(id, idx, sub, callback)` | ✅ 有 |
| SDO 异步写 `AsyncWrite<T>(id, idx, sub, val, callback)` | ✅ 有 |
| SDO 超时处理 | ✅ 有 |
| 周期性轮询调度 | ❌ 没有 |
| 降级模式切换 | ❌ 没有 |

系统需要基于 `AsyncRead / AsyncWrite` 构建周期性轮询循环，Lely 本身不处理。

---

## 完整方案

### 一、系统边界

```
┌──────────────────────────────────────────────────────┐
│                    你的系统                            │
│                                                      │
│  joints.yaml ──► 启动逻辑 ──► AxisDriver(per axis)   │
│                                  │                   │
│                          ┌───────┴───────┐           │
│                        PDO 路径      SDO 降级路径     │
│                       (已有实现)     (需要新建)        │
│                                                      │
├──────────────────────────────────────────────────────┤
│                    Lely 提供                          │
│                                                      │
│  DCF 解析 / OnConfig 自动下发 / OnBoot 回调            │
│  AsyncRead / AsyncWrite / PDO 收发                    │
│  事件循环 / 定时器                                     │
└──────────────────────────────────────────────────────┘
```

### 二、配置

```yaml
# joints.yaml
axis_1:
  node_id: 2
  verify_pdo_mapping: true

axis_2:
  node_id: 3
  verify_pdo_mapping: false

sdo_fallback:
  poll_interval_ms: 100
  max_velocity_scale: 0.1
```

### 三、启动流程

```
1. 加载 joints.yaml
2. 加载 DCF，初始化 Lely AsyncMaster
3. 从站上线
4. Lely OnConfig(id)
   └─ Lely 自动将 DCF 中全部配置（含 PDO 映射）通过 SDO 写入从站
5. Lely OnBoot(id, state)
   ├─ OnConfig 本身报失败 → 该轴 pdo_ok=false，WARNING
   ├─ verify_pdo_mapping=false → 该轴 pdo_ok=true，跳过
   └─ verify_pdo_mapping=true →
        a. PdoMappingReader：SDO 读回 0x1600/0x1A00 系列
        b. PdoMappingDiff：与 DCF 期望对比
        c. 一致 → pdo_ok=true
        d. 不一致 → pdo_ok=false，WARNING（含差异明细）
6. 该轴 AxisDriver 根据 pdo_ok 选择运行路径
   ├─ true  → PDO 路径（已有实现）
   └─ false → SdoFallbackDriver（新建）
7. 发送 NMT Start，进入 Operational
```

### 四、模块设计

#### 4.1 PdoMappingReader

- 职责：通过 SDO 读回一个从站的全部 RPDO/TPDO 映射
- 输入：从站 `node_id`
- 输出：映射列表（每个 PDO 通道的 `index:subindex:bitlength` 数组）
- 读取范围：
  - RPDO 映射：`0x1600~0x1603` 的 `sub0~subN`
  - TPDO 映射：`0x1A00~0x1A03` 的 `sub0~subN`
- 错误处理：任何一次 SDO 读取超时或失败 → 整体视为失败
- 依赖：Lely `AsyncRead`

#### 4.2 PdoMappingDiff

- 职责：对比 DCF 期望映射与实际读回映射
- 输入：DCF 期望映射 + Reader 读回的实际映射
- 输出：`bool` 是否一致 + 差异明细列表
- 比较规则：
  - 逐 PDO 通道比较
  - 逐条目比较 `index:subindex:bitlength` 三元组
  - 数量不同视为不一致
  - 不比较 COB-ID、传输类型、inhibit time 等
- 依赖：Lely DCF 解析 API（`co_dev_find_obj / co_obj_get_val`）

#### 4.3 SdoFallbackDriver

- 职责：当 PDO 不可用时，通过 SDO 轮询实现基本控制
- 触发条件：该轴 `pdo_ok = false`
- 生命周期：启动时确定，运行期间不切换回 PDO
- 行为：
  - 周期性 SDO 读取：`statusword (0x6041)`、`position actual (0x6064)`
  - 按需 SDO 写入：`controlword (0x6040)`、`target position (0x607A)`
  - 轮询间隔：`100ms`（可配置）
  - 多轴交错调度：轴间偏移 = `poll_interval / 轴数`
  - 限速：`max_velocity_scale` 约束
- 状态更新：读回数据写入 `SharedState`，与 PDO 路径相同的接口
- 依赖：Lely `AsyncRead / AsyncWrite / 定时器`

### 五、日志规范

```
[INFO]  轴2: OnConfig 完成，开始 PDO 映射验证
[INFO]  轴2: TPDO0 映射一致 (2 entries)
[WARN]  轴2: TPDO1 映射不一致
          期望: [6041:00/16, 6064:00/32]
          实际: [6041:00/16, 606C:00/32]
[WARN]  轴2: PDO 验证失败，进入 SDO 降级模式（非实时）
[INFO]  轴3: verify_pdo_mapping=false，跳过验证
```

### 六、不做的事情

- 不自己写 PDO 映射
- 不重试 OnConfig
- 不重启从站
- 不区分失败原因
- 不因单轴降级影响其他轴
- 不改 COB-ID / 传输类型
- 不在运行期间从 SDO 切换回 PDO

### 七、实现清单

| 序号 | 任务 | 依赖 |
|---|---|---|
| 1 | PdoMappingReader | Lely AsyncRead |
| 2 | PdoMappingDiff | Lely DCF 解析 + Reader 输出 |
| 3 | SdoFallbackDriver | Lely AsyncRead/AsyncWrite + 定时器 |
| 4 | OnBoot 中接入验证流程 | Reader + Diff |
| 5 | AxisDriver 分支：根据 pdo_ok 选路径 | SdoFallbackDriver |
| 6 | joints.yaml 新增配置项解析 | 已有 yaml 解析 |
| 7 | 日志输出 | 已有日志框架 |

建议实现顺序：`1 → 2 → 4 → 3 → 5 → 6 → 7`。

