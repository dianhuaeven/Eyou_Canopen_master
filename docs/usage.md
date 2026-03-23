# CANopen 主站使用文档

本文档面向上机联调与日常使用，覆盖：环境准备、配置、生成 DCF、编译运行、验证点、关机与故障排查。

---

## 1. 环境准备

### 1.1 依赖工具

```bash
sudo apt install can-utils
```

### 1.2 启动 CAN 接口

```bash
sudo ip link set can0 up type can bitrate 1000000
ip -details -statistics link show can0
```

### 1.3 抓包检查（建议常开）

```bash
# 实时抓包
candump can0

# 仅看关键帧（SYNC + PDO + Heartbeat + EMCY）
candump can0,080:7FF,180:7FF,200:7FF,280:7FF,700:7FF
```

预期：
- SYNC 周期约 10ms
- Node 1-6 心跳帧可见（0x701~0x706）
- 运行后可见 PDO 帧（0x18x/0x28x/0x20x）

---

## 2. 配置文件

### 2.1 joints.yaml

路径：`config/joints.yaml`

重点字段（每轴）：
- `counts_per_rev`
- `rated_torque_nm`
- `velocity_scale`
- `torque_scale`
 - `canopen.verify_pdo_mapping`（启动时是否验证 PDO 映射）

建议每轴都填完整参数，避免默认值掩盖问题。

### 2.2 master.yaml / master.dcf

路径：
- `config/master.yaml`
- `config/master.dcf`

`master.dcf` 由 `dcfgen` 根据 `master.yaml` 生成，且在启动时必须存在。

---

## 3. 生成 DCF

```bash
dcfgen -S -r -d /home/dianhua/robot_test/config \
       /home/dianhua/robot_test/config/master.yaml
```

说明：
- 若 `dcfgen` 报 EDS 格式错误，请改用 `YiyouServo_V1.4.dcfgen.eds`。

---

## 4. 编译

### 4.1 独立编译（不依赖 ROS）

```bash
cmake -S . -B build
cmake --build build -j
```

### 4.2 catkin 编译（ROS 模式）

```bash
cd ~/Robot24_catkin_ws
catkin_make --pkg Eyou_Canopen_Master
source devel/setup.bash
```

---

## 5. 启动

### 5.1 独立模式

```bash
./build/canopen_hw_node \
  --dcf config/master.dcf \
  --joints config/joints.yaml
```

说明：
- `--dcf` 必须指向存在的 DCF 文件，否则程序直接退出。
- `--joints` 不存在时只报警，但仍可运行。
- 路径建议使用绝对路径，避免工作目录变化导致失败。

### 5.2 ROS 模式

```bash
# 默认手动初始化（推荐）
roslaunch Eyou_Canopen_Master canopen_hw.launch auto_init:=false

# 首次启动必须手动初始化电机
rosservice call /canopen_hw_node/init "{}"
```

可选参数：
- `dcf_path:=<path>` — DCF 文件路径
- `joints_path:=<path>` — joints.yaml 路径
- `loop_hz:=100.0` — 控制循环频率
- `auto_init:=false|true` — 是否启动后自动初始化电机（默认 `false`）

---

## 6. ROS 接口

### 6.1 Services

| Service | 类型 | 说明 |
|---------|------|------|
| `~init` | `std_srvs/Trigger` | 手动初始化电机（Configured -> Active） |
| `~halt` | `std_srvs/Trigger` | 轻量停转：置 Halt bit，保持 Active 与通信 |
| `~resume` | `std_srvs/Trigger` | 清 Halt bit 恢复运动；若有故障需先 `~recover` |
| `~recover` | `std_srvs/Trigger` | 仅做 Fault Reset（Active 内），不重启通信 |
| `~shutdown` | `std_srvs/Trigger` | 执行 402 失能 + 停通信，不退出节点（随后需 `~init`） |
| `~set_mode` | `Eyou_Canopen_Master/SetMode` | 允许在 `Configured` 或 `Active+halted` 切模式 |

### 6.1.2 命令协议（epoch-ready）

自 2026-03-22 起，控制链路采用 `AxisCommand.valid + AxisCommand.arm_epoch` 协议：

1. `arm_epoch=0` 永远无效；
2. 进入新的使能会话后，底层会更新 `AxisFeedback.arm_epoch`；
3. 上层仅在命令源重同步完成后才应将 `valid=true`；
4. `valid=true` 但 `arm_epoch` 不匹配时，状态机仍保持锁定，不会透传目标。

工程建议：

1. 在 `/init`、`/recover`、`/resume` 后先等待反馈 epoch 稳定；
2. 重同步 controller setpoint 到当前位置；
3. 再把 `valid` 置 true。

### 6.1.1 shutdown/recover/init 关系

- `~shutdown`：执行 402 失能与停通信，节点进程不退出，并设置 `require_init=true`。
- `~recover`：仅处理 Fault，不重启主站；若 `require_init=true` 会被拒绝。
- `~init`：`~shutdown` 后唯一重新建立通信的入口。
- `~halt` / `~resume`：仅影响控制字 Halt bit，不改变 lifecycle 到 Configured。
- 当 `global_fault` 闩锁为 true 时，`~resume` 会被拒绝；必须先 `~recover`。

### 6.2 模式切换流程

```bash
# 0. 首次启动先初始化电机（若 auto_init:=false）
rosservice call /canopen_hw_node/init "{}"

# 1. 轻量停转（保持 Active，不断通信）
rosservice call /canopen_hw_node/halt

# 2. 切换到 CSV 模式（所有轴）
for i in 0 1 2 3 4 5; do
  rosservice call /canopen_hw_node/set_mode "{axis_index: $i, mode: 9}"
done

# 3. 切换 controller
rosrun controller_manager controller_manager stop arm_position_controller
rosrun controller_manager controller_manager start arm_velocity_controller

# 4. 恢复运动
rosservice call /canopen_hw_node/resume
```

切换到 IP 模式（mode=7）：

```bash
# 1) 停转
rosservice call /canopen_hw_node/halt "{}"

# 2) 切到 IP
for i in 0 1 2 3 4 5; do
  rosservice call /canopen_hw_node/set_mode "{axis_index: $i, mode: 7}"
done

# 3) 恢复
rosservice call /canopen_hw_node/resume "{}"
```

说明：
- IP 模式位置目标优先走 `0x60C1:01`（RPDO2 映射）。  
- 若驱动器拒绝 `0x60C1:01` PDO 写入，运行时会回退到 `0x607A`，并打印一次告警日志。  
- `0x60C2:01`（插补周期，ms）在 boot 时通过 SDO 下发，取自 `joints.yaml` 的 `ip_interpolation_period_ms`（未配置时按 `loop_hz` 推导）。

### 6.3 Controllers

| Controller | 类型 | 模式 | 默认状态 |
|-----------|------|------|---------|
| `joint_state_controller` | JointStateController | — | 启动 |
| `arm_position_controller` | position_controllers/JointTrajectoryController | CSP | 启动 |
| `arm_velocity_controller` | velocity_controllers/JointTrajectoryController | CSV | 已加载未启动 |

### 6.4 Diagnostics

`/diagnostics` topic 每轴上报：
- `heartbeat_lost` — 心跳丢失计数
- `emcy_count` — EMCY 计数
- `fault_reset_attempts` — 故障复位尝试次数
- `is_operational` / `is_fault` / `heartbeat_lost_flag`

---

## 7. 运行期验证点

基础验证：
- 主站启动无报错
- 心跳可见
- PDO 正常刷新

状态机验证：
- 状态能够进入 `OPERATION_ENABLED`
- 位置反馈更新后 `all_operational` 变为 true
- 在 `valid=false` 或 `epoch` 失配时，轴保持锁定且 `safe_target` 跟随实际位置
- `all_axes_halted_by_fault=true` 时，上层不应继续发送有效运动命令

---

## 8. 关机与停通信流程

### 8.1 `~shutdown`（节点不退出）

`~shutdown` 被调用时按以下顺序执行：
1. `Disable Operation` (0x6040=0x0007)
2. 等待状态进入 `SwitchedOn`（超时 2s）
3. `Shutdown` (0x6040=0x0006)
4. 等待状态进入 `ReadyToSwitchOn`（超时 1s）
5. `NMT Stop` + `master_->Stop()` 停通信
6. 标记为“需要重新 init”，随后 `~recover` 会拒绝并提示先 `~init`

若第 2/4 步超时：
- 仍继续执行第 5 步，避免 service 卡死
- service 返回 `success=false`，`message` 给出超时轴信息
- 仍标记为“需要重新 init”

### 8.2 Ctrl+C（进程级 teardown）

Ctrl+C 才会触发进程级退出，释放 ROS 节点与内部对象。

---

## 9. 启动身份失配诊断

当启动日志出现 `OnConfig failed (es=...)` 时，驱动会额外输出以下诊断信息：

- `expected identity from master.dcf`：主站 DCF 对该节点期望的 `1000:00 / 1018:01 / 1018:02 / 1018:03`
- `actual identity snapshot`：启动失败当下通过 SDO 读回的实际值
- `boot identity mismatch fields`：直接列出不一致字段（如 `1000:00(device_type)`, `1018:02(product_code)`）

建议排查顺序：
1. 对比 `boot identity mismatch fields` 与驱动器实测对象字典。
2. 若字段不一致，先修正 EDS/DCF（尤其 `1F84/1F85/1F86`）再重新 `dcfgen`。
3. 若字段一致但仍失败，继续排查 PDO 映射或上电时序问题。

---

## 10. 现场验证清单（最终版）

> 目标：现场按此清单一次性完成“启动可用 + 抓包可证 + 关机可控”的验收。

### 10.1 前置准备

1. CAN 链路已连通，`can0` 已 up，波特率与驱动器一致。
2. `master.dcf` 与现场固件版本匹配（更新固件后必须重生成 DCF）。
3. 启动抓包：

```bash
# Node=5 示例：SYNC + TPDO/RPDO + Heartbeat + NMT
candump -L can0,080:7FF,185:7FF,205:7FF,285:7FF,705:7FF,000:7FF
```

### 10.2 启动与 Boot 身份判据

启动后检查日志：

- 通过：无 `OnConfig failed`；或出现后自动恢复且最终进入运行态。
- 失败：持续出现 `OnConfig failed` 且无法进入运行态。

若失败，必须核对 C03 诊断日志：

- `expected identity from master.dcf`
- `actual identity snapshot`
- `boot identity mismatch fields`

字段判据（必须一致）：

- `1000:00`（Device Type）
- `1018:01`（Vendor ID）
- `1018:02`（Product Code）

### 10.3 PDO 抓包判据（C01/C02 关键）

1. 使能阶段（C01）：在 `0x205` 中可看到控制字低字节序列 `06 00 -> 0F 00`（little-endian，对应 0x0006 -> 0x000F）。
2. 运行阶段：`0x185` 反馈应持续周期出现（无长时间稀疏/中断）。
3. 退出阶段（C02）：Ctrl+C 后应看到控制字回落序列 `07 00 -> 06 00`，随后出现 NMT Stop（`0x000`）。

### 10.4 运行态判据

- `all_operational` 最终为 `true`。
- `/diagnostics` 中各轴 `is_operational=true`，且无持续 `is_fault=true`。
- 无持续 heartbeat 丢失累计增长。

### 10.5 启动负路径判据（C04）

- 缺失 `--dcf` 文件：进程应直接失败退出（返回码 1）。
- 缺失 `--joints` 文件：进程应直接失败退出（返回码 1）。

### 10.6 验收结论模板

| 检查项 | 结果 | 证据 |
|---|---|---|
| Boot 身份一致（1000/1018） | PASS/FAIL | 启动日志片段 |
| 控制字使能序列（06 00 -> 0F 00） | PASS/FAIL | candump 片段 |
| 运行期 TPDO 连续性（0x185） | PASS/FAIL | candump 片段 |
| 关机序列（07 00 -> 06 00 + NMT Stop） | PASS/FAIL | Ctrl+C 后抓包 |
| all_operational 与 diagnostics | PASS/FAIL | topic/日志截图 |

---

## 11. 单测

```bash
# 独立编译模式
cmake --build build -j && cd build && ctest --output-on-failure

# catkin 模式
catkin_make run_tests_Eyou_Canopen_Master
```

---

## 12. 常见问题

1. 无任何帧：优先检查终端电阻、电源、CAN_H/CAN_L 线序。
2. 有心跳无 PDO：确认 NMT 已进入 Operational，检查 PDO COB-ID 是否被禁用(bit31)。
3. 反复 FAULT：检查 statusword 与 EMCY 码，确认复位节流参数是否过短。
4. 退出后行为异常：做“断 SYNC/强杀进程”实验并确认驱动器策略。

---

## 13. D1~D9 联调记录

请参见 [docs/debug_notes.md](/home/dianhua/robot_test/docs/debug_notes.md) 的模板并按阶段记录。
