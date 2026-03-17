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

可选字段（总线）：
- `canopen.auto_fix_pdo`：是否允许启动时自动改写 PDO 映射（默认 false）。

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

```bash
cmake -S /home/dianhua/robot_test -B /home/dianhua/robot_test/build
cmake --build /home/dianhua/robot_test/build -j
```

---

## 5. 启动

```bash
/home/dianhua/robot_test/build/canopen_hw_node \
  --dcf /home/dianhua/robot_test/config/master.dcf \
  --joints /home/dianhua/robot_test/config/joints.yaml
```

说明：
- `--dcf` 必须指向存在的 DCF 文件，否则程序直接退出。
- `--joints` 不存在时只报警，但仍可运行。
- 路径建议使用绝对路径，避免工作目录变化导致失败。

---

## 6. 运行期验证点

基础验证：
- 主站启动无报错
- 心跳可见
- PDO 正常刷新

状态机验证：
- 状态能够进入 `OPERATION_ENABLED`
- 位置反馈更新后 `all_operational` 变为 true

---

## 7. 关机流程

程序退出时执行：
1. `Disable Operation` (0x6040=0x0007)
2. 等待状态进入 `SwitchedOn`（超时 2s）
3. `Shutdown` (0x6040=0x0006)
4. 等待状态进入 `ReadyToSwitchOn`（超时 1s）
5. `NMT Stop`

验证方式：
- Ctrl+C 后可观察到 PDO 下发顺序与状态回落。

---

## 8. 单测

```bash
cmake --build /home/dianhua/robot_test/build -j

# 或直接运行目标
/home/dianhua/robot_test/build/test_state_machine
/home/dianhua/robot_test/build/test_shared_state
/home/dianhua/robot_test/build/test_unit_conversion
/home/dianhua/robot_test/build/test_joints_config
```

---

## 9. 常见问题

1. 无任何帧：优先检查终端电阻、电源、CAN_H/CAN_L 线序。
2. 有心跳无 PDO：确认 NMT 已进入 Operational，检查 PDO COB-ID 是否被禁用(bit31)。
3. 反复 FAULT：检查 statusword 与 EMCY 码，确认复位节流参数是否过短。
4. 退出后行为异常：做“断 SYNC/强杀进程”实验并确认驱动器策略。

---

## 10. D1~D9 联调记录

请参见 [docs/debug_notes.md](/home/dianhua/robot_test/docs/debug_notes.md) 的模板并按阶段记录。
