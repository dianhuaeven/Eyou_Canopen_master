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
catkin_make --pkg canopen_hw
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
roslaunch canopen_hw canopen_hw.launch
```

可选参数：
- `dcf_path:=<path>` — DCF 文件路径
- `joints_path:=<path>` — joints.yaml 路径
- `loop_hz:=100.0` — 控制循环频率

---

## 6. ROS 接口

### 6.1 Services

| Service | 类型 | 说明 |
|---------|------|------|
| `~halt` | `std_srvs/Trigger` | 停止运动，保持总线连接 |
| `~recover` | `std_srvs/Trigger` | 从 halt/故障状态恢复 |
| `~shutdown` | `std_srvs/Trigger` | 关闭主站并退出节点 |
| `~set_mode` | `canopen_hw/SetMode` | 切换运动模式（CSP=8, CSV=9） |

### 6.2 模式切换流程

```bash
# 1. 停止运动
rosservice call /canopen_hw_node/halt

# 2. 切换到 CSV 模式（所有轴）
for i in 0 1 2 3 4 5; do
  rosservice call /canopen_hw_node/set_mode "{axis_index: $i, mode: 9}"
done

# 3. 切换 controller
rosrun controller_manager controller_manager stop arm_position_controller
rosrun controller_manager controller_manager start arm_velocity_controller

# 4. 恢复运动
rosservice call /canopen_hw_node/recover
```

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

## 9. 单测

```bash
# 独立编译模式
cmake --build build -j && cd build && ctest --output-on-failure

# catkin 模式
catkin_make run_tests_canopen_hw
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
