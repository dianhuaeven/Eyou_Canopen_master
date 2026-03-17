# CANopen 主站调试操作手册

## 1. 环境准备

```bash
# 1) 启动 CAN 口
sudo ip link set can0 up type can bitrate 1000000

# 2) 查看接口状态
ip -details -statistics link show can0
```

## 2. 总线观测

```bash
# 实时抓包
candump can0

# 仅看关键帧（SYNC + PDO + Heartbeat + EMCY）
candump can0,080:7FF,180:7FF,200:7FF,280:7FF,700:7FF
```

预期：
- SYNC 周期约 10ms
- Node 1-6 心跳帧应可见（0x701~0x706）
- 运行时可见 `0x18x/0x28x` 与 `0x20x` PDO 帧

## 3. SDO 手工诊断（Node 1 示例）

```bash
# 读 statusword 0x6041
cansend can0 601#40.41.60.00.00.00.00.00

# 设 mode_of_operation=8 (CSP)
cansend can0 601#2F.60.60.00.08.00.00.00

# 发 shutdown (0x6040=0x0006)
cansend can0 601#2B.40.60.00.06.00.00.00

# 发 enable operation (0x6040=0x000F)
cansend can0 601#2B.40.60.00.0F.00.00.00
```

## 4. dcfgen 生成流程

```bash
# 从 master.yaml 生成主站 DCF 和各轴 concise bin
dcfgen -S -r -d /home/dianhua/robot_test/config \
       /home/dianhua/robot_test/config/master.yaml
```

说明：
- 当前仓库包含两份 EDS：
  - `YiyouServo_V1.4.eds`：原始文件
  - `YiyouServo_V1.4.dcfgen.eds`：为适配 dcf-tools 2.3.5 解析规则的兼容版本
- `master.yaml` 当前指向兼容版 EDS。

## 5. 本地编译与单测

```bash
# 状态机单测
g++ -std=c++17 -I/home/dianhua/robot_test/include \
    /home/dianhua/robot_test/src/cia402_state_machine.cpp \
    /home/dianhua/robot_test/test/test_state_machine.cpp \
    -o /tmp/test_state_machine && /tmp/test_state_machine

# shared_state 单测
g++ -std=c++17 -I/home/dianhua/robot_test/include \
    /home/dianhua/robot_test/src/shared_state.cpp \
    /home/dianhua/robot_test/test/test_shared_state.cpp \
    -o /tmp/test_shared_state && /tmp/test_shared_state

# 单位换算单测
g++ -std=c++17 -I/home/dianhua/robot_test/include \
    /home/dianhua/robot_test/src/canopen_robot_hw.cpp \
    /home/dianhua/robot_test/src/shared_state.cpp \
    /home/dianhua/robot_test/test/test_unit_conversion.cpp \
    -o /tmp/test_unit_conversion && /tmp/test_unit_conversion
```

## 6. 故障排查清单

1. 看不到任何帧：优先检查终端电阻、电源、CAN_H/CAN_L 线序。
2. 有心跳但无 PDO：检查 NMT 是否进入 Operational、PDO COB-ID 是否被禁用(bit31)。
3. 反复 FAULT：检查 statusword 与 EMCY 码，确认复位节流参数是否过短。
4. dcfgen 报 EDS 格式错误：切换到 `YiyouServo_V1.4.dcfgen.eds` 并使用 `-S`。
5. 主站退出后行为异常：做“断 SYNC/强杀进程”实验并确认驱动器策略。
