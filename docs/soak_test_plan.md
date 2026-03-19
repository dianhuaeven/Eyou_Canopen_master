# Soak 测试计划

> 适用于 canopen_hw 驱动层上线前的长时间稳定性验证
> 日期：2026-03-19

---

## 一、测试目标

在真实硬件（6 轴伺服 + CAN 总线）上长时间运行，验证：

1. 无内存泄漏或资源耗尽
2. 无状态机死锁或异常跳转
3. 心跳丢失后能正确检测并恢复
4. 日志无异常 ERROR 级别输出
5. 关机流程可靠完成

---

## 二、前置条件

| 项目 | 要求 |
|------|------|
| 硬件 | 6 轴伺服驱动器 + CAN 总线连接正常 |
| 软件 | `canopen_hw_node` 编译通过（Release 模式） |
| 配置 | `config/joints.yaml` 6 轴参数已校准 |
| 监控 | `htop` / `top` 可观察进程内存和 CPU |
| 日志 | spdlog 输出重定向到文件：`./canopen_hw_node 2>&1 | tee soak.log` |

---

## 三、8 小时 Soak 测试

### 3.1 执行步骤

1. **启动前检查**
   ```bash
   ip link show can0          # 确认 CAN 接口 UP
   candump can0 -n 10         # 确认总线有心跳帧
   ```

2. **启动节点**
   ```bash
   ./build/canopen_hw_node --dcf config/master.dcf --joints config/joints.yaml \
       2>&1 | tee soak_8h_$(date +%Y%m%d_%H%M%S).log &
   SOAK_PID=$!
   ```

3. **持续监控（每 30 分钟记录一次）**
   ```bash
   # 内存占用
   ps -o pid,vsz,rss,comm -p $SOAK_PID

   # 检查日志中的 ERROR 计数
   grep -c '\[ERROR\]' soak_8h_*.log
   ```

4. **8 小时后停止**
   ```bash
   kill -SIGINT $SOAK_PID
   wait $SOAK_PID
   echo "Exit code: $?"
   ```

### 3.2 通过标准

| 指标 | 通过条件 |
|------|----------|
| 进程存活 | 8 小时内无崩溃、无段错误 |
| 内存 RSS | 波动 < 初始值的 20%，无持续增长趋势 |
| ERROR 日志 | 0 条（排除已知的启动阶段 PDO 重试） |
| 退出码 | `kill -SIGINT` 后退出码为 0 |
| 关机时间 | 收到 SIGINT 后 5 秒内完成退出 |
| 心跳 | 无 `heartbeat lost` 日志（排除人为拔线测试） |

---

## 四、24 小时 Soak 测试（可选）

在 8 小时测试通过后执行，流程相同，额外增加：

### 4.1 附加监控

```bash
# 每小时记录一次完整状态
watch -n 3600 'ps -o pid,vsz,rss,comm -p $SOAK_PID; \
  grep -c "\[ERROR\]" soak_24h_*.log; \
  grep -c "\[WARN\]" soak_24h_*.log'
```

### 4.2 附加通过标准

| 指标 | 通过条件 |
|------|----------|
| 内存 RSS 趋势 | 24 小时内无单调递增（允许周期性 GC 波动） |
| WARN 日志 | < 100 条 / 24h |
| CAN 总线错误帧 | `ip -s link show can0` 中 errors 计数无增长 |

---

## 五、故障场景 Soak（结合故障注入）

在 Soak 运行期间，按 `fault_injection_checklist.md` 中的场景逐一注入故障，观察恢复行为。每次注入间隔 >= 5 分钟。

---

## 六、结果记录模板

```
测试日期: ____
测试时长: ____ 小时
硬件配置: ____
软件版本: git commit ____

启动时 RSS: ____ KB
结束时 RSS: ____ KB
ERROR 日志数: ____
WARN  日志数: ____
退出码: ____
关机耗时: ____ 秒

结论: [ ] 通过  [ ] 不通过
备注: ____
```
