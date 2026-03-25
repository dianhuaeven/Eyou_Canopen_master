# 发布就绪评审清单

更新时间：2026-03-25  
适用范围：`Eyou_Canopen_Master`

---

## 1. 代码与提交状态

1. 工作区无非预期脏改动（允许现场参数文件如 `config/joints.yaml` 的单独本地差异）。
2. 本次发布涉及的功能提交已按主题拆分（`feat/refactor/fix`）。
3. 关键风险修复已纳入历史：
   - IP executor 多轴化与模块拆分
   - 生命周期缺失 DCF 导致的启动崩溃防护
   - action 回调线程与主循环共享数据互斥保护

---

## 2. 构建与测试门槛

必过命令：

```bash
cd ~/Robot24_catkin_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES=Eyou_Canopen_Master --no-color
```

推荐最小回归集：

```bash
cd ~/Robot24_catkin_ws/build/Eyou_Canopen_Master
ctest -R '^(CanopenRobotHwRos|IpFollowJointTrajectoryExecutor|LifecycleManager)\.' --output-on-failure
```

发布前完整回归（建议）：

```bash
cd ~/Robot24_catkin_ws/build/Eyou_Canopen_Master
ctest --output-on-failure
```

---

## 3. 配置与现场一致性检查

1. `master.dcf` 存在且与当前驱动固件匹配。
2. `joints.yaml` 轴数量、`node_id`、缩放参数、IP 约束参数与现场一致。
3. 启动参数与目标模式匹配：
   - 若走 JTC：`use_ip_executor=false`
   - 若走 IP executor：`use_ip_executor=true`，并确认 action namespace 正确
4. 启动后 `/diagnostics` 可见每轴健康数据。

---

## 4. 功能验收项

1. 生命周期服务链路可用：`init -> enable -> resume -> halt/disable -> shutdown`。
2. 故障路径可控：故障后 `recover -> enable -> resume` 行为符合预期。
3. IP 模式下对象通道行为正确：
   - 优先 `0x60C1:01`
   - 失败时回退 `0x607A` 并有日志
4. `use_ip_executor=true` 时，多轴 `FollowJointTrajectory` 可执行并反馈正常。

---

## 5. 压测与故障注入（发布前建议）

1. 按 `docs/soak_test_plan.md` 跑 soak。
2. 按 `docs/fault_injection_checklist.md` 完成故障注入。
3. 对发现的问题补充回归测试后再发版。

---

## 6. 最终签核模板

```text
日期:        ____
评审人:      ____
发布 Commit: ____

[ ] 构建通过
[ ] 最小回归集通过
[ ] 配置一致性检查通过
[ ] 生命周期与故障恢复验收通过
[ ] （建议）完整回归通过
[ ] （建议）soak 与故障注入通过

结论: [ ] 可发布  [ ] 不可发布
备注: ____
```
