# 文档索引与归档规则

更新时间：2026-03-25

## 当前有效文档（优先阅读）

1. `docs/2026-03-25_现行安全行为规范.md`  
   当前安全行为与回归验收基线；与 `0324import.md` 冲突时以此为准。
2. `docs/2026-03-25_ip_follow_joint_trajectory_executor_design.md`  
   MoveIt 到 IP 模式执行器的设计方案（单轴先行）。
3. `docs/2026-03-25_ip_follow_joint_trajectory_executor_commit_plan.md`  
   单轴 MVP 到后续扩六轴的实施计划与验收标准。
4. `docs/2026-03-25_ip_follow_joint_trajectory_executor_implementation_report.md`  
   单轴执行器当前实际落地状态与提交级实现报告。
5. `docs/project_overview.md`  
   当前代码架构与运行机制总览。
6. `docs/usage.md`  
   部署、运行、联调步骤。
7. `docs/yaml_config_guide.md`  
   `master.yaml` 与 `joints.yaml` 配置说明。
8. `docs/api_reference.md`  
   核心模块与接口说明。
9. `docs/release_readiness.md`  
   发布前检查项。
10. `docs/ros_adapter_plan.md` / `docs/soak_test_plan.md` / `docs/fault_injection_checklist.md`  
   运行稳定性与压测/注入计划。
11. `docs/2026-03-21_canopen_pdo_boot_diagnosis_report.md`  
   CANopen 启动/PDO 故障统一修复报告与 commit 级修复计划。
12. `docs/2026-03-21_dcf_urdf_fix.md`  
   EDS/DCF/URDF 修复摘要。
13. `docs/2026-03-22_enable_protection_epoch_ready_bug_report_and_fix_plan.md`  
    使能保护与 epoch-ready 机制的历史问题分析。
14. `docs/2026-03-23_ip_mode_support_commit_plan.md`  
    IP（mode=7）支持改造记录与回退策略。
15. `docs/command_cheatsheet.md`  
    CAN 启动、赋权、启动节点、service/topic 调用命令速查。
16. `docs/0324import.md`  
    2026-03-24 架构设计草案；不是当前唯一验收基线。

## 2026-03-21 已归档文档

以下阶段性报告已转入：

`docs/archive/2026-03-21_deprecated/`

- `bug_report_C06-C14.md`
- `bugfix_report_2026-03-19_known_issues.md`
- `gap_report_engineering_completeness.md`
- `quality_review_2026-03-19.md`
- `work_report_C06-C14.md`
- `work_report_ros_adapter.md`

归档原因：内容已被 2026-03-21 的统一修复报告覆盖，保留历史追溯但不再作为当前执行基线。

## 2026-03-25 已归档文档

以下一次性问题报告与阶段性重构计划已转入：

`docs/archive/2026-03-25_deprecated/`

- `2026-03-21_controller_manager_blocking_bug_report.md`
- `2026-03-22_canopen_integration_summary_and_fix_plan.md`
- `2026-03-22_csp_is_operational_bug_report.md`
- `2026-03-22_zero_position_jerk_bug_report.md`
- `2026-03-23_reboot_auto_enable_fix_plan.md`
- `2026-03-24_state_management_refactor_commit_plan.md`

归档原因：这些文档属于阶段性排障记录或过期实施计划，当前已分别被
`usage.md`、`api_reference.md`、`project_overview.md`、`2026-03-25_现行安全行为规范.md`
以及现有代码/测试覆盖，不再作为执行入口保留在 `docs/` 顶层。

## 历史归档目录

- `docs/archive/`：历史设计、评审、阶段性计划、旧版排障记录。

## 归档规则

1. 阶段性报告（`bug_report/work_report/quality_review`）完成后归档，不覆盖历史版本。
2. 同主题仅保留一个“当前执行基线”文档在 `docs/` 顶层。
3. 新的现场修复与联调结论优先写入日期化文档（建议格式 `YYYY-MM-DD_*.md`）。
