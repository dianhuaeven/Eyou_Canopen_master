# 文档索引与归档规则

更新时间：2026-03-24

## 当前有效文档（优先阅读）

1. `docs/2026-03-21_canopen_pdo_boot_diagnosis_report.md`  
   CANopen 启动/PDO 故障统一修复报告与 commit 级修复计划。
2. `docs/2026-03-21_dcf_urdf_fix.md`  
   本次 EDS/DCF/URDF 修复摘要。
3. `docs/project_overview.md`  
   当前代码架构与运行机制总览。
4. `docs/usage.md`  
   部署、运行、联调步骤。
5. `docs/yaml_config_guide.md`  
   `master.yaml` 与 `joints.yaml` 配置说明。
6. `docs/api_reference.md`  
   核心模块与接口说明。
7. `docs/release_readiness.md`  
   发布前检查项。
8. `docs/ros_adapter_plan.md` / `docs/soak_test_plan.md` / `docs/fault_injection_checklist.md`  
   运行稳定性与压测/注入计划。
9. `docs/2026-03-22_enable_protection_epoch_ready_bug_report_and_fix_plan.md`  
   使能保护零位抽动问题的根因分析与分阶段修复计划（epoch-ready + fault latch）。
10. `docs/2026-03-23_reboot_auto_enable_fix_plan.md`  
    电机掉线复电后自动使能问题的 commit 级修复路线图与代码量评估。
11. `docs/2026-03-23_ip_mode_support_commit_plan.md`  
    IP（mode=7）支持改造的 commit 路线表（含 0x60C1/0x60C2 策略与回退路径）。
12. `docs/command_cheatsheet.md`  
    CAN 启动、赋权、启动节点、service/topic 调用命令速查。
13. `docs/2026-03-24_state_management_refactor_commit_plan.md`  
    基于 `0324import.md` 的“现状调研 + 精确到 commit 的状态管理重构计划图”。

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

## 历史归档目录

- `docs/archive/`：历史设计、评审、阶段性计划、旧版排障记录。

## 归档规则

1. 阶段性报告（`bug_report/work_report/quality_review`）完成后归档，不覆盖历史版本。
2. 同主题仅保留一个“当前执行基线”文档在 `docs/` 顶层。
3. 新的现场修复与联调结论优先写入日期化文档（建议格式 `YYYY-MM-DD_*.md`）。
