# 文档索引与归档规则

更新时间：2026-03-25

## 当前有效文档（优先阅读）

1. `docs/2026-03-25_现行安全行为规范.md`  
   当前安全行为与回归验收基线；与历史设计草案冲突时以此为准。
2. `docs/project_overview.md`  
   当前代码架构、线程模型与控制链路总览。
3. `docs/ip_follow_joint_trajectory_executor.md`  
   IP 轨迹执行器当前实现（多轴、动态 DOF、接线位置与限制）。
4. `docs/usage.md`  
   上机部署、启动参数、联调流程与常见操作。
5. `docs/yaml_config_guide.md`  
   `master.yaml/master.dcf` 与 `joints.yaml` 配置说明。
6. `docs/api_reference.md`  
   核心模块与对外接口速查。
7. `docs/release_readiness.md`  
   发布前检查项与验收门槛。
8. `docs/command_cheatsheet.md`  
   常用命令速查。
9. `docs/soak_test_plan.md` / `docs/fault_injection_checklist.md`  
   稳定性验证、故障注入和压测计划。
10. `docs/0324import.md`  
    历史架构草案（仅供背景参考，不作为当前验收基线）。

## 2026-03-21 已归档文档

目录：`docs/archive/2026-03-21_deprecated/`

- `bug_report_C06-C14.md`
- `bugfix_report_2026-03-19_known_issues.md`
- `gap_report_engineering_completeness.md`
- `quality_review_2026-03-19.md`
- `work_report_C06-C14.md`
- `work_report_ros_adapter.md`

归档原因：内容已被后续统一修复报告和当前使用文档覆盖，仅保留历史追溯价值。

## 2026-03-25 已归档文档

目录：`docs/archive/2026-03-25_deprecated/`

- `2026-03-21_controller_manager_blocking_bug_report.md`
- `2026-03-21_canopen_pdo_boot_diagnosis_report.md`
- `2026-03-21_dcf_urdf_fix.md`
- `2026-03-22_canopen_integration_summary_and_fix_plan.md`
- `2026-03-22_enable_protection_epoch_ready_bug_report_and_fix_plan.md`
- `2026-03-22_csp_is_operational_bug_report.md`
- `2026-03-22_zero_position_jerk_bug_report.md`
- `2026-03-23_ip_mode_support_commit_plan.md`
- `2026-03-23_reboot_auto_enable_fix_plan.md`
- `2026-03-24_state_management_refactor_commit_plan.md`
- `2026-03-25_ip_follow_joint_trajectory_executor_commit_plan.md`
- `2026-03-25_ip_follow_joint_trajectory_executor_design.md`
- `2026-03-25_ip_follow_joint_trajectory_executor_implementation_report.md`
- `eyou_submodule_history_migration_plan.md`
- `ros_adapter_plan.md`

归档原因：这些文档属于阶段性计划/实施报告，相关改造已落地，继续保留在顶层会误导为“当前执行入口”。

## 项目级关联文档

- `../../Eyou_ROS1_Master/docs/实施计划.md`：统一外观层 `Eyou_ROS1_Master` 包的分阶段实施计划（同时托管 `can_driver` 和 `Eyou_Canopen_Master` 两套后端）

## 当前模型来源说明

- 运行时默认 `robot_description` 现已统一使用 `../../car_urdf/urdf/car_urdf.urdf`
- `urdf/robot.urdf` 保留但已弃用，仅作历史参考与局部测试使用

## 历史归档目录

- `docs/archive/`：历史设计、阶段性计划、旧版排障记录、一次性评审产物。

## 归档规则

1. 阶段性计划、一次性问题报告、实施报告在目标完成后归档。
2. `docs/` 顶层仅保留“当前可执行基线”文档。
3. 新结论优先更新长期文档（`usage/api_reference/project_overview`），避免持续叠加“临时日期文档”。
