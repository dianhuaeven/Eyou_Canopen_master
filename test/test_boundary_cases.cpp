#include <gtest/gtest.h>

#include <climits>
#include <fstream>
#include <string>

#include "canopen_hw/canopen_master.hpp"
#include "canopen_hw/cia402_state_machine.hpp"
#include "canopen_hw/joints_config.hpp"
#include "canopen_hw/shared_state.hpp"

using canopen_hw::CiA402State;
using canopen_hw::CiA402StateMachine;
using canopen_hw::kCtrl_DisableVoltage;
using canopen_hw::kCtrl_FaultReset;
using canopen_hw::kMode_CSP;

// --- axis_count 边界 ---

TEST(Boundary, AxisCountZeroNormalizesToOne) {
  canopen_hw::SharedState shared(6);
  canopen_hw::CanopenMasterConfig cfg;
  cfg.axis_count = 0;
  cfg.master_node_id = 127;
  cfg.can_interface = "can0";

  canopen_hw::CanopenMaster master(cfg, &shared);
  EXPECT_EQ(master.config().axis_count, 1u);
}

TEST(Boundary, AxisCountExceedsMaxClamped) {
  // 构造时传入超大值，内部应限制在 kMaxAxisCount
  canopen_hw::SharedState shared(100);
  EXPECT_EQ(shared.axis_count(), canopen_hw::SharedState::kMaxAxisCount);

  // 验证 RecomputeAllOperational 不越界
  shared.RecomputeAllOperational();
  const auto snap = shared.Snapshot();
  // all_operational 应为 false (默认 feedback 未设置 is_operational)
  EXPECT_FALSE(snap.all_operational);
}

TEST(Boundary, AxisCountZeroClampedToOne) {
  canopen_hw::SharedState shared(0);
  EXPECT_EQ(shared.axis_count(), 1u);
}

// --- node_id 边界 ---

TEST(Boundary, NodeIdZeroDefaultFill) {
  canopen_hw::SharedState shared(6);
  canopen_hw::CanopenMasterConfig cfg;
  cfg.axis_count = 3;
  cfg.master_node_id = 127;
  cfg.can_interface = "can0";
  cfg.joints.assign(3, canopen_hw::CanopenMasterConfig::JointConfig{});

  canopen_hw::CanopenMaster master(cfg, &shared);
  ASSERT_EQ(master.config().joints.size(), 3u);
  EXPECT_EQ(master.config().joints[0].node_id, 1u);
  EXPECT_EQ(master.config().joints[1].node_id, 2u);
  EXPECT_EQ(master.config().joints[2].node_id, 3u);
}

TEST(Boundary, NodeIdOutOfRange) {
  // node_id: 0 应被拒绝
  {
    const std::string path = "/tmp/joints_test_node0.yaml";
    std::ofstream ofs(path);
    ofs << "joints:\n"
           "  - name: joint_bad\n"
           "    canopen:\n"
           "      node_id: 0\n"
           "    counts_per_rev: 1000\n";
    ofs.close();

    std::string error;
    canopen_hw::CanopenMasterConfig cfg;
    const bool ok =
        canopen_hw::LoadJointsYaml(path, &error, &cfg);
    EXPECT_FALSE(ok);
    EXPECT_NE(error.find("invalid node_id"), std::string::npos);
  }

  // node_id: 128 应被拒绝
  {
    const std::string path = "/tmp/joints_test_node128.yaml";
    std::ofstream ofs(path);
    ofs << "joints:\n"
           "  - name: joint_bad\n"
           "    canopen:\n"
           "      node_id: 128\n"
           "    counts_per_rev: 1000\n";
    ofs.close();

    std::string error;
    canopen_hw::CanopenMasterConfig cfg;
    const bool ok =
        canopen_hw::LoadJointsYaml(path, &error, &cfg);
    EXPECT_FALSE(ok);
    EXPECT_NE(error.find("invalid node_id"), std::string::npos);
  }
}

// --- int32 极值位置 ---

TEST(Boundary, Int32ExtremePositionSameValue) {
  // ros_target == actual_position 时应能正常解锁
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);

  sm.Update(0x0040, kMode_CSP, INT32_MAX);
  sm.Update(0x0021, kMode_CSP, INT32_MAX);
  sm.set_ros_target(INT32_MAX);
  sm.Update(0x0027, kMode_CSP, INT32_MAX);

  // ros_target == actual, AbsDiff=0 <= threshold，应解锁
  EXPECT_EQ(sm.safe_target(), INT32_MAX);
  EXPECT_FALSE(sm.is_position_locked());
  EXPECT_TRUE(sm.is_operational());
}

TEST(Boundary, Int32ExtremeAbsDiffAutoAlignsAndUnlocks) {
  // 大偏差场景下，CSP 会先自动重基准到当前实际位置，
  // 避免长期停留在 not operational。
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);
  sm.Update(0x0040, kMode_CSP, INT32_MIN);
  sm.Update(0x0021, kMode_CSP, INT32_MIN);
  sm.set_ros_target(INT32_MAX);  // 最大差值
  sm.Update(0x0027, kMode_CSP, INT32_MIN);

  EXPECT_FALSE(sm.is_position_locked());
  EXPECT_TRUE(sm.is_operational());
  EXPECT_EQ(sm.safe_target(), INT32_MIN);
}

// --- 故障复位次数耗尽 ---

TEST(Boundary, FaultResetMaxAttemptsReached) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);
  sm.set_fault_reset_policy(1, 2, 2);  // hold=1, wait=2, max=2

  // 推进到 OperationEnabled
  sm.Update(0x0040, kMode_CSP, 100);  // SwitchOnDisabled
  sm.Update(0x0021, kMode_CSP, 100);  // ReadyToSwitchOn
  sm.set_ros_target(100);
  sm.Update(0x0027, kMode_CSP, 100);  // OperationEnabled
  ASSERT_TRUE(sm.is_operational());

  // 进入 Fault (statusword=0x0008)
  // 注: 0x0008 对应 Fault (kState_Fault=0x08), 不经过 FaultReactionActive
  sm.Update(0x0008, kMode_CSP, 100);
  ASSERT_EQ(sm.state(), CiA402State::Fault);

  // 首次进入 Fault: fault_phase_ = Idle -> HoldLow, phase_cycles_=0
  // 本次 Update 里 phase_cycles 增到 1, hold_low_cycles_=1, 所以直接转到 SendEdge
  // 下一次 Update 才执行 SendEdge

  // 第一次复位尝试:
  // 上面的 Update(0x0008) 已经走了 HoldLow 且 phase_cycles==1==hold, 转到 SendEdge
  sm.Update(0x0008, kMode_CSP, 100);  // SendEdge: 发 FaultReset, 转 WaitRecovery
  EXPECT_EQ(sm.controlword(), kCtrl_FaultReset);
  EXPECT_EQ(sm.fault_reset_count(), 1);

  // WaitRecovery: wait=2 周期
  sm.Update(0x0008, kMode_CSP, 100);  // WaitRecovery wait_cycles_=1
  sm.Update(0x0008, kMode_CSP, 100);  // WaitRecovery wait_cycles_=2 >= max_wait(2), 回到 HoldLow

  // 第二次复位尝试:
  sm.Update(0x0008, kMode_CSP, 100);  // HoldLow phase_cycles_=1 >= hold(1), 转 SendEdge
  sm.Update(0x0008, kMode_CSP, 100);  // SendEdge: 发 FaultReset
  EXPECT_EQ(sm.controlword(), kCtrl_FaultReset);
  EXPECT_EQ(sm.fault_reset_count(), 2);

  // WaitRecovery 超时
  sm.Update(0x0008, kMode_CSP, 100);  // WaitRecovery wait_cycles_=1
  sm.Update(0x0008, kMode_CSP, 100);  // WaitRecovery wait_cycles_=2 >= max_wait(2)

  // fault_reset_count==2 == max, 应进入 PermanentFault
  sm.Update(0x0008, kMode_CSP, 100);
  EXPECT_EQ(sm.controlword(), kCtrl_DisableVoltage);
  EXPECT_EQ(sm.fault_reset_count(), 2);  // 不再增长

  // 再多跑几个周期，确认不会再发 FaultReset
  sm.Update(0x0008, kMode_CSP, 100);
  sm.Update(0x0008, kMode_CSP, 100);
  EXPECT_EQ(sm.controlword(), kCtrl_DisableVoltage);
  EXPECT_EQ(sm.fault_reset_count(), 2);
}
