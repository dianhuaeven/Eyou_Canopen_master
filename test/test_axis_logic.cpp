#include "canopen_hw/axis_logic.hpp"

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "canopen_hw/bus_io.hpp"
#include "canopen_hw/cia402_defs.hpp"
#include "canopen_hw/shared_state.hpp"

namespace canopen_hw {
namespace {

struct BusCall {
  enum Type { kControlword, kPosition, kVelocity, kTorque, kMode };
  Type type;
  int64_t value;
};

class MockBusIO : public BusIO {
 public:
  std::vector<BusCall> calls;

  bool WriteControlword(uint16_t cw) override {
    calls.push_back({BusCall::kControlword, cw});
    return true;
  }
  bool WriteTargetPosition(int32_t pos) override {
    calls.push_back({BusCall::kPosition, pos});
    return true;
  }
  bool WriteTargetVelocity(int32_t vel) override {
    calls.push_back({BusCall::kVelocity, vel});
    return true;
  }
  bool WriteTargetTorque(int16_t torque) override {
    calls.push_back({BusCall::kTorque, torque});
    return true;
  }
  bool WriteModeOfOperation(int8_t mode) override {
    calls.push_back({BusCall::kMode, mode});
    return true;
  }
};

// statusword 常量
constexpr uint16_t kSW_NotReady = 0x0000;
constexpr uint16_t kSW_SwitchOnDisabled = 0x0040;
constexpr uint16_t kSW_ReadyToSwitchOn = 0x0021;
constexpr uint16_t kSW_SwitchedOn = 0x0023;
constexpr uint16_t kSW_OperationEnabled = 0x0027;
constexpr uint16_t kSW_Fault = 0x0008;

class AxisLogicTest : public ::testing::Test {
 protected:
  void SetUp() override {
    shared_ = std::make_unique<SharedState>(1);
    logic_ = std::make_unique<AxisLogic>(0, &bus_, shared_.get());
  }

  // 快速推进到 OperationEnabled 状态
  void DriveToOperational() {
    logic_->RequestEnable();
    logic_->ProcessRpdo(kSW_SwitchOnDisabled, 0, 0, 0, kMode_CSP);
    logic_->ProcessRpdo(kSW_ReadyToSwitchOn, 0, 0, 0, kMode_CSP);
    logic_->ProcessRpdo(kSW_SwitchedOn, 0, 0, 0, kMode_CSP);
    logic_->ProcessRpdo(kSW_OperationEnabled, 0, 0, 0, kMode_CSP);
  }

  MockBusIO bus_;
  std::unique_ptr<SharedState> shared_;
  std::unique_ptr<AxisLogic> logic_;
};

// --- RPDO 处理 ---

TEST_F(AxisLogicTest, ProcessRpdoWritesBusIO) {
  logic_->ProcessRpdo(kSW_SwitchOnDisabled, 100, 50, 10, kMode_CSP);

  // 应该写了 controlword + mode + position + velocity + torque
  ASSERT_GE(bus_.calls.size(), 5u);
  EXPECT_EQ(bus_.calls[0].type, BusCall::kControlword);
  EXPECT_EQ(bus_.calls[0].value, kCtrl_Shutdown);
  EXPECT_EQ(bus_.calls[1].type, BusCall::kMode);
  EXPECT_EQ(bus_.calls[2].type, BusCall::kPosition);
  EXPECT_EQ(bus_.calls[3].type, BusCall::kVelocity);
  EXPECT_EQ(bus_.calls[4].type, BusCall::kTorque);
}

TEST_F(AxisLogicTest, ProcessRpdoWritesEnableOperationControlwordInReadyToSwitchOn) {
  bus_.calls.clear();
  logic_->RequestEnable();
  logic_->ProcessRpdo(kSW_ReadyToSwitchOn, 0, 0, 0, kMode_CSP);

  ASSERT_FALSE(bus_.calls.empty());
  EXPECT_EQ(bus_.calls[0].type, BusCall::kControlword);
  EXPECT_EQ(bus_.calls[0].value, kCtrl_EnableOperation);
}

TEST_F(AxisLogicTest, ProcessRpdoUpdatesSharedState) {
  logic_->ProcessRpdo(kSW_SwitchOnDisabled, 1234, 56, 7, kMode_CSP);

  auto snap = shared_->Snapshot();
  EXPECT_EQ(snap.feedback[0].actual_position, 1234);
  EXPECT_EQ(snap.feedback[0].actual_velocity, 56);
  EXPECT_EQ(snap.feedback[0].actual_torque, 7);
  EXPECT_EQ(snap.feedback[0].statusword, kSW_SwitchOnDisabled);
}

TEST_F(AxisLogicTest, CSVModeWritesVelocity) {
  logic_->SetTargetMode(kMode_CSV);
  logic_->SetRosTargetVelocity(500);
  DriveToOperational();
  bus_.calls.clear();

  logic_->ProcessRpdo(kSW_OperationEnabled, 0, 0, 0, kMode_CSV);

  bool found_velocity = false;
  for (const auto& c : bus_.calls) {
    if (c.type == BusCall::kVelocity) {
      found_velocity = true;
    }
  }
  EXPECT_TRUE(found_velocity);
}

TEST_F(AxisLogicTest, CSTModeWritesTorque) {
  logic_->SetTargetMode(kMode_CST);
  logic_->SetRosTargetTorque(200);
  DriveToOperational();
  bus_.calls.clear();

  logic_->ProcessRpdo(kSW_OperationEnabled, 0, 0, 0, kMode_CST);

  bool found_torque = false;
  for (const auto& c : bus_.calls) {
    if (c.type == BusCall::kTorque) {
      found_torque = true;
    }
  }
  EXPECT_TRUE(found_torque);
}

// --- EMCY 处理 ---

TEST_F(AxisLogicTest, ProcessEmcyIncrementsCounter) {
  logic_->ProcessEmcy(0x1234, 0x01);
  EXPECT_EQ(logic_->health().emcy_count.load(), 1u);

  logic_->ProcessEmcy(0x5678, 0x02);
  EXPECT_EQ(logic_->health().emcy_count.load(), 2u);
}

TEST_F(AxisLogicTest, ProcessEmcyUpdatesSharedFeedback) {
  logic_->ProcessEmcy(0xABCD, 0x01);

  auto snap = shared_->Snapshot();
  EXPECT_EQ(snap.feedback[0].last_emcy_eec, 0xABCD);
}

// --- 心跳处理 ---

TEST_F(AxisLogicTest, HeartbeatLostSetsFault) {
  DriveToOperational();
  logic_->ProcessHeartbeat(true);

  auto snap = shared_->Snapshot();
  EXPECT_TRUE(snap.feedback[0].heartbeat_lost);
  EXPECT_TRUE(snap.feedback[0].is_fault);
  EXPECT_FALSE(snap.feedback[0].is_operational);
  EXPECT_EQ(logic_->health().heartbeat_lost.load(), 1u);
}

TEST_F(AxisLogicTest, HeartbeatRecoveredClearsFault) {
  DriveToOperational();
  logic_->ProcessHeartbeat(true);
  logic_->ProcessHeartbeat(false);

  auto snap = shared_->Snapshot();
  EXPECT_FALSE(snap.feedback[0].heartbeat_lost);
  EXPECT_EQ(logic_->health().heartbeat_recovered.load(), 1u);
}

// --- 命令接口 ---

TEST_F(AxisLogicTest, ConfigureDelegatesToStateMachine) {
  // 不崩溃即可
  logic_->Configure(100, 5, 3);
}

TEST_F(AxisLogicTest, ResetFaultReenables) {
  logic_->ResetFault();
  // 验证不崩溃，状态机内部会重置计数器并请求使能
}

TEST_F(AxisLogicTest, RequestDisableImmediatelyClearsOperationalFlag) {
  DriveToOperational();
  logic_->ProcessRpdo(kSW_OperationEnabled, 0, 0, 0, kMode_CSP);

  auto before = shared_->Snapshot();
  EXPECT_TRUE(before.feedback[0].is_operational);

  logic_->RequestDisable();

  auto after = shared_->Snapshot();
  EXPECT_FALSE(after.feedback[0].is_operational);
}

TEST_F(AxisLogicTest, FaultResetAttemptsMirroredToHealthCounters) {
  // 默认 hold_low_cycles=5，第 6 个 FAULT 周期会发送一次 reset edge。
  for (int i = 0; i < 6; ++i) {
    logic_->ProcessRpdo(kSW_Fault, 0, 0, 0, kMode_CSP);
  }
  EXPECT_EQ(logic_->health().fault_reset_attempts.load(), 1u);

  logic_->ResetFault();
  EXPECT_EQ(logic_->health().fault_reset_attempts.load(), 0u);
}

TEST_F(AxisLogicTest, NullBusIODoesNotCrash) {
  AxisLogic logic_no_bus(0, nullptr, shared_.get());
  logic_no_bus.ProcessRpdo(kSW_SwitchOnDisabled, 0, 0, 0, kMode_CSP);
  logic_no_bus.ProcessEmcy(0x1234, 0x01);
  logic_no_bus.ProcessHeartbeat(true);
}

TEST_F(AxisLogicTest, NullSharedStateDoesNotCrash) {
  AxisLogic logic_no_ss(0, &bus_, nullptr);
  logic_no_ss.ProcessRpdo(kSW_SwitchOnDisabled, 0, 0, 0, kMode_CSP);
  logic_no_ss.ProcessEmcy(0x1234, 0x01);
  logic_no_ss.ProcessHeartbeat(true);
}

TEST_F(AxisLogicTest, HaltRequestSetsHaltBitAndFreezesVelocityCommand) {
  logic_->SetTargetMode(kMode_CSV);
  logic_->SetRosTargetVelocity(500);
  logic_->RequestEnable();

  logic_->ProcessRpdo(kSW_SwitchOnDisabled, 1000, 0, 0, kMode_CSV);
  logic_->ProcessRpdo(kSW_ReadyToSwitchOn, 1000, 0, 0, kMode_CSV);
  logic_->ProcessRpdo(kSW_OperationEnabled, 1000, 0, 0, kMode_CSV);
  logic_->ProcessRpdo(kSW_OperationEnabled, 1000, 0, 0, kMode_CSV);

  bus_.calls.clear();
  logic_->RequestHalt();
  logic_->ProcessRpdo(kSW_OperationEnabled, 1010, 0, 0, kMode_CSV);

  ASSERT_GE(bus_.calls.size(), 5u);
  EXPECT_EQ(bus_.calls[0].type, BusCall::kControlword);
  EXPECT_EQ(bus_.calls[0].value,
            static_cast<int64_t>(kCtrl_EnableOperation | kCtrl_Bit_Halt));
  EXPECT_EQ(bus_.calls[2].type, BusCall::kPosition);
  EXPECT_EQ(bus_.calls[2].value, 1010);
  EXPECT_EQ(bus_.calls[3].type, BusCall::kVelocity);
  EXPECT_EQ(bus_.calls[3].value, 0);

  logic_->SetRosTargetVelocity(900);
  bus_.calls.clear();
  logic_->ProcessRpdo(kSW_OperationEnabled, 1020, 0, 0, kMode_CSV);
  ASSERT_GE(bus_.calls.size(), 5u);
  EXPECT_EQ(bus_.calls[2].value, 1020);
  EXPECT_EQ(bus_.calls[3].value, 0);

  logic_->RequestResume();
  bus_.calls.clear();
  logic_->ProcessRpdo(kSW_OperationEnabled, 1020, 0, 0, kMode_CSV);
  ASSERT_GE(bus_.calls.size(), 5u);
  EXPECT_EQ(bus_.calls[0].value, kCtrl_EnableOperation);
  EXPECT_EQ(bus_.calls[3].value, 900);
}

}  // namespace
}  // namespace canopen_hw
