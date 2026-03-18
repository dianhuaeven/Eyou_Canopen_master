#pragma once

#include <cstdint>

namespace canopen_hw {

// CiA 402 state decoding masks and values.
constexpr uint16_t kStateMask_ReadySwitchOn = 0x006F;
// 0x004F 同时用于 NotReady/SwitchOnDisabled/Fault/FaultReactionActive 判定。
constexpr uint16_t kStateMask_Basic = 0x004F;

constexpr uint16_t kState_NotReadyToSwitchOn = 0x0000;
constexpr uint16_t kState_SwitchOnDisabled = 0x0040;
constexpr uint16_t kState_ReadyToSwitchOn = 0x0021;
constexpr uint16_t kState_SwitchedOn = 0x0023;
constexpr uint16_t kState_OperationEnabled = 0x0027;
constexpr uint16_t kState_QuickStopActive = 0x0007;
constexpr uint16_t kState_FaultReactionActive = 0x000F;
constexpr uint16_t kState_Fault = 0x0008;

// Statusword bit positions.
constexpr uint16_t kStatusBit_VoltageEnabled = (1u << 4);
constexpr uint16_t kStatusBit_Warning = (1u << 7);
constexpr uint16_t kStatusBit_Remote = (1u << 9);
constexpr uint16_t kStatusBit_TargetReached = (1u << 10);
constexpr uint16_t kStatusBit_SetPointAcknowledged = (1u << 12);
constexpr uint16_t kStatusBit_FollowingError = (1u << 13);

// Controlword commands.
constexpr uint16_t kCtrl_Shutdown = 0x0006;
constexpr uint16_t kCtrl_SwitchOn = 0x0007;
constexpr uint16_t kCtrl_EnableOperation = 0x000F;
constexpr uint16_t kCtrl_DisableVoltage = 0x0000;
constexpr uint16_t kCtrl_EnableVoltage = 0x0002;
constexpr uint16_t kCtrl_QuickStop = 0x0002;
constexpr uint16_t kCtrl_DisableOperation = 0x0007;
constexpr uint16_t kCtrl_FaultReset = 0x0080;

// Mode of operation for CSP.
constexpr int8_t kMode_CSP = 8;

enum class CiA402State {
  NotReadyToSwitchOn,
  SwitchOnDisabled,
  ReadyToSwitchOn,
  SwitchedOn,
  OperationEnabled,
  QuickStopActive,
  FaultReactionActive,
  Fault,
};

}  // namespace canopen_hw
