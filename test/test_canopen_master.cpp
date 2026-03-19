#include <cassert>

#include "canopen_hw/canopen_master.hpp"

int main() {
  canopen_hw::SharedState shared;
  canopen_hw::CanopenMasterConfig cfg;
  cfg.axis_count = 0;  // 构造时会被归一化为至少 1 轴。
  cfg.master_node_id = 127;
  cfg.can_interface = "can0";
  cfg.node_ids.clear();  // 空配置会按 axis_count 自动补默认 node_id。

  canopen_hw::CanopenMaster master(cfg, &shared);

  // 不依赖真实 can0 或硬件，仅验证构造期配置归一化逻辑。
  const auto& normalized = master.config();
  assert(normalized.axis_count == 1);
  assert(normalized.node_ids.size() == 1);
  assert(normalized.node_ids[0] == 1);
  assert(normalized.verify_pdo_mapping.size() == 1);
  assert(normalized.position_lock_thresholds.size() == 1);
  assert(normalized.max_fault_resets.size() == 1);
  assert(normalized.fault_reset_hold_cycles.size() == 1);

  assert(!master.running());
  assert(master.axis_count() == 0);
  return 0;
}
