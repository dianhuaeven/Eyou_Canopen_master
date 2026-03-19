#include <gtest/gtest.h>

#include "canopen_hw/canopen_master.hpp"

TEST(MasterConfig, ZeroAxisNormalized) {
  canopen_hw::SharedState shared;
  canopen_hw::CanopenMasterConfig cfg;
  cfg.axis_count = 0;  // 构造时会被归一化为至少 1 轴。
  cfg.master_node_id = 127;
  cfg.can_interface = "can0";
  cfg.node_ids.clear();

  canopen_hw::CanopenMaster master(cfg, &shared);

  const auto& normalized = master.config();
  EXPECT_EQ(normalized.axis_count, 1u);
  EXPECT_EQ(normalized.node_ids.size(), 1u);
  EXPECT_EQ(normalized.node_ids[0], 1u);
  EXPECT_EQ(normalized.verify_pdo_mapping.size(), 1u);
  EXPECT_EQ(normalized.position_lock_thresholds.size(), 1u);
  EXPECT_EQ(normalized.max_fault_resets.size(), 1u);
  EXPECT_EQ(normalized.fault_reset_hold_cycles.size(), 1u);
}

TEST(MasterConfig, NotRunningAfterConstruction) {
  canopen_hw::SharedState shared;
  canopen_hw::CanopenMasterConfig cfg;
  cfg.axis_count = 0;
  cfg.master_node_id = 127;
  cfg.can_interface = "can0";
  cfg.node_ids.clear();

  canopen_hw::CanopenMaster master(cfg, &shared);

  EXPECT_FALSE(master.running());
  EXPECT_EQ(master.axis_count(), 0u);
}
