#include <cassert>

#include "canopen_hw/canopen_master.hpp"

int main() {
  canopen_hw::SharedState shared;
  canopen_hw::CanopenMasterConfig cfg;
  cfg.axis_count = 6;
  cfg.master_node_id = 127;
  cfg.can_interface = "can0";

  canopen_hw::CanopenMaster master(cfg, &shared);
  assert(!master.running());
  assert(master.axis_count() == 0);

  const bool started = master.Start();
  assert(started);
  assert(master.running());

  master.Stop();
  assert(!master.running());

  return 0;
}
