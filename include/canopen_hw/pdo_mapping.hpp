#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace lely {
namespace canopen {
class BasicDriver;
}  // namespace canopen
}  // namespace lely

namespace canopen_hw {

struct PdoChannelMapping {
  uint32_t cob_id = 0;
  std::vector<uint32_t> entries;
};

struct PdoMapping {
  std::array<PdoChannelMapping, 4> rpdo{};
  std::array<PdoChannelMapping, 4> tpdo{};
};

bool LoadExpectedPdoMappingFromDcf(const std::string& path, PdoMapping* mapping,
                                   std::string* error);

bool DiffPdoMapping(const PdoMapping& expected, const PdoMapping& actual,
                    std::vector<std::string>* diffs);

class PdoMappingReader {
 public:
  using DoneCallback =
      std::function<void(bool, const std::string&, const PdoMapping&)>;

  void Start(lely::canopen::BasicDriver& driver, DoneCallback cb);

 private:
  struct ReadStep {
    uint16_t idx = 0;
    uint8_t sub = 0;
    bool is_u8 = false;
    std::function<void(uint32_t)> on_value;
  };

  void BuildHeaderSteps();
  void BuildEntrySteps();
  void ScheduleNext();
  void Finish(bool ok, const std::string& error);

  lely::canopen::BasicDriver* driver_ = nullptr;
  DoneCallback done_cb_;
  std::vector<ReadStep> steps_;
  std::array<uint8_t, 4> rpdo_counts_{};
  std::array<uint8_t, 4> tpdo_counts_{};
  std::size_t step_index_ = 0;
  bool phase_entries_ = false;
  bool finished_ = false;
  PdoMapping mapping_{};
  std::string error_;
};

}  // namespace canopen_hw
