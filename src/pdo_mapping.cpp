#include "canopen_hw/pdo_mapping.hpp"

#include <condition_variable>
#include <mutex>
#include <sstream>
#include <thread>

#include <lely/co/dcf.h>
#include <lely/co/dev.h>
#include <lely/coapp/driver.hpp>

namespace canopen_hw {

namespace {

bool GetU8(const co_dev_t* dev, uint16_t idx, uint8_t sub, uint8_t* out,
           std::string* error) {
  if (!co_dev_find_sub(dev, idx, sub)) {
    if (error) {
      std::ostringstream oss;
      oss << "missing object 0x" << std::hex << idx << ":" << std::dec
          << static_cast<int>(sub);
      *error = oss.str();
    }
    return false;
  }
  *out = co_dev_get_val_u8(dev, idx, sub);
  return true;
}

bool GetU32(const co_dev_t* dev, uint16_t idx, uint8_t sub, uint32_t* out,
            std::string* error) {
  if (!co_dev_find_sub(dev, idx, sub)) {
    if (error) {
      std::ostringstream oss;
      oss << "missing object 0x" << std::hex << idx << ":" << std::dec
          << static_cast<int>(sub);
      *error = oss.str();
    }
    return false;
  }
  *out = co_dev_get_val_u32(dev, idx, sub);
  return true;
}

std::string FormatEntry(uint32_t entry) {
  const uint16_t idx = static_cast<uint16_t>(entry & 0xFFFFu);
  const uint8_t sub = static_cast<uint8_t>((entry >> 16) & 0xFFu);
  const uint8_t bits = static_cast<uint8_t>((entry >> 24) & 0xFFu);
  std::ostringstream oss;
  oss << std::hex << idx << ":" << std::dec << static_cast<int>(sub) << "/"
      << static_cast<int>(bits);
  return oss.str();
}

}  // namespace

bool LoadExpectedPdoMappingFromDcf(const std::string& path, PdoMapping* mapping,
                                   std::string* error) {
  if (!mapping) {
    if (error) {
      *error = "mapping is null";
    }
    return false;
  }

  co_dev_t* dev = co_dev_create_from_dcf_file(path.c_str());
  if (!dev) {
    if (error) {
      *error = "failed to load dcf file";
    }
    return false;
  }

  bool ok = true;
  for (uint16_t i = 0; i < 4 && ok; ++i) {
    uint32_t cob_id = 0;
    ok = GetU32(dev, static_cast<uint16_t>(0x1400 + i), 1, &cob_id, error);
    if (!ok) {
      break;
    }
    mapping->rpdo[i].cob_id = cob_id;

    uint8_t count = 0;
    ok = GetU8(dev, static_cast<uint16_t>(0x1600 + i), 0, &count, error);
    if (!ok) {
      break;
    }
    mapping->rpdo[i].entries.clear();
    mapping->rpdo[i].entries.reserve(count);
    for (uint8_t sub = 1; sub <= count && ok; ++sub) {
      uint32_t entry = 0;
      ok = GetU32(dev, static_cast<uint16_t>(0x1600 + i), sub, &entry, error);
      if (!ok) {
        break;
      }
      mapping->rpdo[i].entries.push_back(entry);
    }
  }

  for (uint16_t i = 0; i < 4 && ok; ++i) {
    uint32_t cob_id = 0;
    ok = GetU32(dev, static_cast<uint16_t>(0x1800 + i), 1, &cob_id, error);
    if (!ok) {
      break;
    }
    mapping->tpdo[i].cob_id = cob_id;

    uint8_t count = 0;
    ok = GetU8(dev, static_cast<uint16_t>(0x1A00 + i), 0, &count, error);
    if (!ok) {
      break;
    }
    mapping->tpdo[i].entries.clear();
    mapping->tpdo[i].entries.reserve(count);
    for (uint8_t sub = 1; sub <= count && ok; ++sub) {
      uint32_t entry = 0;
      ok = GetU32(dev, static_cast<uint16_t>(0x1A00 + i), sub, &entry, error);
      if (!ok) {
        break;
      }
      mapping->tpdo[i].entries.push_back(entry);
    }
  }

  co_dev_destroy(dev);
  return ok;
}

bool DiffPdoMapping(const PdoMapping& expected, const PdoMapping& actual,
                    std::vector<std::string>* diffs) {
  if (diffs) {
    diffs->clear();
  }

  auto record = [&](const std::string& msg) {
    if (diffs) {
      diffs->push_back(msg);
    }
  };

  for (std::size_t i = 0; i < 4; ++i) {
    if (expected.rpdo[i].cob_id != actual.rpdo[i].cob_id) {
      std::ostringstream oss;
      oss << "RPDO" << (i + 1) << " COB-ID mismatch: expected 0x" << std::hex
          << expected.rpdo[i].cob_id << " actual 0x" << actual.rpdo[i].cob_id;
      record(oss.str());
    }
    if (expected.rpdo[i].entries.size() != actual.rpdo[i].entries.size()) {
      std::ostringstream oss;
      oss << "RPDO" << (i + 1) << " entry count mismatch: expected "
          << expected.rpdo[i].entries.size() << " actual "
          << actual.rpdo[i].entries.size();
      record(oss.str());
    }
    const std::size_t count =
        std::min(expected.rpdo[i].entries.size(),
                 actual.rpdo[i].entries.size());
    for (std::size_t j = 0; j < count; ++j) {
      if (expected.rpdo[i].entries[j] != actual.rpdo[i].entries[j]) {
        std::ostringstream oss;
        oss << "RPDO" << (i + 1) << " entry[" << j << "] mismatch: expected "
            << FormatEntry(expected.rpdo[i].entries[j]) << " actual "
            << FormatEntry(actual.rpdo[i].entries[j]);
        record(oss.str());
      }
    }
  }

  for (std::size_t i = 0; i < 4; ++i) {
    if (expected.tpdo[i].cob_id != actual.tpdo[i].cob_id) {
      std::ostringstream oss;
      oss << "TPDO" << (i + 1) << " COB-ID mismatch: expected 0x" << std::hex
          << expected.tpdo[i].cob_id << " actual 0x" << actual.tpdo[i].cob_id;
      record(oss.str());
    }
    if (expected.tpdo[i].entries.size() != actual.tpdo[i].entries.size()) {
      std::ostringstream oss;
      oss << "TPDO" << (i + 1) << " entry count mismatch: expected "
          << expected.tpdo[i].entries.size() << " actual "
          << actual.tpdo[i].entries.size();
      record(oss.str());
    }
    const std::size_t count =
        std::min(expected.tpdo[i].entries.size(),
                 actual.tpdo[i].entries.size());
    for (std::size_t j = 0; j < count; ++j) {
      if (expected.tpdo[i].entries[j] != actual.tpdo[i].entries[j]) {
        std::ostringstream oss;
        oss << "TPDO" << (i + 1) << " entry[" << j << "] mismatch: expected "
            << FormatEntry(expected.tpdo[i].entries[j]) << " actual "
            << FormatEntry(actual.tpdo[i].entries[j]);
        record(oss.str());
      }
    }
  }

  return !diffs || diffs->empty();
}

PdoMappingReader::~PdoMappingReader() {
  timeout_stop_.store(true);
  timeout_cv_.notify_all();
  if (timeout_thread_.joinable()) {
    if (timeout_thread_.get_id() == std::this_thread::get_id()) {
      timeout_thread_.detach();
    } else {
      timeout_thread_.join();
    }
  }
}

void PdoMappingReader::Start(lely::canopen::BasicDriver& driver,
                             DoneCallback cb,
                             std::chrono::milliseconds timeout) {
  if (finished_.load()) {
    return;
  }
  driver_ = &driver;
  done_cb_ = std::move(cb);
  BuildHeaderSteps();
  ScheduleNext();

  if (timeout.count() > 0) {
    timeout_stop_.store(false);
    std::weak_ptr<PdoMappingReader> weak = shared_from_this();
    timeout_thread_ = std::thread([weak, timeout]() {
      auto self = weak.lock();
      if (!self) {
        return;
      }

      std::unique_lock<std::mutex> lk(self->timeout_mtx_);
      const bool stopped = self->timeout_cv_.wait_for(
          lk, timeout, [self]() { return self->timeout_stop_.load(); });
      lk.unlock();
      if (!stopped) {
        self->Finish(false, "PDO verify timeout");
      }
    });
  }
}

void PdoMappingReader::BuildHeaderSteps() {
  steps_.clear();
  step_index_ = 0;
  phase_entries_ = false;

  for (uint16_t i = 0; i < 4; ++i) {
    ReadStep rpdo_cob;
    rpdo_cob.idx = static_cast<uint16_t>(0x1400 + i);
    rpdo_cob.sub = 1;
    rpdo_cob.on_value = [this, i](uint32_t value) {
      mapping_.rpdo[i].cob_id = value;
    };
    steps_.push_back(rpdo_cob);

    ReadStep rpdo_count;
    rpdo_count.idx = static_cast<uint16_t>(0x1600 + i);
    rpdo_count.sub = 0;
    rpdo_count.is_u8 = true;
    rpdo_count.on_value = [this, i](uint32_t value) {
      rpdo_counts_[i] = static_cast<uint8_t>(value);
    };
    steps_.push_back(rpdo_count);

    ReadStep tpdo_cob;
    tpdo_cob.idx = static_cast<uint16_t>(0x1800 + i);
    tpdo_cob.sub = 1;
    tpdo_cob.on_value = [this, i](uint32_t value) {
      mapping_.tpdo[i].cob_id = value;
    };
    steps_.push_back(tpdo_cob);

    ReadStep tpdo_count;
    tpdo_count.idx = static_cast<uint16_t>(0x1A00 + i);
    tpdo_count.sub = 0;
    tpdo_count.is_u8 = true;
    tpdo_count.on_value = [this, i](uint32_t value) {
      tpdo_counts_[i] = static_cast<uint8_t>(value);
    };
    steps_.push_back(tpdo_count);
  }
}

void PdoMappingReader::BuildEntrySteps() {
  steps_.clear();
  step_index_ = 0;
  phase_entries_ = true;

  for (uint16_t i = 0; i < 4; ++i) {
    mapping_.rpdo[i].entries.clear();
    mapping_.rpdo[i].entries.reserve(rpdo_counts_[i]);
    for (uint8_t sub = 1; sub <= rpdo_counts_[i]; ++sub) {
      ReadStep step;
      step.idx = static_cast<uint16_t>(0x1600 + i);
      step.sub = sub;
      step.on_value = [this, i](uint32_t value) {
        mapping_.rpdo[i].entries.push_back(value);
      };
      steps_.push_back(step);
    }
  }

  for (uint16_t i = 0; i < 4; ++i) {
    mapping_.tpdo[i].entries.clear();
    mapping_.tpdo[i].entries.reserve(tpdo_counts_[i]);
    for (uint8_t sub = 1; sub <= tpdo_counts_[i]; ++sub) {
      ReadStep step;
      step.idx = static_cast<uint16_t>(0x1A00 + i);
      step.sub = sub;
      step.on_value = [this, i](uint32_t value) {
        mapping_.tpdo[i].entries.push_back(value);
      };
      steps_.push_back(step);
    }
  }
}

void PdoMappingReader::ScheduleNext() {
  if (finished_.load()) {
    return;
  }
  if (step_index_ >= steps_.size()) {
    if (!phase_entries_) {
      BuildEntrySteps();
      ScheduleNext();
      return;
    }
    Finish(true, std::string());
    return;
  }

  const ReadStep step = steps_[step_index_];
  if (step.is_u8) {
    driver_->SubmitRead<uint8_t>(
        step.idx, step.sub,
        [this, step](uint8_t, uint16_t, uint8_t, std::error_code ec,
                     uint8_t value) {
          if (ec) {
            std::ostringstream oss;
            oss << "SDO read failed at 0x" << std::hex << step.idx << ":"
                << std::dec << static_cast<int>(step.sub);
            Finish(false, oss.str());
            return;
          }
          if (step.on_value) {
            step.on_value(value);
          }
          ++step_index_;
          ScheduleNext();
        });
  } else {
    driver_->SubmitRead<uint32_t>(
        step.idx, step.sub,
        [this, step](uint8_t, uint16_t, uint8_t, std::error_code ec,
                     uint32_t value) {
          if (ec) {
            std::ostringstream oss;
            oss << "SDO read failed at 0x" << std::hex << step.idx << ":"
                << std::dec << static_cast<int>(step.sub);
            Finish(false, oss.str());
            return;
          }
          if (step.on_value) {
            step.on_value(value);
          }
          ++step_index_;
          ScheduleNext();
        });
  }
}

void PdoMappingReader::Finish(bool ok, const std::string& error) {
  if (finished_.exchange(true)) {
    return;
  }

  timeout_stop_.store(true);
  timeout_cv_.notify_all();
  if (timeout_thread_.joinable()) {
    if (timeout_thread_.get_id() == std::this_thread::get_id()) {
      timeout_thread_.detach();
    } else {
      timeout_thread_.join();
    }
  }

  error_ = error;
  if (done_cb_) {
    done_cb_(ok, error_, mapping_);
  }
}

}  // namespace canopen_hw
