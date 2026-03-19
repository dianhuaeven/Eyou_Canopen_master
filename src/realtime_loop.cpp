#include "canopen_hw/realtime_loop.hpp"

#include <cmath>
#include <thread>

#include "canopen_hw/logging.hpp"

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#include <time.h>
#endif

namespace canopen_hw {

RealtimeLoop::RealtimeLoop(const Config& config) : config_(config) {}

void RealtimeLoop::Run(std::function<bool()> tick) {
  if (!tick) return;

#ifdef __linux__
  if (config_.use_fifo) {
    struct sched_param sp{};
    sp.sched_priority = config_.fifo_priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
      CANOPEN_LOG_WARN("RealtimeLoop: failed to set SCHED_FIFO (need root or CAP_SYS_NICE), falling back to CFS");
    }
  }

  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);

  const int64_t period_ns = config_.period.count();
  int64_t jitter_sum = 0;

  while (true) {
    // 推进到下一个绝对时间点
    next.tv_nsec += period_ns;
    while (next.tv_nsec >= 1000000000L) {
      next.tv_nsec -= 1000000000L;
      next.tv_sec += 1;
    }

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    const int64_t wake_ns = (now.tv_sec - next.tv_sec) * 1000000000L +
                            (now.tv_nsec - next.tv_nsec);
    const int64_t jitter_us = std::abs(wake_ns) / 1000;

    stats_.iterations++;
    stats_.last_jitter_us = jitter_us;
    if (jitter_us > stats_.max_jitter_us) {
      stats_.max_jitter_us = jitter_us;
    }
    jitter_sum += jitter_us;
    stats_.avg_jitter_us = jitter_sum / static_cast<int64_t>(stats_.iterations);

    if (!tick()) break;
  }

#else
  // 非 Linux 回退到 sleep_for
  int64_t jitter_sum = 0;

  while (true) {
    auto before = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(config_.period);
    auto after = std::chrono::steady_clock::now();

    const auto actual = std::chrono::duration_cast<std::chrono::microseconds>(
        after - before);
    const auto expected = std::chrono::duration_cast<std::chrono::microseconds>(
        config_.period);
    const int64_t jitter_us = std::abs(actual.count() - expected.count());

    stats_.iterations++;
    stats_.last_jitter_us = jitter_us;
    if (jitter_us > stats_.max_jitter_us) {
      stats_.max_jitter_us = jitter_us;
    }
    jitter_sum += jitter_us;
    stats_.avg_jitter_us = jitter_sum / static_cast<int64_t>(stats_.iterations);

    if (!tick()) break;
  }
#endif
}

LoopStats RealtimeLoop::stats() const {
  return stats_;
}

}  // namespace canopen_hw
