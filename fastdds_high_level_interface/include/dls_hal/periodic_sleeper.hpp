#pragma once

#include <chrono>
#include <stdexcept>
#include <thread>

// Paces a loop at a fixed period, like SchedulerUtils::executeEndLoopTask().
// Sleeps until absolute deadlines, so the work time does not add to the
// period, and skips the expired slots after an overrun instead of running a
// burst of catch-up iterations.
class PeriodicSleeper {
public:
    explicit PeriodicSleeper(std::chrono::duration<double> period)
        : period_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(period))
        , next_wakeup_(std::chrono::steady_clock::now())
    {
        if (period_ <= std::chrono::steady_clock::duration::zero())
            throw std::invalid_argument("PeriodicSleeper: period must be positive");
    }

    // Sleep until the next period boundary.
    void sleep()
    {
        next_wakeup_ += period_;
        const auto now = std::chrono::steady_clock::now();
        if (now > next_wakeup_)
            next_wakeup_ += period_ * ((now - next_wakeup_) / period_ + 1);
        std::this_thread::sleep_until(next_wakeup_);
    }

private:
    std::chrono::steady_clock::duration period_;
    std::chrono::steady_clock::time_point next_wakeup_;
};
