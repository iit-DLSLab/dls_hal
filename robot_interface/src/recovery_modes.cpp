#include "dls_hal/recovery_modes.hpp"
#include "dls_hal/robot_invariants.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

RecoveryMode::~RecoveryMode() = default;

void RecoveryESTOP::execute(const RobotState&, RobotCommands& commands) {
    std::fill(commands.position.begin(), commands.position.end(), 0.0);
    std::fill(commands.velocity.begin(), commands.velocity.end(), 0.0);
    std::fill(commands.torque.begin(), commands.torque.end(), 0.0);
}

RecoveryDamping::RecoveryDamping(double damping_gain)
    : damping_gain_(damping_gain) {
    if (!std::isfinite(damping_gain_) || damping_gain_ < 0.0) {
        throw std::invalid_argument("Recovery damping gain must be finite and non-negative");
    }
}

void RecoveryDamping::execute(const RobotState& state, RobotCommands& commands) {
    RecoveryESTOP{}.execute(state, commands);
    const auto joint_count = std::min(state.velocity.size(), commands.torque.size());
    for (std::size_t i = 0; i < joint_count; ++i) {
        const double torque = -damping_gain_ * state.velocity[i];
        if (std::isfinite(torque)) {
            commands.torque[i] = torque;
        }
    }
}

void RecoveryVirtualJointLimit::init(const std::vector<double>& position_min, const std::vector<double>& position_max,
                                     const std::vector<double>& k, const std::vector<double>& d){
    const auto joint_count = position_min.size();
    if (position_max.size() != joint_count || k.size() != joint_count || d.size() != joint_count) {
        throw std::invalid_argument("Virtual joint limit position_min/position_max/k/d must have one entry per joint");
    }
    const auto valid_gain = [](double gain) { return std::isfinite(gain) && gain >= 0.0; };
    if (!std::all_of(k.begin(), k.end(), valid_gain) || !std::all_of(d.begin(), d.end(), valid_gain)) {
        throw std::invalid_argument("Virtual joint limit gains must be finite and non-negative");
    }
    position_min_ = position_min;
    position_max_ = position_max;
    k_ = k;
    d_ = d;
}

void RecoveryVirtualJointLimit::execute(const RobotState& state, RobotCommands& commands){
    const auto joint_count = std::min({state.position.size(), state.velocity.size(),
                                       commands.torque.size(), position_min_.size()});
    for (std::size_t id = 0; id < joint_count; ++id) {
        const double q = state.position[id];
        double torque = 0.0;
        if (q > position_max_[id]) {
            torque = k_[id] * (position_max_[id] - q) - d_[id] * state.velocity[id];
        } else if (q < position_min_[id]) {
            torque = k_[id] * (position_min_[id] - q) - d_[id] * state.velocity[id];
        }
        if (std::isfinite(torque)) {
            commands.torque[id] += torque;
        }
    }
}
