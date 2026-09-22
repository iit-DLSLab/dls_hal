#include "dls_hal/recovery_modes.hpp"
#include "dls_hal/robot_invariants.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

RecoveryMode::~RecoveryMode() = default;

void RecoveryESTOP::run(const RobotState&, RobotCommands& commands) {
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

void RecoveryDamping::run(const RobotState& state, RobotCommands& commands) {
    RecoveryESTOP{}.run(state, commands);
    const auto joint_count = std::min(state.velocity.size(), commands.torque.size());
    for (std::size_t i = 0; i < joint_count; ++i) {
        const double torque = -damping_gain_ * state.velocity[i];
        if (std::isfinite(torque)) {
            commands.torque[i] = torque;
        }
    }
}
