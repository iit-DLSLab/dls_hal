#include "dls_hal/recovery_modes.hpp"
#include "dls_hal/robot_invariants.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace {

void validateLimitPair(const std::vector<double>& min_limit, const std::vector<double>& max_limit, const char* name) {
    if (min_limit.size() != max_limit.size()) {
        throw std::invalid_argument(std::string("Recovery ") + name + " limits: min/max size mismatch");
    }
    for (std::size_t i = 0; i < min_limit.size(); ++i) {
        if (std::isnan(min_limit[i]) || std::isnan(max_limit[i]) || min_limit[i] > max_limit[i]) {
            throw std::invalid_argument(std::string("Recovery ") + name + " limits: invalid range at joint " + std::to_string(i));
        }
    }
}

// Replaces non-finite values with fallback(i), then clamps to [min_limit, max_limit]
// where limits exist for that joint.
template <typename Fallback>
void sanitizeCommand(std::vector<double>& command, const std::vector<double>& min_limit,
                     const std::vector<double>& max_limit, Fallback fallback) {
    for (std::size_t i = 0; i < command.size(); ++i) {
        if (!std::isfinite(command[i])) {
            command[i] = fallback(i);
        }
        if (i < min_limit.size()) {
            command[i] = std::clamp(command[i], min_limit[i], max_limit[i]);
        }
    }
}

void zeroCommands(RobotCommands& commands) {
    std::fill(commands.position.begin(), commands.position.end(), 0.0);
    std::fill(commands.velocity.begin(), commands.velocity.end(), 0.0);
    std::fill(commands.torque.begin(), commands.torque.end(), 0.0);
}

} // namespace

RecoveryMode::~RecoveryMode() = default;

void RecoveryMode::setLimits(const RecoveryLimits& limits) {
    validateLimitPair(limits.position_min, limits.position_max, "position");
    validateLimitPair(limits.velocity_min, limits.velocity_max, "velocity");
    validateLimitPair(limits.torque_min, limits.torque_max, "torque");
    limits_ = limits;
}

void RecoveryMode::apply(const RobotState& state, RobotCommands& commands) {
    execute(state, commands);
    sanitize(state, commands);
}

void RecoveryMode::sanitize(const RobotState& state, RobotCommands& commands) const {
    sanitizeCommand(commands.position, limits_.position_min, limits_.position_max,
        [&state](std::size_t i) {
            return i < state.position.size() && std::isfinite(state.position[i]) ? state.position[i] : 0.0;
        });
    sanitizeCommand(commands.velocity, limits_.velocity_min, limits_.velocity_max,
        [](std::size_t) { return 0.0; });
    sanitizeCommand(commands.torque, limits_.torque_min, limits_.torque_max,
        [](std::size_t) { return 0.0; });
}

void RecoveryESTOP::execute(const RobotState&, RobotCommands& commands) {
    zeroCommands(commands);
}

RecoveryDamping::RecoveryDamping(double damping_gain)
    : damping_gain_(damping_gain) {
    if (!std::isfinite(damping_gain_) || damping_gain_ < 0.0) {
        throw std::invalid_argument("Recovery damping gain must be finite and non-negative");
    }
}

void RecoveryDamping::execute(const RobotState& state, RobotCommands& commands) {
    zeroCommands(commands);
    const auto joint_count = std::min(state.velocity.size(), commands.torque.size());
    for (std::size_t i = 0; i < joint_count; ++i) {
        commands.torque[i] = -damping_gain_ * state.velocity[i];
    }
}

void RecoveryVirtualJointLimit::init(size_t n_joint){
    if (limits_.position_min.size() != n_joint || limits_.position_max.size() != n_joint) {
        throw std::invalid_argument("Virtual joint limit: setLimits() must provide one position limit per joint before init()");
    }
    k_.assign(n_joint, 0.0);
    d_.assign(n_joint, 0.0);
    q_min_trigger_ = limits_.position_min;
    q_max_trigger_ = limits_.position_max;
}

void RecoveryVirtualJointLimit::reset(size_t index){
    if (index >= k_.size() || index >= d_.size()
        || index >= q_min_trigger_.size() || index >= q_max_trigger_.size()) {
        return;
    }
    k_[index] = 0.0;
    d_[index] = 0.0;
    q_min_trigger_[index] = limits_.position_min[index];
    q_max_trigger_[index] = limits_.position_max[index];
}
void RecoveryVirtualJointLimit::setParams(size_t index,
                                           const double& inertia, 
                                           const double& position, 
                                           const double& velocity, 
                                           const double& torque,
                                           const double& q_min_trigger,
                                           const double& q_max_trigger)
{

    if(index >= k_.size() || index >= d_.size() 
        || index >= q_min_trigger_.size() || index >= q_max_trigger_.size()){
        return;
    }

    const double q_min = limits_.position_min[index];
    const double q_max = limits_.position_max[index];

    // The side is the threshold the position crossed; the velocity sign only breaks the
    // tie when both are crossed (thresholds overlapping at high speed or saturated torque).
    const bool above = position > q_max_trigger;
    const bool below = position < q_min_trigger;
    if (!above && !below) {
        return;
    }
    const bool upper_limit = (above && below) ? velocity > 0.0 : above;
    const double toward_sign = upper_limit ? 1.0 : -1.0;

    // Only the velocity toward the limit has to be absorbed.
    const double considered_velocity = std::max(toward_sign * velocity, 0.0);

    // Anchor where the joint is now, kept within the position limits: the crossed
    // threshold itself can lie beyond the opposite limit when the torque saturates.
    const double anchor = std::clamp(position, q_min, q_max);

    constexpr double dq_eps = 1e-6; // Denominator regulation term
    const double dq_remaining = std::max(upper_limit ? q_max - anchor : anchor - q_min, dq_eps);

    const double tau_push = std::max(toward_sign * torque, 0.0);

    double new_k =
        (inertia * considered_velocity * considered_velocity)/ (dq_remaining * dq_remaining)
        + 2.0 * tau_push / dq_remaining;

    double new_d = 2.0 * std::sqrt(new_k * inertia);

    if (!std::isfinite(new_k) || !std::isfinite(new_d) || new_k < 0.0 || new_d < 0.0
        || !std::isfinite(anchor)) {
        return;
    }

    k_[index] = new_k;
    d_[index] = new_d;
    q_max_trigger_[index] = upper_limit ? anchor : q_max;
    q_min_trigger_[index] = upper_limit ? q_min : anchor;
}

void RecoveryVirtualJointLimit::execute(const RobotState& state, RobotCommands& commands){
    const auto joint_count = std::min({state.position.size(), state.velocity.size(), commands.torque.size(),
                                       limits_.position_min.size(), k_.size()});
    for (std::size_t id = 0; id < joint_count; ++id) {
        const double q = state.position[id];
        if (q > q_max_trigger_[id]) {
            commands.torque[id] += k_[id] * (q_max_trigger_[id] - q) - d_[id] * state.velocity[id];
        } else if (q < q_min_trigger_[id]) {
            commands.torque[id] += k_[id] * (q_min_trigger_[id] - q) - d_[id] * state.velocity[id];
        }
    }
}
