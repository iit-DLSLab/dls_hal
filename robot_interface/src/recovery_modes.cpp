#include "dls_hal/recovery_modes.hpp"
#include "dls_hal/robot_invariants.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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

void RecoveryVirtualJointLimit::init(size_t n_joint, double release_tolerance, bool damping_after_stop,
                                     double damping_ratio){
    if (limits_.position_min.size() != n_joint || limits_.position_max.size() != n_joint) {
        throw std::invalid_argument("Virtual joint limit: setLimits() must provide one position limit per joint before init()");
    }
    if (!std::isfinite(release_tolerance) || release_tolerance < 0.0) {
        throw std::invalid_argument("Virtual joint limit release tolerance must be finite and non-negative");
    }
    if (!std::isfinite(damping_ratio) || damping_ratio < 0.5 || damping_ratio > 1.0) {
        throw std::invalid_argument("Virtual joint limit damping ratio must be in [0.5, 1]");
    }
    k_.assign(n_joint, 0.0);
    d_.assign(n_joint, 0.0);
    q_min_trigger_ = limits_.position_min;
    q_max_trigger_ = limits_.position_max;
    side_.assign(n_joint, 0);
    damping_only_.assign(n_joint, false);
    release_tolerance_ = release_tolerance;
    damping_after_stop_ = damping_after_stop;
    damping_ratio_ = damping_ratio;
    // Underdamped (zeta < 1): x(t) = w / w_d * exp(-zeta * w_n * t) * sin(w_d * t), deepest at
    // w_d * t = arccos(zeta), i.e. w / w_n * exp(-zeta * arccos(zeta) / sqrt(1 - zeta^2)).
    // The constant-push step response overshoots tau / k by exp(-pi * zeta / sqrt(1 - zeta^2)).
    // Both tend to the critically damped values (1 / e and 1) as zeta -> 1.
    if (damping_ratio < 1.0) {
        const double s = std::sqrt(1.0 - damping_ratio * damping_ratio);
        penetration_factor_ = std::exp(-damping_ratio * std::acos(damping_ratio) / s);
        push_overshoot_ = 1.0 + std::exp(-M_PI * damping_ratio / s);
    } else {
        penetration_factor_ = 1.0 / M_E;
        push_overshoot_ = 1.0;
    }
}

double RecoveryVirtualJointLimit::getStoppingDistanceCoefficient() const {
    // Peak torque d * w = tau with d = 2 * zeta * sqrt(k * I) gives w_n = tau / (2 * zeta * I * w),
    // so the penetration w / w_n * penetration_factor is (2 * zeta * penetration_factor) * I * w^2 / tau.
    return 2.0 * damping_ratio_ * penetration_factor_;
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
    side_[index] = 0;
    damping_only_[index] = false;
}

bool RecoveryVirtualJointLimit::isActive(size_t index) const {
    return index < side_.size() && side_[index] != 0;
}

bool RecoveryVirtualJointLimit::anyActive() const {
    return std::any_of(side_.begin(), side_.end(), [](int side) { return side != 0; });
}

void RecoveryVirtualJointLimit::updateDampingOnly(size_t index, double velocity) {
    if (!damping_after_stop_ || !isActive(index) || damping_only_[index]) {
        return;
    }
    // Braking moves the joint toward the limit (side * velocity > 0); a sign change means it
    // stopped short of the limit and now moves away from it.
    if (side_[index] * velocity < 0.0) {
        damping_only_[index] = true;
    }
}

bool RecoveryVirtualJointLimit::isDampingOnly(size_t index) const {
    return index < damping_only_.size() && damping_only_[index];
}

const std::vector<bool>& RecoveryVirtualJointLimit::getDampingOnly() const {
    return damping_only_;
}

const std::vector<double>& RecoveryVirtualJointLimit::getK() const {
    return k_;
}

const std::vector<double>& RecoveryVirtualJointLimit::getD() const {
    return d_;
}

const std::vector<int>& RecoveryVirtualJointLimit::getSide() const {
    return side_;
}

double RecoveryVirtualJointLimit::getAnchor(size_t index) const {
    if (!isActive(index)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return side_[index] > 0 ? q_max_trigger_[index] : q_min_trigger_[index];
}

bool RecoveryVirtualJointLimit::hasReturnedToAnchor(size_t index, double position, double velocity) const {
    if (!isActive(index)) {
        return false;
    }
    const double side = side_[index];
    const double anchor = side > 0.0 ? q_max_trigger_[index] : q_min_trigger_[index];
    return side * (position - anchor) <= release_tolerance_ && side * velocity <= 0.0;
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

    // Nothing to absorb (e.g. past the limit but already moving back inside): activating
    // would latch zero gains and override the controller with zero torque.
    if (considered_velocity <= 0.0 && tau_push <= 0.0) {
        return;
    }

    // k_max: the stiffest gain the actuator can realize. The activation torque, spring on the
    // initial penetration p0 plus damping on w, is the peak of the braking (for zeta >= 0.5);
    // k_max makes it equal the torque available in the braking direction:
    //   p0 * k + 2 * zeta * sqrt(I) * w * sqrt(k) = tau_avail
    // A stiffer k would only be clamped downstream and chatter (-d * v saturating both ways).
    const double tau_avail = upper_limit ? std::abs(limits_.torque_min[index])
                                         : std::abs(limits_.torque_max[index]);
    const double p0 = std::max(toward_sign * (position - anchor), 0.0);
    const double b_term = 2.0 * damping_ratio_ * std::sqrt(inertia) * considered_velocity;
    const double sqrt_k_max = p0 > 0.0
        ? (-b_term + std::sqrt(b_term * b_term + 4.0 * p0 * tau_avail)) / (2.0 * p0)
        : (b_term > 0.0 ? tau_avail / b_term : std::numeric_limits<double>::infinity());

    // At or past the limit there is no room left to size k on: brake as hard as possible.
    const bool outside_limit = upper_limit ? position >= q_max : position <= q_min;

    double sqrt_k = sqrt_k_max;
    if (!outside_limit) {
        // Spring-damper anchored at the trigger, d = 2 * zeta * sqrt(k * I). From velocity w the
        // deepest penetration is w * sqrt(I / k) * p (p = penetration_factor_, 1 / e when
        // critically damped). A constant push tau adds at most o * tau / k (o = push_overshoot_).
        // k is chosen so that the joint stops exactly at the limit:
        //   w * sqrt(I / k) * p + o * tau / k = dq  ->  dq * k - (w * sqrt(I) * p) * sqrt(k) - o * tau = 0
        // (tau = 0, zeta = 1: k = I * w^2 / (e^2 * dq^2)). Still capped by k_max: close to the
        // limit dq is tiny and the distance-based k would exceed what the actuator can deliver.
        const double w_term = considered_velocity * std::sqrt(inertia) * penetration_factor_;
        const double push_term = push_overshoot_ * tau_push;
        const double sqrt_k_distance =
            (w_term + std::sqrt(w_term * w_term + 4.0 * dq_remaining * push_term))
            / (2.0 * dq_remaining);
        sqrt_k = std::min(sqrt_k_distance, sqrt_k_max);
    }

    const double new_k = sqrt_k * sqrt_k;
    const double new_d = 2.0 * damping_ratio_ * std::sqrt(new_k * inertia);

    if (!std::isfinite(new_k) || !std::isfinite(new_d) || new_k < 0.0 || new_d < 0.0
        || !std::isfinite(anchor)) {
        return;
    }

    k_[index] = new_k;
    d_[index] = new_d;
    q_max_trigger_[index] = upper_limit ? anchor : q_max;
    q_min_trigger_[index] = upper_limit ? q_min : anchor;
    side_[index] = upper_limit ? 1 : -1;
}

void RecoveryVirtualJointLimit::execute(const RobotState& state, RobotCommands& commands){
    const auto joint_count = std::min({state.position.size(), state.velocity.size(), commands.torque.size(),
                                       limits_.position_min.size(), side_.size()});
    for (std::size_t id = 0; id < joint_count; ++id) {
        // Only active joints are braked; the others keep their controller torque.
        if (side_[id] == 0) {
            continue;
        }
        // Latched damping only: the joint is no longer sprung back, wherever it is.
        if (damping_only_[id]) {
            commands.torque[id] = -d_[id] * state.velocity[id];
            continue;
        }
        const double q = state.position[id];
        const double anchor = side_[id] > 0 ? q_max_trigger_[id] : q_min_trigger_[id];
        // Inclusive: the anchor is the position at the trigger, so the triggering cycle
        // already brakes through the damping term (zero penetration, no spring force yet).
        // The spring-damper torque replaces the controller torque until the joint is back.
        if (side_[id] * (q - anchor) >= 0.0) {
            commands.torque[id] = k_[id] * (anchor - q) - d_[id] * state.velocity[id];
        }
    }
}
