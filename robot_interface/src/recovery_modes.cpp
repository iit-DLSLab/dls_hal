#include "dls_hal/recovery_modes.hpp"
#include "dls_hal/robot_invariants.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

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

namespace {
    // Accept either one value shared by all joints or one value per joint.
    std::vector<double> perJointParameter(const YAML::Node& node, std::size_t n_joints, const char* name) {
        std::vector<double> values;
        if (node.IsScalar()) {
            values.assign(n_joints, node.as<double>());
        } else if (node.IsSequence()) {
            values = node.as<std::vector<double>>();
        }
        if (values.size() != n_joints) {
            throw std::invalid_argument(std::string("joint_limit_hard_stop.") + name
                + " must be a scalar or a list with one value per joint");
        }
        return values;
    }

    void requirePositive(const std::vector<double>& values, const char* name) {
        for (double value : values) {
            if (!std::isfinite(value) || value <= 0.0) {
                throw std::invalid_argument(std::string("joint_limit_hard_stop.") + name
                    + " must be finite and positive");
            }
        }
    }

    double nonNegativeParameter(const YAML::Node& node, const char* name) {
        const double value = node ? node.as<double>() : 0.0;
        if (!std::isfinite(value) || value < 0.0) {
            throw std::invalid_argument(std::string("joint_limit_hard_stop.") + name
                + " must be finite and non-negative");
        }
        return value;
    }
} // namespace

void RecoveryJointLimitHardStop::init(const YAML::Node& config,
                                      const std::vector<double>& position_min,
                                      const std::vector<double>& position_max) {
    enabled_ = false;
    action_ = BrakingAction::NONE;
    const auto hard_stop = config["robot_interface"]["safety"]["joint_limit_hard_stop"];
    if (!hard_stop || !hard_stop["enabled"] || !hard_stop["enabled"].as<bool>()) {
        return;
    }

    const auto n_joints = static_cast<std::size_t>(config["general"]["n_joints"].as<int>());
    if (position_min.size() != n_joints || position_max.size() != n_joints) {
        throw std::invalid_argument("joint_limit_hard_stop requires one position limit per joint");
    }
    position_min_ = position_min;
    position_max_ = position_max;

    if (!hard_stop["braking_acceleration"]) {
        throw std::invalid_argument("joint_limit_hard_stop.braking_acceleration is required when enabled");
    }
    braking_acceleration_ = perJointParameter(hard_stop["braking_acceleration"], n_joints, "braking_acceleration");
    requirePositive(braking_acceleration_, "braking_acceleration");
    reaction_time_ = nonNegativeParameter(hard_stop["reaction_time"], "reaction_time");
    position_margin_ = nonNegativeParameter(hard_stop["position_margin"], "position_margin");

    // The braking command must go through the field the low-level interface sends.
    const auto mode = static_cast<CONTROL_MODE>(
        config["robot_interface"]["low_level_control"]["mode"].as<int>());
    switch (mode) {
        case CONTROL_MODE::TORQUE_CONTROL:
        case CONTROL_MODE::IMPEDANCE_CONTROL:
            action_ = BrakingAction::TORQUE;
            if (!hard_stop["braking_gain"]) {
                throw std::invalid_argument(
                    "joint_limit_hard_stop.braking_gain is required in torque and impedance control");
            }
            braking_gain_ = perJointParameter(hard_stop["braking_gain"], n_joints, "braking_gain");
            requirePositive(braking_gain_, "braking_gain");
            break;
        case CONTROL_MODE::VELOCITY_CONTROL:
            action_ = BrakingAction::VELOCITY;
            break;
        case CONTROL_MODE::POSITION_CONTROL:
            action_ = BrakingAction::POSITION;
            break;
        default:
            throw std::invalid_argument("joint_limit_hard_stop is not supported in the configured control mode");
    }
    enabled_ = true;
}

bool RecoveryJointLimitHardStop::isEnabled() const {
    return enabled_;
}

double RecoveryJointLimitHardStop::approachedLimit(std::size_t joint, double velocity) const {
    return velocity > 0.0 ? position_max_[joint] - position_margin_
                          : position_min_[joint] + position_margin_;
}

double RecoveryJointLimitHardStop::safeVelocity(std::size_t joint, double distance) const {
    // Largest speed that still stops within distance: solves
    // distance = v * reaction_time + v^2 / (2 * a) for v >= 0.
    const double a = braking_acceleration_[joint];
    const double reaction = a * reaction_time_;
    return -reaction + std::sqrt(reaction * reaction + 2.0 * a * std::max(distance, 0.0));
}

double RecoveryJointLimitHardStop::remainingDistance(std::size_t joint, double position, double velocity) const {
    // Negative once the joint is already at or past the approached limit.
    const double limit = approachedLimit(joint, velocity);
    return velocity > 0.0 ? limit - position : position - limit;
}

bool RecoveryJointLimitHardStop::isBrakingRequired(std::size_t joint, double position, double velocity) const {
    if (!enabled_ || joint >= position_max_.size() ||
        !std::isfinite(position) || !std::isfinite(velocity) || velocity == 0.0) {
        return false;
    }
    // Only motion toward the limit is braked, so a joint can always move back inside.
    const double distance = remainingDistance(joint, position, velocity);
    return distance < 0.0 || std::abs(velocity) > safeVelocity(joint, distance);
}

void RecoveryJointLimitHardStop::execute(const RobotState& state, RobotCommands& commands) {
    const auto joint_count = std::min({state.position.size(), state.velocity.size(), position_max_.size()});
    for (std::size_t i = 0; i < joint_count; ++i) {
        const double position = state.position[i];
        const double velocity = state.velocity[i];
        if (!isBrakingRequired(i, position, velocity)) continue;

        // Keep the direction of motion and reduce the speed to the envelope.
        const double target_velocity =
            std::copysign(safeVelocity(i, remainingDistance(i, position, velocity)), velocity);
        switch (action_) {
            case BrakingAction::TORQUE:
                if (i < commands.torque.size())
                    commands.torque[i] = -braking_gain_[i] * (velocity - target_velocity);
                break;
            case BrakingAction::VELOCITY:
                if (i < commands.velocity.size())
                    commands.velocity[i] = target_velocity;
                break;
            case BrakingAction::POSITION:
                // The drive's position loop sets the actual deceleration.
                if (i < commands.position.size())
                    commands.position[i] = approachedLimit(i, velocity);
                break;
            default:
                break;
        }
    }
}
