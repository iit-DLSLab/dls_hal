#include "dls_hal/fastdds_event_checker.hpp"

#include "dls_hal/generic_robot_interface.hpp"
#include "dls2/log/app_logger.hpp"
#include "dls2/log/event_config.hpp"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <sstream>

namespace
{
// Collect all affected joints before logging: AppLogger throttles by event ID.
std::string jointName(const RobotState& state, size_t i)
{
    auto name = i < state.names.size() && !state.names[i].empty()
        ? state.names[i] : "joint[" + std::to_string(i) + "]";
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return name;
}

template <typename Details>
std::string collectJointDetails(const RobotState& state, const std::vector<bool>& flags,
                                const Details& details)
{
    std::ostringstream data;
    bool first = true;
    for (size_t i = 0; i < flags.size(); ++i) {
        if (!flags[i]) continue;
        data << (first ? "\n" : ";\n");
        first = false;
        data << "Joint=" << jointName(state, i) << ", ";
        details(data, i);
    }
    return data.str();
}

template <typename LowerLimit>
void logJointLimit(dls::logging::AppLogger& logger, const RobotState& state,
                   const std::vector<bool>& flags, const std::vector<double>& values,
                   const char* quantity, const char* unit, const LowerLimit& lower,
                   const std::vector<double>& upper, EventID id)
{
    const auto message = collectJointDetails(state, flags, [&](std::ostream& out, size_t i) {
        out << quantity << "=" << values.at(i) << " " << unit
            << ", allowed=[" << lower(i) << ", " << upper.at(i) << "] " << unit;
    });
    if (!message.empty()) logger.warning(message, id);
}

// Commands outside their limits are clamped in place before being sent to
// hardware, so by the time this cycle's snapshot is taken the live command
// already holds the clamped (safe) value; trigger_values instead carries the
// pre-clamp value that actually caused the clamp (see
// RobotAnomalies::joint_*_command_trigger_value).
template <typename LowerLimit>
void logClampingAnomaly(dls::logging::AppLogger& logger, const RobotState& state,
                        const std::vector<bool>& flags, const std::vector<double>& trigger_values,
                        const char* quantity, const char* unit, const LowerLimit& lower,
                        const std::vector<double>& upper, EventID id)
{
    const auto message = collectJointDetails(state, flags, [&](std::ostream& out, size_t i) {
        out << "clamping anomaly on " << quantity << " command, triggering value="
            << trigger_values.at(i) << " " << unit
            << ", allowed=[" << lower(i) << ", " << upper.at(i) << "] " << unit;
    });
    if (!message.empty()) logger.warning(message, id);
}

// State and command anomalies share IDs; collect both before notifying.
template <typename Values>
std::string collectInvalidJointValues(const RobotState& state, const Values& values,
                                     const std::vector<bool>& position,
                                     const std::vector<bool>& velocity,
                                     const std::vector<bool>& torque, const char* source)
{
    const auto flagged = [](const std::vector<bool>& flags, size_t i) {
        return i < flags.size() && flags[i];
    };
    std::vector<bool> affected(std::max({position.size(), velocity.size(), torque.size()}), false);
    for (size_t i = 0; i < affected.size(); ++i)
        affected[i] = flagged(position, i) || flagged(velocity, i) || flagged(torque, i);
    return collectJointDetails(state, affected, [&](std::ostream& out, size_t i) {
        out << "source=" << source << ", ";
        if (flagged(position, i)) out << "position=" << values.position.at(i) << " rad";
        if (flagged(velocity, i)) {
            if (flagged(position, i)) out << ", ";
            out << "velocity=" << values.velocity.at(i) << " rad/s";
        }
        if (flagged(torque, i)) {
            if (flagged(position, i) || flagged(velocity, i)) out << ", ";
            out << "torque=" << values.torque.at(i) << " Nm";
        }
    });
}
} // namespace

FastDDSEventChecker::FastDDSEventChecker(const std::string& ID)
    : event_logger_(std::make_shared<dls::logging::AppLogger>(ID))
{}

void FastDDSEventChecker::setEventLogger(dls::logging::AppLogger& logger)
{
    // Borrow an externally owned logger without deleting it when the checker dies.
    event_logger_ = std::shared_ptr<dls::logging::AppLogger>(
        &logger, [](dls::logging::AppLogger*) {});
}

dls::logging::AppLogger& FastDDSEventChecker::eventLogger() const
{
    return *event_logger_;
}

void FastDDSEventChecker::check(const GenericRobotInterface& ri)
{
    checkJointLimitEvents(ri);
    checkInvalidValueEvents(ri);
    checkMotorFaultEvents(ri);
    checkMotorNotOperationalEvents(ri);
    checkBatteryEvents(ri);
    checkEmergencyAndRecoveryEvents(ri);
    checkVirtualEndstopEvents(ri);
    checkSafetyStateEvents(ri);
}

void FastDDSEventChecker::checkJointLimitEvents(const GenericRobotInterface& ri)
{
    const auto* state = &ri.state;
    const auto* limits = &ri.safety.getLimits();
    const auto* anomalies = &ri.safety.anomalies;

    logJointLimit(*event_logger_, *state, anomalies->joint_pos_limit_violation,
                  state->position, "position", "rad", [&](size_t i) { return limits->position_min.at(i); },
                  limits->position_max, EventID::JOINT_POS_OUT_OF_BOUND);
    logJointLimit(*event_logger_, *state, anomalies->joint_vel_limit_violation,
                  state->velocity, "velocity", "rad/s", [&](size_t i) { return limits->velocity_min.at(i); },
                  limits->velocity_max, EventID::JOINT_VEL_OUT_OF_BOUND);
    logJointLimit(*event_logger_, *state, anomalies->joint_torque_limit_violation,
                  state->torque, "torque", "Nm", [&](size_t i) { return limits->torque_min.at(i); },
                  limits->torque_max, EventID::JOINT_TORQUE_OUT_OF_BOUND);
    logClampingAnomaly(*event_logger_, *state, anomalies->joint_pos_clamping_anomaly,
                  anomalies->joint_pos_command_trigger_value, "position", "rad", [&](size_t i) { return limits->position_min.at(i); },
                  limits->position_max, EventID::JOINT_POS_CLAMPING_ANOMALY);
    logClampingAnomaly(*event_logger_, *state, anomalies->joint_vel_clamping_anomaly,
                  anomalies->joint_vel_command_trigger_value, "velocity", "rad/s", [&](size_t i) { return limits->velocity_min.at(i); },
                  limits->velocity_max, EventID::JOINT_VEL_CLAMPING_ANOMALY);
    logClampingAnomaly(*event_logger_, *state, anomalies->joint_torque_clamping_anomaly,
                  anomalies->joint_torque_command_trigger_value, "torque", "Nm", [&](size_t i) { return limits->torque_min.at(i); },
                  limits->torque_max, EventID::JOINT_TORQUE_CLAMPING_ANOMALY);
    logJointLimit(*event_logger_, *state, anomalies->joint_current_limit_violation,
                  state->motors_current, "current", "A", [&](size_t i) { return limits->current_min.at(i); },
                  limits->current_max, EventID::JOINT_CURRENT_OUT_OF_BOUND);
    logJointLimit(*event_logger_, *state, anomalies->motor_overheat_detected,
                  state->motor_temperature, "temperature", "degC", [&](size_t i) { return limits->motor_temperature_min.at(i); },
                  limits->motor_temperature_max, EventID::MOTOR_OVERHEAT);
    logJointLimit(*event_logger_, *state, anomalies->motor_under_temperature_detected,
                  state->motor_temperature, "temperature", "degC", [&](size_t i) { return limits->motor_temperature_min.at(i); },
                  limits->motor_temperature_max, EventID::MOTOR_UNDER_TEMPERATURE);
}

void FastDDSEventChecker::checkInvalidValueEvents(const GenericRobotInterface& ri)
{
    const auto* state = &ri.state;
    const auto* anomalies = &ri.safety.anomalies;

    const auto& commands = ri.low_level_control.commands;
    const auto nan_message = collectInvalidJointValues(*state, *state,
        anomalies->nan_position_detected, anomalies->nan_velocity_detected,
        anomalies->nan_torque_detected, "state")
        + collectInvalidJointValues(*state, commands,
        anomalies->nan_position_command_detected, anomalies->nan_velocity_command_detected,
        anomalies->nan_torque_command_detected, "command");
    const auto inf_message = collectInvalidJointValues(*state, *state,
        anomalies->inf_position_detected, anomalies->inf_velocity_detected,
        anomalies->inf_torque_detected, "state")
        + collectInvalidJointValues(*state, commands,
        anomalies->inf_position_command_detected, anomalies->inf_velocity_command_detected,
        anomalies->inf_torque_command_detected, "command");

    if (!nan_message.empty()) event_logger_->error(nan_message, EventID::NAN_VALUE);
    if (!inf_message.empty()) event_logger_->error(inf_message, EventID::INF_VALUE);
}

void FastDDSEventChecker::checkMotorFaultEvents(const GenericRobotInterface& ri)
{
    const auto& fault_info = ri.state.motor_fault_info;
    const auto message = collectJointDetails(ri.state, ri.safety.anomalies.motor_fault_detected,
                                             [&](std::ostream& out, size_t i) {
        const auto& info = fault_info.at(i);
        out << "fault_info=" << std::string(info.data(), strnlen(info.data(), info.size()));
    });
    if (!message.empty()) event_logger_->error("Motor fault detected:" + message, EventID::MOTOR_FAULT);
}

void FastDDSEventChecker::checkMotorNotOperationalEvents(const GenericRobotInterface& ri)
{
    const auto& status_word = ri.state.motors_status_word;
    const auto message = collectJointDetails(ri.state, ri.safety.anomalies.motor_not_operational_detected,
                                             [&](std::ostream& out, size_t i) {
        out << "status_word=0x" << std::hex << status_word.at(i) << std::dec;
    });
    if (!message.empty())
        event_logger_->error("Motor not in operational state:" + message, EventID::MOTOR_NOT_OPERATIONAL);
}

void FastDDSEventChecker::checkBatteryEvents(const GenericRobotInterface& ri)
{
    const auto* state = &ri.state;
    const auto* limits = &ri.safety.getLimits();
    const auto* anomalies = &ri.safety.anomalies;

    std::ostringstream batteries;
    bool low = false;
    for (size_t i = 0; i < anomalies->battery_charge_low.size(); ++i) {
        if (!anomalies->battery_charge_low[i]) continue;
        if (low) batteries << "; ";
        low = true;
        const auto& battery = state->battery.at(i);
        batteries << "battery[" << i << "] (id=" << battery.id << "): charge="
                  << battery.charge_percentage * 100.0 << "%, minimum="
                  << limits->percentage_battery_charge_min * 100.0 << "%";
    }
    if (low) event_logger_->error("Low battery charge: " + batteries.str(), EventID::BATTERY_CHARGE_LOW);
}

void FastDDSEventChecker::checkEmergencyAndRecoveryEvents(const GenericRobotInterface& ri)
{
    if (ri.safety.isFungoPressed())
        event_logger_->error("Fungo button is pressed", EventID::FUNGO_PRESSED);
    if (ri.safety.isEmergencyStopActive())
        event_logger_->error("Emergency stop triggered.", EventID::EMERGENCY_STOP_PRESSED);

    const auto& anomalies = ri.safety.anomalies;
    if (anomalies.lost_references.load(std::memory_order_relaxed))
        event_logger_->error("Control reference inputs were lost",
                              EventID::MISSING_INPUT);
}

// The virtual endstop overrides the high-level command of every joint it is
// braking, from the predicted violation until the joint is released.
void FastDDSEventChecker::checkVirtualEndstopEvents(const GenericRobotInterface& ri)
{
    VirtualJointLimitDebug endstop;
    ri.safety.getVirtualJointLimitDebug(endstop);

    std::vector<bool> active(endstop.side.size(), false);
    for (size_t i = 0; i < active.size(); ++i) active[i] = endstop.side[i] != 0;

    const auto& limits = ri.safety.getLimits();
    const auto message = collectJointDetails(ri.state, active, [&](std::ostream& out, size_t i) {
        const bool at_max = endstop.side[i] > 0;
        out << "position=" << ri.state.position.at(i) << " rad"
            << ", velocity=" << ri.state.velocity.at(i) << " rad/s"
            << ", braking toward " << (at_max ? "position_max=" : "position_min=")
            << (at_max ? limits.position_max.at(i) : limits.position_min.at(i)) << " rad"
            << ", anchor=" << endstop.anchor.at(i) << " rad"
            << ", torque=" << endstop.torque.at(i) << " Nm";
        if (endstop.damping_only.at(i)) out << ", damping only";
    });
    if (!message.empty())
        event_logger_->warning("Virtual endstop enforcing, high-level command overridden:" + message,
                               EventID::VIRTUAL_ENDSTOP_ENFORCING);
}

void FastDDSEventChecker::checkSafetyStateEvents(const GenericRobotInterface& ri)
{
    using SafetyState = decltype(ri.safety)::SafetyState;
    const auto safety_state = ri.safety.getSafetyState();
    // Only publish while the robot is not in its nominal safety state.
    if (safety_state == SafetyState::NOMINAL) return;

    const char* state_name = "UNKNOWN";
    switch (safety_state) {
        case SafetyState::EMERGENCY_STOP:
            state_name = "EMERGENCY_STOP";
            break;
        case SafetyState::DAMPING:
            state_name = "DAMPING";
            break;
        case SafetyState::VIRTUAL_ENDSTOP_ENFORCING:
            state_name = "VIRTUAL_ENDSTOP_ENFORCING";
            break;
        default:
            break;
    }
    event_logger_->error(std::string("Safety state is not nominal: ") + state_name + " mode is active.",
                           EventID::SAFETY_STATE_NOT_NOMINAL);
}
