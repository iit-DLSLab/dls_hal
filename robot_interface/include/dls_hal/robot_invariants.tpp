#include "robot_invariants.hpp"
#include <iostream>
#include <stdexcept>

#define ROBOT_SAFETY_TEMPLATE template <typename TState, typename TRef, typename TCommands, typename TLimits>
#define ROBOT_SAFETY_TYPE RobotSafety<TState, TRef, TCommands, TLimits>

#define ROBOT_LLC_TEMPLATE template <typename TState, typename TRef, typename TCommands>
#define ROBOT_LLC_TYPE RobotLowLevelControl<TState, TRef, TCommands>

// RobotSafety
ROBOT_SAFETY_TEMPLATE
ROBOT_SAFETY_TYPE::RobotSafety(){
    static_assert(std::is_base_of<RobotState, TState>::value, "TState must be derived from RobotState");
    static_assert(std::is_base_of<RobotRef, TRef>::value, "TRef must be derived from RobotRef");
    static_assert(std::is_base_of<RobotCommands, TCommands>::value, "TCommands must be derived from RobotCommands");
    static_assert(std::is_base_of<RobotLimits, TLimits>::value, "TLimits must be derived from RobotLimits");
    anomalies.emergency_stop_triggered.store(false, std::memory_order_relaxed);
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::init(const YAML::Node& config){
    anomalies.emergency_stop_triggered.store(false, std::memory_order_relaxed);
    const auto references_timeout_node = config["robot_interface"]["references_timeout"];
    references_timeout = references_timeout_node ? references_timeout_node.as<double>() : 1.0;
    if (!std::isfinite(references_timeout) || references_timeout <= 0.0)
        throw std::invalid_argument("robot_interface.references_timeout must be finite and positive (seconds)");
    const auto events_frequency_node = config["robot_interface"]["events_frequency"];
    events_frequency = events_frequency_node ? events_frequency_node.as<double>() : 10.0;
    if (!std::isfinite(events_frequency) || events_frequency <= 0.0)
        throw std::invalid_argument("robot_interface.events_frequency must be finite and strictly positive (Hz)");
    const auto n_joint = config["general"]["n_joints"].as<int>();
    anomalies.joint_pos_limit_violation.resize(n_joint, false);
    anomalies.joint_vel_limit_violation.resize(n_joint, false);
    anomalies.joint_current_limit_violation.resize(n_joint, false);
    anomalies.joint_torque_limit_violation.resize(n_joint, false);
    anomalies.nan_position_detected.resize(n_joint, false);
    anomalies.nan_velocity_detected.resize(n_joint, false);
    anomalies.nan_torque_detected.resize(n_joint, false);
    anomalies.inf_position_detected.resize(n_joint, false);
    anomalies.inf_velocity_detected.resize(n_joint, false);
    anomalies.inf_torque_detected.resize(n_joint, false);
    anomalies.joint_pos_clamping_anomaly.assign(n_joint, false);
    anomalies.joint_vel_clamping_anomaly.assign(n_joint, false);
    anomalies.joint_torque_clamping_anomaly.assign(n_joint, false);
    anomalies.joint_pos_command_trigger_value.assign(n_joint, 0.0);
    anomalies.joint_vel_command_trigger_value.assign(n_joint, 0.0);
    anomalies.joint_torque_command_trigger_value.assign(n_joint, 0.0);
    anomalies.nan_position_command_detected.assign(n_joint, false);
    anomalies.nan_velocity_command_detected.assign(n_joint, false);
    anomalies.nan_torque_command_detected.assign(n_joint, false);
    anomalies.inf_position_command_detected.assign(n_joint, false);
    anomalies.inf_velocity_command_detected.assign(n_joint, false);
    anomalies.inf_torque_command_detected.assign(n_joint, false);
    anomalies.motor_under_temperature_detected.resize(n_joint, false);
    anomalies.motor_overheat_detected.resize(n_joint, false);
    anomalies.battery_charge_low.resize(config["general"]["n_batteries"].as<int>(), false);
    anomalies.fungo_pressed.store(false, std::memory_order_relaxed);
    mechanical_stress_start_per_joint.assign(n_joint, std::nullopt);
    auto mechanical_stress_max_torque_node = config["robot_interface"]["safety"]["mechanical_stress_max_torque"];
    if (mechanical_stress_max_torque_node) {
        mechanical_stress_max_torque = mechanical_stress_max_torque_node.as<double>();
    } else {
        mechanical_stress_max_torque = 0.0; // default value if not specified
    }
    // Mechanical-stress detection is disabled when exponential coefficients are absent.
    mechanical_stress_exp_time_scale = 0.0;
    mechanical_stress_exp_decay = 0.0;
    const auto coefficients = config["robot_interface"]["safety"]["mechanical_stress_exp_coefficients"];
    if (coefficients) {
        const double a = coefficients["a"].as<double>();
        const double b = coefficients["b"].as<double>();
        if (!std::isfinite(a) || a <= 0.0 || !std::isfinite(b) || b < 0.0)
            throw std::invalid_argument("Mechanical stress coefficients require finite a > 0 and b >= 0");
        mechanical_stress_exp_time_scale = a;
        mechanical_stress_exp_decay = b;
    }
    if (!std::isfinite(mechanical_stress_max_torque) || mechanical_stress_max_torque < 0.0)
        throw std::invalid_argument("Mechanical stress torque threshold must be finite and non-negative");
    anomalies.breaking_mechanics = false;
    mechanical_stress_trigger_torque.assign(n_joint, std::nullopt);
    anomalies.lost_references.store(false, std::memory_order_relaxed);
    limits.init(config);
    
    const auto virtual_joint_limit = config["robot_interface"]["safety"]["virtual_joint_limit"];
    // Gains are either one value per joint or a scalar applied to every joint.
    const auto per_joint_gains = [n_joint](const YAML::Node& node) {
        return node.IsSequence() ? node.as<std::vector<double>>()
                                 : std::vector<double>(n_joint, node.as<double>());
    };
    recovery_virtual_joint_limit.init(per_joint_gains(virtual_joint_limit["k"]), per_joint_gains(virtual_joint_limit["d"]));
    // Recovery modes keep their own copy: limits are fixed after init().
    const RecoveryLimits recovery_limits{
        limits.position_min, limits.position_max,
        limits.velocity_min, limits.velocity_max,
        limits.torque_min, limits.torque_max};
    recovery_estop.setLimits(recovery_limits);
    recovery_damping.setLimits(recovery_limits);
    recovery_virtual_joint_limit.setLimits(recovery_limits);

    safety_state = SafetyState::NOMINAL;
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::checkState(const TState& state) {
    setFungoPressed(false);
    anomalies.breaking_mechanics = false;
    // Check if state is within limits, and setting anomalies if necessary
    for (size_t i = 0; i < state.position.size(); ++i) {
        anomalies.joint_pos_limit_violation[i] = checkJointStateLimit(state.position[i], limits.position_min[i], limits.position_max[i]);
        anomalies.joint_vel_limit_violation[i] = checkJointStateLimit(state.velocity[i], limits.velocity_min[i], limits.velocity_max[i]);
        anomalies.joint_torque_limit_violation[i] = checkJointStateLimit(state.torque[i], limits.torque_min[i], limits.torque_max[i]);
        anomalies.joint_current_limit_violation[i] = checkJointStateLimit(state.motors_current[i], limits.current_min[i], limits.current_max[i]);
        anomalies.motor_overheat_detected[i] = state.motor_temperature[i] > limits.motor_temperature_max[i];
        anomalies.motor_under_temperature_detected[i] = state.motor_temperature[i] < limits.motor_temperature_min[i];
        anomalies.nan_position_detected[i] = std::isnan(state.position[i]);
        anomalies.inf_position_detected[i] = std::isinf(state.position[i]);
        anomalies.nan_velocity_detected[i] = std::isnan(state.velocity[i]);
        anomalies.inf_velocity_detected[i] = std::isinf(state.velocity[i]);
        anomalies.nan_torque_detected[i] = std::isnan(state.torque[i]);
        anomalies.inf_torque_detected[i] = std::isinf(state.torque[i]);
        if (state.motors_status_word[i] == state.const_motors_status_words.fungo_pressed) {
            setFungoPressed(true);
        }
        checkBreakingMechanics(i, state.torque[i]);
    }
    for (size_t i = 0; i < state.battery.size(); ++i) {
        anomalies.battery_charge_low[i] = state.battery[i].charge_percentage < limits.percentage_battery_charge_min;
    }
}
ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::checkBreakingMechanics(size_t joint_index, double torque) {
    const auto now = std::chrono::steady_clock::now();
    auto& stress_start = mechanical_stress_start_per_joint[joint_index];
    if (mechanical_stress_exp_time_scale > 0.0 &&
        anomalies.joint_pos_limit_violation[joint_index] &&
        std::abs(torque) > mechanical_stress_max_torque) {
        if (!stress_start) {
            stress_start = now;
        }
        const double stress_time = std::chrono::duration<double>(now - *stress_start).count();
        const double excess_torque = std::abs(torque) - mechanical_stress_max_torque;
        // Use current torque: higher loads shorten the allowable continuous exposure.
        // Keep b = 0 explicit to avoid 0 * infinity for invalid torque samples.
        const double allowed_time = mechanical_stress_exp_decay == 0.0
            ? mechanical_stress_exp_time_scale
            : mechanical_stress_exp_time_scale * std::exp(-mechanical_stress_exp_decay * excess_torque);
        if (stress_time > allowed_time) {
            auto& trigger_torque = mechanical_stress_trigger_torque[joint_index];
            if (!trigger_torque) trigger_torque = torque;
            anomalies.breaking_mechanics = true;
        }
    } else {
        stress_start.reset();
    }
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::setFungoPressed(bool pressed) {
    anomalies.fungo_pressed.store(pressed, std::memory_order_relaxed);
}

ROBOT_SAFETY_TEMPLATE
bool ROBOT_SAFETY_TYPE::checkJointStateLimit(double value, double min_limit, double max_limit) {
    if (value < min_limit || value > max_limit) {
        return true;
    } else {
        return false;
    }
}

ROBOT_SAFETY_TEMPLATE
bool ROBOT_SAFETY_TYPE::checkJointStateNanInf(double value) {
    if (std::isnan(value) || std::isinf(value)) {
        return true;
    } else {
        return false;
    }
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::checkCommand(std::vector<double>& command , std::vector<double> min_limit, std::vector<double> max_limit, std::vector<bool>& anomaly_nan, std::vector<bool>& anomaly_inf, std::vector<bool>& anomaly_limit_min, std::vector<bool>& anomaly_limit_max, std::vector<double>& trigger_value) {
    bool is_valid = true;
    for (size_t i = 0; i < command.size(); ++i) {
        // init anomalies to false
        anomaly_nan[i] = false;
        anomaly_inf[i] = false;
        anomaly_limit_min[i] = false;
        anomaly_limit_max[i] = false;
        if (std::isnan(command[i])) {
            anomaly_nan[i] = true;
            is_valid = false;
        }
        else if (std::isinf(command[i])) {
            anomaly_inf[i] = true;
            is_valid = false;
        }
        else if (command[i] < min_limit[i]) {
            anomaly_limit_min[i] = true;
            // Record the pre-clamp command that triggered this: it, not the
            // post-clamp value, is what the caller actually asked for.
            trigger_value[i] = command[i];
            command[i] = std::clamp(command[i], min_limit[i], max_limit[i]);
        }
        else if (command[i] > max_limit[i]) {
            anomaly_limit_max[i] = true;
            trigger_value[i] = command[i];
            command[i] = std::clamp(command[i], min_limit[i], max_limit[i]);
        }
    }
    if (!is_valid) {
        triggerEmergencyStop();
    }
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::checkCommands(TCommands& commands) {
    checkCommand(commands.position, limits.position_min, limits.position_max, anomalies.nan_position_command_detected, anomalies.inf_position_command_detected, anomalies.joint_pos_clamping_anomaly, anomalies.joint_pos_clamping_anomaly, anomalies.joint_pos_command_trigger_value);
    checkCommand(commands.velocity, limits.velocity_min, limits.velocity_max, anomalies.nan_velocity_command_detected, anomalies.inf_velocity_command_detected, anomalies.joint_vel_clamping_anomaly, anomalies.joint_vel_clamping_anomaly, anomalies.joint_vel_command_trigger_value);
    checkCommand(commands.torque, limits.torque_min, limits.torque_max, anomalies.nan_torque_command_detected, anomalies.inf_torque_command_detected, anomalies.joint_torque_clamping_anomaly, anomalies.joint_torque_clamping_anomaly, anomalies.joint_torque_command_trigger_value);
}


ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::displayAnomalies() const {
    for (size_t i = 0; i < anomalies.joint_pos_limit_violation.size(); ++i) {
        if (anomalies.joint_pos_limit_violation[i])
            std::cout << "Joint " << i << " position limit violation detected." << std::endl;
        if (anomalies.joint_vel_limit_violation[i])
            std::cout << "Joint " << i << " velocity limit violation detected." << std::endl;
        if (anomalies.joint_torque_limit_violation[i])
            std::cout << "Joint " << i << " torque limit violation detected." << std::endl;
        if (anomalies.nan_position_detected[i])
            std::cout << "Joint " << i << " NaN position detected." << std::endl;
        if (anomalies.inf_position_detected[i])
            std::cout << "Joint " << i << " Inf position detected." << std::endl;
        if (anomalies.motor_under_temperature_detected[i])
            std::cout << "Motor " << i << " temperature is below minimum." << std::endl;
        if (anomalies.motor_overheat_detected[i])
            std::cout << "Joint " << i << " motor overheat detected." << std::endl;
    }
    if (std::any_of(anomalies.battery_charge_low.begin(), anomalies.battery_charge_low.end(),
                    [](bool low) { return low; }))
        std::cout << "Battery charge is low." << std::endl;
}


ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::triggerEmergencyStop() {
    anomalies.emergency_stop_triggered.store(true, std::memory_order_relaxed);
}

ROBOT_SAFETY_TEMPLATE
bool ROBOT_SAFETY_TYPE::isEmergencyStopActive() const {
    return anomalies.emergency_stop_triggered.load(std::memory_order_relaxed);
}

ROBOT_SAFETY_TEMPLATE
bool ROBOT_SAFETY_TYPE::isFungoPressed() const {
    return anomalies.fungo_pressed.load(std::memory_order_relaxed);
}

ROBOT_SAFETY_TEMPLATE
double ROBOT_SAFETY_TYPE::getReferencesTimeout() const {
    return references_timeout;
}

ROBOT_SAFETY_TEMPLATE
double ROBOT_SAFETY_TYPE::getEventsFrequency() const {
    return events_frequency;
}

ROBOT_SAFETY_TEMPLATE
const std::vector<std::optional<double>>& ROBOT_SAFETY_TYPE::getMechanicalStressTriggerTorque() const {
    return mechanical_stress_trigger_torque;
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::manageAnomalies(const TState& state, TCommands& commands) {
    checkState(state);
    checkCommands(commands);
    // set safety state machine based on the anomalies detected
    setSafetyState();
    // Debug: report safety state transitions only (this runs every control cycle).
    static SafetyState last_safety_state = SafetyState::NOMINAL;
    if (safety_state != last_safety_state) {
        std::cout << "[safety] state " << static_cast<int>(last_safety_state)
                  << " -> " << static_cast<int>(safety_state) << std::endl;
        for (size_t i = 0; i < anomalies.joint_pos_limit_violation.size(); ++i) {
            if (anomalies.joint_pos_limit_violation[i])
                std::cout << "[safety]   joint " << i << " q=" << state.position[i]
                          << " limits=[" << limits.position_min[i] << ", " << limits.position_max[i] << "]" << std::endl;
        }
        last_safety_state = safety_state;
    }
    // execute the corresponding recovery mode based on the safety state machine
    if(safety_state != SafetyState::NOMINAL){
        executeRecoveryMode(state, commands);
    }
}

ROBOT_SAFETY_TEMPLATE
const TLimits& ROBOT_SAFETY_TYPE::getLimits() const {
    return limits;
}

ROBOT_SAFETY_TEMPLATE
typename ROBOT_SAFETY_TYPE::SafetyState ROBOT_SAFETY_TYPE::getSafetyState() const {
    return safety_state;
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::setSafetyState() {
    if (anomalies.emergency_stop_triggered.load(std::memory_order_relaxed) || 
        anomalies.fungo_pressed.load(std::memory_order_relaxed)) {
        safety_state = SafetyState::EMERGENCY_STOP;
    }else if (std::any_of(anomalies.joint_pos_limit_violation.begin(), anomalies.joint_pos_limit_violation.end(),
                           [](bool violated) { return violated; })) {
        safety_state = SafetyState::VIRTUAL_ENDSTOP_ENFORCING;
    }else if (anomalies.breaking_mechanics || anomalies.lost_references.load(std::memory_order_relaxed)) {
        safety_state = SafetyState::DAMPING;
    } else {
        safety_state = SafetyState::NOMINAL;
    }
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::executeRecoveryMode(const TState& state, TCommands& commands) {
    switch (safety_state) {
        case SafetyState::EMERGENCY_STOP:
            recovery_estop.apply(state, commands);
            break;
        case SafetyState::DAMPING:
            recovery_damping.apply(state, commands);
            break;
        case SafetyState::VIRTUAL_ENDSTOP_ENFORCING:
            recovery_virtual_joint_limit.apply(state, commands);
            break;
        default:
            break;
    }
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::setLostReferences(bool lost) {
    anomalies.lost_references.store(lost, std::memory_order_relaxed);
}

// RobotLowLevelControl
ROBOT_LLC_TEMPLATE
ROBOT_LLC_TYPE::RobotLowLevelControl(){}

ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::init(const YAML::Node& config){
    // Initialization logic for low-level control based on the configuration
    control_mode = static_cast<CONTROL_MODE>(config["robot_interface"]["low_level_control"]["mode"].as<int>());
    commands.init(config);
}

ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::computeCommands(const TState& state, const TRef& ref){
    // Implementation for computing commands based on state and reference
    if (control_mode == CONTROL_MODE::POSITION_CONTROL) {
        positionControl(state, ref);
    } else if (control_mode == CONTROL_MODE::VELOCITY_CONTROL) {
        velocityControl(state, ref);
    } else if (control_mode == CONTROL_MODE::TORQUE_CONTROL) {
        torqueControl(state, ref);
    } else if (control_mode == CONTROL_MODE::IMPEDANCE_CONTROL) {
        impedanceControl(state, ref);
    } else if (control_mode == CONTROL_MODE::CURRENT_CONTROL) {
        currentControl(state, ref);
    }
}

ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::positionControl(const TState& state, const TRef& ref) {
    commands.position = ref.position; // Simple position control logic    
}
ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::velocityControl(const TState& state, const TRef& ref) {
    commands.velocity = ref.velocity; // Simple velocity control logic
}
ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::torqueControl(const TState& state, const TRef& ref) {
    commands.torque = ref.torque; // Simple torque control logic
}
ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::impedanceControl(const TState& state, const TRef& ref) {
    // Impedance control logic (to be implemented)
    for (int id = 0; id < state.position.size(); ++id) {
        commands.torque[id] =   ref.torque[id] 
                                + ref.kp[id] * (ref.position[id] - state.position[id])
                                + ref.kd[id] * (ref.velocity[id] - state.velocity[id]);
    }
}
ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::currentControl(const TState& state, const TRef& ref) { 
    // Current control logic (to be implemented)
    std::cout << "Current control logic not implemented yet." << std::endl;
}

#undef ROBOT_SAFETY_TEMPLATE
#undef ROBOT_SAFETY_TYPE
#undef ROBOT_LLC_TEMPLATE
#undef ROBOT_LLC_TYPE
