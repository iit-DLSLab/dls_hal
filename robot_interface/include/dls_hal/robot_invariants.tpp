#include "robot_invariants.hpp"
#include <iostream>

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
    auto mechanical_stress_max_time_node = config["robot_interface"]["safety"]["mechanical_stress_max_time"];
    auto mechanical_stress_max_torque_node = config["robot_interface"]["safety"]["mechanical_stress_max_torque"];
    if (mechanical_stress_max_time_node) {
        mechanical_stress_max_time = mechanical_stress_max_time_node.as<double>();
    } else {
        mechanical_stress_max_time = -1.0; // disabled
    }
    if (mechanical_stress_max_torque_node) {
        mechanical_stress_max_torque = mechanical_stress_max_torque_node.as<double>();
    } else {
        mechanical_stress_max_torque = 0.0; // default value if not specified
    }
    anomalies.breaking_mechanics = false;
    anomalies.mechanical_stress_trigger_torque.assign(n_joint, std::nullopt);
    anomalies.references_not_received.store(false, std::memory_order_relaxed);
    limits.init(config);
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::checkState(const TState& state) {
    setFungoPressed(false);
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
    if (mechanical_stress_max_time > 0.0 &&
        anomalies.joint_pos_limit_violation[joint_index] &&
        std::abs(torque) > mechanical_stress_max_torque) {
        if (!stress_start) {
            stress_start = now;
        }
        const double stress_time = std::chrono::duration<double>(now - *stress_start).count();
        if (stress_time > mechanical_stress_max_time) {
            auto& trigger_torque = anomalies.mechanical_stress_trigger_torque[joint_index];
            if (!trigger_torque) trigger_torque = torque;
            anomalies.breaking_mechanics = true;
        }
    } else {
        // Both conditions must hold continuously for the configured duration.
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
void ROBOT_SAFETY_TYPE::checkCommand(std::vector<double>& command , std::vector<double> min_limit, std::vector<double> max_limit, std::vector<bool>& anomaly_nan, std::vector<bool>& anomaly_inf) {
    bool is_valid = true;
    for (size_t i = 0; i < command.size(); ++i) {
        
        if (std::isnan(command[i])) {
            anomaly_nan[i] = true;
            is_valid = false;
        }
        else if (std::isinf(command[i])) {
            anomaly_inf[i] = true;
            is_valid = false;
        }
        else if (command[i] < min_limit[i] || command[i] > max_limit[i]) {
            command[i] = std::clamp(command[i], min_limit[i], max_limit[i]);
        }
    }
    if (!is_valid) {
        triggerEmergencyStop();
    }
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::checkCommands(TCommands& commands) {
    checkCommand(commands.position, limits.position_min, limits.position_max, anomalies.nan_position_command_detected, anomalies.inf_position_command_detected);
    checkCommand(commands.velocity, limits.velocity_min, limits.velocity_max, anomalies.nan_velocity_command_detected, anomalies.inf_velocity_command_detected);
    checkCommand(commands.torque, limits.torque_min, limits.torque_max, anomalies.nan_torque_command_detected, anomalies.inf_torque_command_detected);
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
bool ROBOT_SAFETY_TYPE::manageAnomalies(const TState& state, TCommands& commands) {
    checkState(state);
    checkCommands(commands);
    // check which anomaly is active and handle it accordingly, using the corresponding recovery mode. The anomaly are checked based on their severity.
    // cases that trigger emergency stop
    if (isEmergencyStopActive() || isFungoPressed()) {
        recovery_estop.run(state,commands);
    }
    else if (anomalies.references_not_received.load(std::memory_order_relaxed)) {
        // Handle the case where references are not received
        recovery_damping.run(state,commands);
    }
    else if(anomalies.breaking_mechanics){
        recovery_damping.run(state,commands);
    }
    return false; // No anomaly detected
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::setReferencesNotReceived(bool not_received) {
    anomalies.references_not_received.store(not_received, std::memory_order_relaxed);
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
