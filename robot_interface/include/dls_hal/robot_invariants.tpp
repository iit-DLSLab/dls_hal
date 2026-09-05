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
    e_stop_triggered = false;
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::init(const YAML::Node& config){
    e_stop_triggered = config["robot_interface"]["safety"]["e_stop"].as<int>();
    anomalies.joint_pos_limit_violation.resize(config["general"]["n_joints"].as<int>(), false);
    anomalies.joint_vel_limit_violation.resize(config["general"]["n_joints"].as<int>(), false);
    anomalies.joint_torque_limit_violation.resize(config["general"]["n_joints"].as<int>(), false);
    anomalies.nan_position_detected.resize(config["general"]["n_joints"].as<int>(), false);
    anomalies.nan_velocity_detected.resize(config["general"]["n_joints"].as<int>(), false);
    anomalies.nan_torque_detected.resize(config["general"]["n_joints"].as<int>(), false);
    anomalies.inf_position_detected.resize(config["general"]["n_joints"].as<int>(), false);
    anomalies.inf_velocity_detected.resize(config["general"]["n_joints"].as<int>(), false);
    anomalies.inf_torque_detected.resize(config["general"]["n_joints"].as<int>(), false);
    anomalies.motor_overheat_detected.resize(config["general"]["n_joints"].as<int>(), false);
    anomalies.battery_charge_low.resize(config["general"]["n_batteries"].as<int>(), false);
    limits.init(config);
}

ROBOT_SAFETY_TEMPLATE
bool ROBOT_SAFETY_TYPE::checkState(const TState& state) {
    // Check if state is within limits, and setting anomalies if necessary
    for (size_t i = 0; i < state.position.size(); ++i) {
        anomalies.joint_pos_limit_violation[i] = checkJointStateLimit(state.position[i], limits.position_min[i], limits.position_max[i]);
        anomalies.joint_vel_limit_violation[i] = checkJointStateLimit(state.velocity[i], -limits.velocity_max[i], limits.velocity_max[i]);
        anomalies.joint_torque_limit_violation[i] = checkJointStateLimit(state.torque[i], -limits.torque_max[i], limits.torque_max[i]);
        anomalies.motor_overheat_detected[i] = checkJointStateLimit(state.motor_temperature[i], limits.motor_temperature_min[i], limits.motor_temperature_max[i]);
        anomalies.nan_position_detected[i] = checkJointStateNanInf(state.position[i]);
        anomalies.inf_position_detected[i] = checkJointStateNanInf(state.position[i]);
        anomalies.nan_velocity_detected[i] = checkJointStateNanInf(state.velocity[i]);
        anomalies.inf_velocity_detected[i] = checkJointStateNanInf(state.velocity[i]);
        anomalies.nan_torque_detected[i] = checkJointStateNanInf(state.torque[i]);
        anomalies.inf_torque_detected[i] = checkJointStateNanInf(state.torque[i]);
    }

    for (size_t i = 0; i < state.battery.size(); ++i) {
        anomalies.battery_charge_low[i] = state.battery[i].charge_percentage < limits.percentage_battery_charge_min;
    }
    return true;
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
bool ROBOT_SAFETY_TYPE::checkReference(const TRef& ref) {
    // Check if reference is within limits, and clamping if necessary
    for (size_t i = 0; i < ref.position.size(); ++i) {
        if (ref.position[i] < limits.position_min[i] || ref.position[i] > limits.position_max[i]) {
            std::clamp(ref.position[i], limits.position_min[i], limits.position_max[i]);
            return true;
        }
        if (ref.velocity[i] < -limits.velocity_max[i] || ref.velocity[i] > limits.velocity_max[i]) {
            std::clamp(ref.velocity[i], -limits.velocity_max[i], limits.velocity_max[i]);
            return true;
        }
        if (ref.torque[i] < -limits.torque_max[i] || ref.torque[i] > limits.torque_max[i]) {
            std::clamp(ref.torque[i], -limits.torque_max[i], limits.torque_max[i]);
            return true;
        }
    }
    return true;
}

ROBOT_SAFETY_TEMPLATE
bool ROBOT_SAFETY_TYPE::checkCommands(const TCommands& commands) {
    // Check if commands are within limits, and clamping if necessary
    for (size_t i = 0; i < commands.position.size(); ++i) {
        if (commands.position[i] < limits.position_min[i] || commands.position[i] > limits.position_max[i]) {
            std::clamp(commands.position[i], limits.position_min[i], limits.position_max[i]);
            return true;
        }
        if (commands.velocity[i] < -limits.velocity_max[i] || commands.velocity[i] > limits.velocity_max[i]) {
            std::clamp(commands.velocity[i], -limits.velocity_max[i], limits.velocity_max[i]);
            return true;
        }
        if (commands.torque[i] < -limits.torque_max[i] || commands.torque[i] > limits.torque_max[i]) {
            std::clamp(commands.torque[i], -limits.torque_max[i], limits.torque_max[i]);
            return true;
        }
    }
    
    return true;
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::eSTOP(TCommands& commands) {
    // Set all commands to zero or safe values
    std::fill(commands.position.begin(), commands.position.end(), 0.0);
    std::fill(commands.velocity.begin(), commands.velocity.end(), 0.0);
    std::fill(commands.torque.begin(), commands.torque.end(), 0.0);
    std::cout << "Emergency stop activated. Commands set to safe values." << std::endl;
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
        if (anomalies.motor_overheat_detected[i])
            std::cout << "Joint " << i << " motor overheat detected." << std::endl;
    }
    if (anomalies.battery_charge_low)
        std::cout << "Battery charge is low." << std::endl;
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