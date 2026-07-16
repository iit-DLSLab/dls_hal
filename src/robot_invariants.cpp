#include "dls_hal/robot_invariants.hpp"

#include <iostream>

// RobotState
RobotState::RobotState() {}

void RobotState::init(const YAML::Node& config){
    int num_joints = config["general"]["n_joints"].as<int>();
    motor_status_word.resize(num_joints, 0);
    joints_position.resize(num_joints, 0.0);
    joints_velocity.resize(num_joints, 0.0);
    joints_acceleration.resize(num_joints, 0.0);
    joints_torques.resize(num_joints, 0.0);
    motor_temperature.resize(num_joints, 0.0);
    motor_current.resize(num_joints, 0.0);
}

// RobotCommands
RobotCommands::RobotCommands() {}

void RobotCommands::init(const YAML::Node& config){
    int num_joints = config["general"]["n_joints"].as<int>();
    position.resize(num_joints, 0.0);
    velocity.resize(num_joints, 0.0);
    torque.resize(num_joints, 0.0);
}

// RobotRef
RobotRef::RobotRef() {}

void RobotRef::init(const YAML::Node& config){
    int num_joints = config["general"]["n_joints"].as<int>();
    position.resize(num_joints, 0.0);
    velocity.resize(num_joints, 0.0);
    torque.resize(num_joints, 0.0);
}

// RobotLimits
RobotLimits::RobotLimits() {}

void RobotLimits::init(const YAML::Node& config){
    int num_joints = config["general"]["n_joints"].as<int>();
    position_min.resize(num_joints, -1.0);
    position_max.resize(num_joints, 1.0);
    velocity_max.resize(num_joints, 1.0);
    torque_max.resize(num_joints, 1.0);
    current_max.resize(num_joints, 1.0);
}

// // RobotSafety
// RobotSafety::RobotSafety(){
//     e_stop_triggered = false;
// }

// void RobotSafety::init(YAML::Node& config){
//     e_stop_triggered = config["Safety"]["e_stop"].as<int>();
// }

// bool RobotSafety::checkState(RobotState state, RobotLimits limits) { return true; }
// bool RobotSafety::checkReference(RobotRef ref, RobotLimits limits) { return true; }
// bool RobotSafety::checkCommands(RobotCommands commands, RobotLimits limits) { return true; }

// // RobotLowLevelControl
// RobotLowLevelControl::RobotLowLevelControl(){}

// void RobotLowLevelControl::init(YAML::Node& config){
//     // Initialization logic for low-level control based on the configuration
//     control_mode = static_cast<CONTROL_MODE>(config["LowLevelControl"]["mode"].as<int>());
//     commands.init(config);
// }

// void RobotLowLevelControl::computeCommands(RobotState state, RobotRef ref){
//     // Implementation for computing commands based on state and reference
//     if (control_mode == CONTROL_MODE::POSITION_CONTROL) {
//         positionControl();
//     } else if (control_mode == CONTROL_MODE::VELOCITY_CONTROL) {
//         velocityControl();
//     } else if (control_mode == CONTROL_MODE::TORQUE_CONTROL) {
//         torqueControl();
//     } else if (control_mode == CONTROL_MODE::IMPEDANCE_CONTROL) {
//         impedanceControl();
//     } else if (control_mode == CONTROL_MODE::CURRENT_CONTROL) {
//         currentControl();
//     }
// }

// void RobotLowLevelControl::positionControl() { std::cout << "Position control logic not implemented yet." << std::endl; }
// void RobotLowLevelControl::velocityControl() { std::cout << "Velocity control logic not implemented yet." << std::endl; }
// void RobotLowLevelControl::torqueControl() { std::cout << "Torque control logic not implemented yet." << std::endl; }
// void RobotLowLevelControl::impedanceControl() { std::cout << "Impedance control logic not implemented yet." << std::endl; }
// void RobotLowLevelControl::currentControl() { std::cout << "Current control logic not implemented yet." << std::endl; }
