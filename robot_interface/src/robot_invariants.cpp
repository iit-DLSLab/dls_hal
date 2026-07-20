#include "dls_hal/robot_invariants.hpp"

#include <iostream>

IMU::IMU() {}
// RobotState
RobotState::RobotState() {}

void RobotState::init(const YAML::Node& config){
    int num_joints = config["general"]["n_joints"].as<int>();
    motor_status_word.resize(num_joints, 0);
    position.resize(num_joints, 0.0);
    velocity.resize(num_joints, 0.0);
    acceleration.resize(num_joints, 0.0);
    torques.resize(num_joints, 0.0);
    motor_temperature.resize(num_joints, 0.0);
    motor_current.resize(num_joints, 0.0);
    names.resize(num_joints, "");
    robot_name = "";
    battery_status = 100.0;
    imu.resize(config["general"]["n_imus"].as<int>(), IMU());
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
    control_mode = config["low_level_control"]["mode"].as<uint32_t>();
    position.resize(num_joints, 0.0);
    velocity.resize(num_joints, 0.0);
    torque.resize(num_joints, 0.0);
    kp.resize(num_joints, 0.0);
    ki.resize(num_joints, 0.0);
    kd.resize(num_joints, 0.0);
}

// RobotLimits
RobotLimits::RobotLimits() {}

void RobotLimits::init(const YAML::Node& config){
    int num_joints = config["general"]["n_joints"].as<int>();
    // no limits by default, set to extreme values
    position_min.resize(num_joints, -std::numeric_limits<double>::max());
    position_max.resize(num_joints, std::numeric_limits<double>::max());
    velocity_max.resize(num_joints, std::numeric_limits<double>::max());
    torque_max.resize(num_joints, std::numeric_limits<double>::max());
    current_max.resize(num_joints, std::numeric_limits<double>::max());
    motor_temperature_min.resize(num_joints, -std::numeric_limits<double>::max());
    motor_temperature_max.resize(num_joints, std::numeric_limits<double>::max());
    battery_charge_min = 0.0;
}