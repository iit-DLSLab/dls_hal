#include "dls_hal/robot_invariants.hpp"

#include <iostream>

IMU::IMU() {}

void IMU::init(){
    frame_id = "";
    orientation.fill(0.0);
    orientation_rpy.fill(0.0);
    orientation_covariance.fill(0.0);
    angular_velocity.fill(0.0);
    angular_velocity_covariance.fill(0.0);
    linear_acceleration.fill(0.0);
    linear_acceleration_covariance.fill(0.0);
}

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
    contacts.resize(config["general"]["n_contacts"].as<int>(), false);
    imu.resize(config["general"]["n_imus"].as<int>(), IMU());
    for (size_t i = 0; i < imu.size(); ++i) {
        imu[i].init();
    }
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

    const YAML::Node limits = config["safety"]["limits"];
    if (limits) {
        if (limits["position_min"]) position_min = limits["position_min"].as<std::vector<double>>();
        if (limits["position_max"]) position_max = limits["position_max"].as<std::vector<double>>();
        if (limits["velocity_max"]) velocity_max = limits["velocity_max"].as<std::vector<double>>();
        if (limits["torque_max"]) torque_max = limits["torque_max"].as<std::vector<double>>();
        if (limits["current_max"]) current_max = limits["current_max"].as<std::vector<double>>();
        if (limits["motor_temperature_min"]) motor_temperature_min = limits["motor_temperature_min"].as<std::vector<double>>();
        if (limits["motor_temperature_max"]) motor_temperature_max = limits["motor_temperature_max"].as<std::vector<double>>();
    }
}
