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

RobotBattery::RobotBattery() {}

void RobotBattery::init(){
    v_batt = 0.0;
    v_load = 0.0;
    i_load = 0.0;
    temp_batt = 0.0;
    temp_heatsink = 0.0;
    temp_pcb = 0.0;
    status = 0;
    fault = 0;
    full_v_batt = 0.0;
    charge_percentage = 0.0;
    id = 0;
}

// RobotState
RobotState::RobotState() {}

void RobotState::init(const YAML::Node& config){
    int num_joints = config["general"]["n_joints"].as<int>();
    motors_status_word.resize(num_joints, 0);
    position.resize(num_joints, 0.0);
    velocity.resize(num_joints, 0.0);
    acceleration.resize(num_joints, 0.0);
    torque.resize(num_joints, 0.0);
    motor_temperature.resize(num_joints, 0.0);
    motors_current.resize(num_joints, 0.0);
    motors_status_word.resize(num_joints, 0);
    motor_fault_info.resize(num_joints, MotorFaultInfo{});
    names.resize(num_joints, "");
    robot_name = "";
    trunk_temperature = 0.0;
    contacts.resize(config["general"]["n_contacts"].as<int>(), false);
    battery.resize(config["general"]["n_batteries"].as<int>(), RobotBattery());
    const_motors_status_words.fungo_pressed = -1;
    const_motors_status_words.operational_mask = 0;
    
    for (size_t i = 0; i < battery.size(); ++i) {
        battery[i].init();
        battery[i].id = i;
        battery[i].full_v_batt = config["general"]["battery_full_voltage"].as<double>();
    }
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
    control_mode = config["robot_interface"]["low_level_control"]["mode"].as<uint32_t>();
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
    velocity_min.resize(num_joints, -std::numeric_limits<double>::max());
    velocity_max.resize(num_joints, std::numeric_limits<double>::max());
    torque_min.resize(num_joints, -std::numeric_limits<double>::max());
    torque_max.resize(num_joints, 0.0);
    current_min.resize(num_joints, 0.0);
    current_max.resize(num_joints, 0.0);
    motor_temperature_min.resize(num_joints, -std::numeric_limits<double>::max());
    motor_temperature_max.resize(num_joints, std::numeric_limits<double>::max());
    joint_inertia.resize(num_joints, 0.0);
    percentage_battery_charge_min = 0.0;

    const YAML::Node safety = config["robot_interface"]["safety"];
    const YAML::Node nested_limits = safety["limits"];
    // Older configurations place the values directly under safety, leaving
    // limits null. Read those values too instead of retaining zero defaults.
    const YAML::Node limits = nested_limits.IsDefined() && !nested_limits.IsNull()
        ? nested_limits : safety;
    if (limits) {
        if (limits["position_min_with_margin"]) position_min = limits["position_min_with_margin"].as<std::vector<double>>();
        if (limits["position_max_with_margin"]) position_max = limits["position_max_with_margin"].as<std::vector<double>>();
        if (limits["velocity_max"]) velocity_max = limits["velocity_max"].as<std::vector<double>>();
        if (limits["torque_max"]) torque_max = limits["torque_max"].as<std::vector<double>>();
        if (limits["current_max"]) current_max = limits["current_max"].as<std::vector<double>>();
        if (limits["motor_temperature_min"]) motor_temperature_min = limits["motor_temperature_min"].as<std::vector<double>>();
        if (limits["motor_temperature_max"]) motor_temperature_max = limits["motor_temperature_max"].as<std::vector<double>>();
        if (limits["joint_inertia"]) joint_inertia = limits["joint_inertia"].as<std::vector<double>>();
        if (limits["percentage_battery_charge_min"]) percentage_battery_charge_min = limits["percentage_battery_charge_min"].as<double>();
    }
    // Compute velocity, torque, and current minima from their maxima.
    for (size_t i = 0; i < velocity_max.size(); ++i) {
        velocity_min[i] = -velocity_max[i];
        torque_min[i] = -torque_max[i];
        current_min[i] = -current_max[i];
    }
}

void RobotRef::initReference(const RobotState& state){
    position = state.position;
}