#pragma once

// Included by fastdds_high_level_interface.hpp
#include "dls_hal/periodic_sleeper.hpp"
#include "dls2/topics/topics.hpp"
#include "dls2/domains/domains.hpp"
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <pthread.h>
#include <sched.h>

template <typename RI>
FastDDSHLInterface<RI>::FastDDSHLInterface(const std::string& ID)
    : event_checker_(ID)
    , command_manager(ID)
{}

template <typename RI>
FastDDSHLInterface<RI>::~FastDDSHLInterface(){
    stopThreads();
}

template <typename RI>
void FastDDSHLInterface<RI>::stopThreads(){
    event_thread_.request_stop(); // eventLoggingLoop() exits at most one event period later
    if (event_thread_.joinable())
        event_thread_.join();

    io_thread_.request_stop(); // ioLoop() exits at most one io period later
    if (io_thread_.joinable())
        io_thread_.join();
}

template <typename RI>
void FastDDSHLInterface<RI>::setEventLogger(dls::logging::AppLogger& logger)
{
    event_checker_.setEventLogger(logger);
}

template <typename RI>
void FastDDSHLInterface<RI>::checkEvents(const RI& source)
{
    event_checker_.check(source);
}

template <typename RI>
void FastDDSHLInterface<RI>::notifyStartupFailure(const RI& source)
{
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(source.safety.getStartupFailureNotifyDuration()));
    // Repeated so that subscribers matched after the first notification also receive it.
    PeriodicSleeper sleeper(std::chrono::duration<double>(1.0 / source.safety.getEventsFrequency()));
    do {
        event_checker_.checkMotorNotOperationalEvents(source);
        event_checker_.checkNodesNotDiscoveredEvents(source);
        sleeper.sleep();
    } while (std::chrono::steady_clock::now() < deadline);
}

template <typename RI>
void FastDDSHLInterface<RI>::init(RI& source, const YAML::Node& config){
    // input control signal
    input_control_signal = this->transport.template buildInput<dls2_interface::msg::ControlSignal>(
    dls::topics::control_signal);
    input_control_signal->setReceivingDataTimeout(std::chrono::duration<double>(source.safety.getReferencesTimeout()));
    output_blind_state = this->transport.template buildOutput<dls2_interface::msg::BlindState>(
    dls::topics::low_level_estimation::blind_state);
    output_imu = this->transport.template buildOutput<dls2_interface::msg::Imu>(
    dls::topics::low_level_estimation::imu);
    output_battery_status = this->transport.template buildOutput<dls2_interface::msg::BatteryStatus>(
    dls::topics::battery_status);
    output_hal_commands_feedback = this->transport.template buildOutput<dls2_interface::msg::ControlSignal>(
    dls::topics::hal_commands_feedback);
    output_hal_safety_debug = this->transport.template buildOutput<dls2_interface::msg::HalSafetyDebug>(
    dls::topics::hal_safety_debug);

    // init inputs
    input_control_signal->msg.joints_position().resize(source.ref.position.size(), 0.0);
    input_control_signal->msg.joints_velocity().resize(source.ref.velocity.size(), 0.0);
    input_control_signal->msg.joints_torques().resize(source.ref.torque.size(), 0.0);
    input_control_signal->msg.kp().resize(source.ref.kp.size(), 0.0);
    input_control_signal->msg.kd().resize(source.ref.kd.size(), 0.0);
    input_control_signal->msg.control_mode() = source.ref.control_mode;
    //init outputs
    output_blind_state->msg.joints_name().resize(source.state.names.size(), "");

    output_blind_state->msg.joints_position().resize(source.state.position.size(), 0.0);
    output_blind_state->msg.joints_velocity().resize(source.state.velocity.size(), 0.0);
    output_blind_state->msg.joints_acceleration().resize(source.state.acceleration.size(), 0.0);
    output_blind_state->msg.joints_effort().resize(source.state.torque.size(), 0.0);
    output_blind_state->msg.joints_temperature().resize(source.state.motor_temperature.size(), 0.0);
    output_blind_state->msg.feet_contact().resize(source.state.contacts.size(), false);
    output_blind_state->msg.motors_current().resize(source.state.motors_current.size(), 0.0);
    output_blind_state->msg.motors_status_word().resize(source.state.motors_status_word.size(), 0);
    output_blind_state->msg.motors_fault_info().resize(source.state.motor_fault_info.size(), "");
    output_blind_state->msg.trunk_temperature() = source.state.trunk_temperature;

    output_hal_commands_feedback->msg = input_control_signal->msg;

    // DDS reading/publishing runs in its own thread, decoupled from the HAL's
    // real-time frequency, since the transport can block on DDS I/O.
    if (config["io_frequency"]) {
        const double io_frequency = config["io_frequency"].as<double>();
        if (io_frequency <= 0.0)
            throw std::invalid_argument("FastDDSHLInterface: io_frequency must be positive");
        io_period_ = std::chrono::duration<double>(1.0 / io_frequency);
    }

    if (!io_thread_.joinable()) {
        // Must be initialized before the io thread starts filling it: copying
        // the source gives its reference vectors the robot's sizes.
        current_input_ = source;
        latest_output_ = source;
        std::cout << "Starting ioLoop HAL thread with period " << io_period_.count() << " seconds" << std::endl;
        io_thread_ = std::jthread([this](std::stop_token stop_token) { ioLoop(stop_token); });
    }

    // Event checking/logging runs in its own thread, decoupled from the HAL's
    // real-time frequency, since AppLogger can block on DDS/file I/O.
    // robot_interface.events_frequency is validated and defaulted by
    // RobotSafety::init(), which has already run by this point.
    const double events_frequency = source.safety.getEventsFrequency();
    event_check_period_ = std::chrono::duration<double>(1.0 / events_frequency);

    if (!event_thread_.joinable()) {
        event_thread_ = std::jthread([this](std::stop_token stop_token) { eventLoggingLoop(stop_token); });
    }
}

template <typename RI>
bool FastDDSHLInterface<RI>::check(){
    return this->transport.areInputsReceivingData() && this->transport.areOutputsUnique();
}

template <typename RI>
void FastDDSHLInterface<RI>::read(RI& source){
    // Only the newest reference matters: drain the queue and keep the last one.
    // If nothing arrived since the previous cycle, source keeps its reference.
    bool received = false;
    while (inputs_queue_.pop(latest_input_))
        received = true;
    if (!received)
        return;

    copyInputs(latest_input_, source);
}

template <typename RI>
void FastDDSHLInterface<RI>::copyInputs(const RI& input, RI& source){
    // Only the reference and the lost-references flag are filled by ioLoop().
    source.safety.setLostReferences(input.safety.anomalies.lost_references.load(std::memory_order_relaxed));
    source.ref.position = input.ref.position;
    source.ref.velocity = input.ref.velocity;
    source.ref.torque = input.ref.torque;
    source.ref.kp = input.ref.kp;
    source.ref.kd = input.ref.kd;
    source.ref.control_mode = input.ref.control_mode;
}

template <typename RI>
void FastDDSHLInterface<RI>::write(const RI& source){
    // Non-blocking: if ioLoop() falls behind and the queue is full, the sample is dropped.
    outputs_queue_.push(source);
}

template <typename RI>
void FastDDSHLInterface<RI>::ioLoop(std::stop_token stop_token){
    PeriodicSleeper sleeper(io_period_);

    while (!stop_token.stop_requested()) {
        readInputs();

        // publish only the latest state, dropping any older ones that have not yet been published
        while (outputs_queue_.pop(latest_output_)){}
        publishOutputs(latest_output_);

        sleeper.sleep();
    }
}

template <typename RI>
void FastDDSHLInterface<RI>::readInputs(){
    this->transport.read();

    // Setting the lost-references flag
    current_input_.safety.setLostReferences(
        input_control_signal->hasStartedReceivingData() && !this->transport.areInputsReceivingData());
    current_input_.ref.position = input_control_signal->msg.joints_position();
    current_input_.ref.velocity = input_control_signal->msg.joints_velocity();
    current_input_.ref.torque = input_control_signal->msg.joints_torques();
    current_input_.ref.kp = input_control_signal->msg.kp();
    current_input_.ref.kd = input_control_signal->msg.kd();
    current_input_.ref.control_mode = input_control_signal->msg.control_mode();

    // Non-blocking: if read() is not being called and the queue is full, the
    // reference is dropped; read() catches up on the next pushed one.
    inputs_queue_.push(current_input_);
}

template <typename RI>
void FastDDSHLInterface<RI>::publishOutputs(const RI& source){
    const auto& state = source.state;
    // fill in blind state
    output_blind_state->msg.joints_name() = source.state.names;
    output_blind_state->msg.joints_position() = state.position;
    output_blind_state->msg.joints_velocity() = state.velocity;
    output_blind_state->msg.joints_acceleration() = state.acceleration;
    output_blind_state->msg.joints_effort() = state.torque;
    output_blind_state->msg.joints_temperature() = state.motor_temperature;
    output_blind_state->msg.feet_contact() = state.contacts;
    output_blind_state->msg.motors_current() = state.motors_current;
    output_blind_state->msg.motors_status_word() = state.motors_status_word;
    output_blind_state->msg.motors_fault_info().resize(state.motor_fault_info.size());
    for (size_t i = 0; i < state.motor_fault_info.size(); ++i) {
        const auto& fault_info = state.motor_fault_info[i];
        output_blind_state->msg.motors_fault_info()[i].assign(fault_info.data(), strnlen(fault_info.data(), fault_info.size()));
    }
    output_blind_state->msg.trunk_temperature() = source.state.trunk_temperature;

    // fill in imu state
    for(size_t i = 0; i < state.imu.size(); ++i){
        output_imu->msg.frame_id() = state.imu[i].frame_id;
        output_imu->msg.orientation() = state.imu[i].orientation;
        output_imu->msg.orientation_rpy() = state.imu[i].orientation_rpy;
        output_imu->msg.orientation_covariance() = state.imu[i].orientation_covariance;
        output_imu->msg.angular_velocity() = state.imu[i].angular_velocity;
        output_imu->msg.angular_velocity_covariance() = state.imu[i].angular_velocity_covariance;
        output_imu->msg.linear_acceleration() = state.imu[i].linear_acceleration;
        output_imu->msg.linear_acceleration_covariance() = state.imu[i].linear_acceleration_covariance;
    }
    // fill in battery state
    for (size_t i = 0; i < state.battery.size(); ++i) {
        output_battery_status->msg.id() = state.battery[i].id;
        output_battery_status->msg.voltage() = state.battery[i].v_batt;
        output_battery_status->msg.temperature() = state.battery[i].temp_batt;
        output_battery_status->msg.state_of_charge() = state.battery[i].charge_percentage;
        output_battery_status->msg.status() = state.battery[i].status;
        output_battery_status->msg.fault() = state.battery[i].fault;
    }
    // fill in hal commands feedback
    output_hal_commands_feedback->msg.control_mode() = static_cast<std::uint8_t>(source.low_level_control.control_mode);
    output_hal_commands_feedback->msg.joints_position() = source.low_level_control.commands.position;
    output_hal_commands_feedback->msg.joints_velocity() = source.low_level_control.commands.velocity;
    output_hal_commands_feedback->msg.joints_torques() = source.low_level_control.commands.torque;
    output_hal_commands_feedback->msg.kp() = source.ref.kp;
    output_hal_commands_feedback->msg.kd() = source.ref.kd;

    // publish hal safety debug
    source.safety.getVirtualJointLimitDebug(virtual_joint_limit_debug_);
    auto& safety_debug = output_hal_safety_debug->msg;
    safety_debug.safety_state() = static_cast<std::uint8_t>(source.safety.getSafetyState());
    safety_debug.joint_pos_limit_violation_predicted() = virtual_joint_limit_debug_.predicted;
    safety_debug.virtual_joint_limit_side().assign(virtual_joint_limit_debug_.side.begin(),
                                                   virtual_joint_limit_debug_.side.end());
    safety_debug.virtual_joint_limit_anchor() = virtual_joint_limit_debug_.anchor;
    safety_debug.virtual_joint_limit_k() = virtual_joint_limit_debug_.k;
    safety_debug.virtual_joint_limit_d() = virtual_joint_limit_debug_.d;
    safety_debug.virtual_joint_limit_damping_only() = virtual_joint_limit_debug_.damping_only;
    safety_debug.virtual_joint_limit_torque() = virtual_joint_limit_debug_.torque;

    // Publish only this class's outputs rather than transport.write(): derived
    // interfaces may own further outputs that they publish from their own thread.
    output_blind_state->publish();
    output_imu->publish();
    output_battery_status->publish();
    output_hal_commands_feedback->publish();
    output_hal_safety_debug->publish();
}

template <typename RI>
void FastDDSHLInterface<RI>::writeEvents(const RI& source){
    events_queue_.push( source );
}

template <typename RI>
void FastDDSHLInterface<RI>::eventLoggingLoop(std::stop_token stop_token){
    PeriodicSleeper sleeper(event_check_period_);

    while (!stop_token.stop_requested()) {
        std::vector<RI> robot_interface_snapshots;
        RI robot_interface;
        while ( events_queue_.pop(robot_interface) )
            robot_interface_snapshots.push_back(robot_interface);

        // check for anomalies
        for(const auto& ri : robot_interface_snapshots){
            checkEvents(ri);
        }

        sleeper.sleep();
    }
}