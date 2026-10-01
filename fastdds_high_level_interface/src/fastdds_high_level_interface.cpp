#include "dls_hal/fastdds_high_level_interface.hpp"
#include "dls2/topics/topics.hpp"
#include "dls2/domains/domains.hpp"
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <pthread.h>
#include <sched.h>

FastDDSHLInterface::FastDDSHLInterface(const std::string& ID)
    : event_checker_(ID)
{}

FastDDSHLInterface::~FastDDSHLInterface(){
    event_thread_.request_stop(); // eventLoggingLoop() exits at most one event period later
    if (event_thread_.joinable())
        event_thread_.join();

    io_thread_.request_stop(); // ioLoop() exits at most one io period later
    if (io_thread_.joinable())
        io_thread_.join();
};

void FastDDSHLInterface::setEventLogger(dls::logging::AppLogger& logger)
{
    event_checker_.setEventLogger(logger);
}

void FastDDSHLInterface::checkEvents(const GenericRobotInterface& source)
{
    event_checker_.check(source);
}

void FastDDSHLInterface::init(GenericRobotInterface& source, const YAML::Node& config){
    // input control signal
    input_control_signal = transport.buildInput<dls2_interface::msg::ControlSignal>(
    dls::topics::control_signal);
    input_control_signal->setReceivingDataTimeout(std::chrono::duration<double>(source.safety.getReferencesTimeout()));
    output_blind_state = transport.buildOutput<dls2_interface::msg::BlindState>(
    dls::topics::low_level_estimation::blind_state);
    output_imu = transport.buildOutput<dls2_interface::msg::Imu>(
    dls::topics::low_level_estimation::imu);
    output_battery_status = transport.buildOutput<dls2_interface::msg::BatteryStatus>(
    dls::topics::battery_status);
    output_hal_commands_feedback = transport.buildOutput<dls2_interface::msg::ControlSignal>(
    dls::topics::hal_commands_feedback);

    // init inputs
    input_control_signal->msg.joints_position().resize(source.ref.position.size(), 0.0);
    input_control_signal->msg.joints_velocity().resize(source.ref.velocity.size(), 0.0);
    input_control_signal->msg.joints_torques().resize(source.ref.torque.size(), 0.0);
    input_control_signal->msg.kp().resize(source.ref.kp.size(), 0.0);
    input_control_signal->msg.kd().resize(source.ref.kd.size(), 0.0);
    input_control_signal->msg.control_mode() = source.ref.control_mode;
    //init outputs
    output_blind_state->msg.joints_position().resize(source.state.position.size(), 0.0);
    output_blind_state->msg.joints_velocity().resize(source.state.velocity.size(), 0.0);
    output_blind_state->msg.joints_acceleration().resize(source.state.acceleration.size(), 0.0);
    output_blind_state->msg.joints_effort().resize(source.state.torque.size(), 0.0);
    output_blind_state->msg.joints_temperature().resize(source.state.motor_temperature.size(), 0.0);
    output_blind_state->msg.feet_contact().resize(source.state.contacts.size(), false);
    output_blind_state->msg.motors_current().resize(source.state.motors_current.size(), 0.0);
    output_blind_state->msg.motors_status_word().resize(source.state.motors_status_word.size(), 0);
    output_hal_commands_feedback->msg = input_control_signal->msg;

    // DDS reading/publishing runs in its own thread, decoupled from the HAL's
    // real-time frequency, since the transport can block on DDS I/O.
    if (config["io_frequency"]) {
        const double io_frequency = config["io_frequency"].as<double>();
        if (io_frequency <= 0.0)
            throw std::invalid_argument("FastDDSHLInterface: io_frequency must be positive");
        io_period_ = std::chrono::duration<double>(1.0 / io_frequency);
    }
    if (config["io_sched_priority"]) {
        io_sched_priority_ = config["io_sched_priority"].as<int>();
        if (io_sched_priority_ < 0 || io_sched_priority_ > sched_get_priority_max(SCHED_FIFO))
            throw std::invalid_argument("FastDDSHLInterface: io_sched_priority must be in [0, "
                + std::to_string(sched_get_priority_max(SCHED_FIFO)) + "]");
    }

    if (!io_thread_.joinable()) {
        // Must be initialized before the io thread starts filling it: copying
        // the source gives its reference vectors the robot's sizes.
        input_robot_interface_ = source;
        latest_output_ = source;
        std::cout << "Starting ioLoop HAL thread with period " << io_period_.count() << " seconds" << std::endl;
        io_thread_ = std::jthread([this](std::stop_token stop_token) { ioLoop(stop_token); });

        // init() runs before the HAL's PeriodicApp switches its own thread to
        // SCHED_FIFO, and that switch applies to the calling thread only: without
        // this the io thread stays SCHED_OTHER and its wake-ups get delayed by
        // every other process on the machine.
        if (io_sched_priority_ > 0) {
            sched_param param{};
            param.sched_priority = io_sched_priority_;
            const int ret = pthread_setschedparam(io_thread_.native_handle(), SCHED_FIFO, &param);
            if (ret != 0)
                std::cerr << "FastDDSHLInterface: could not set SCHED_FIFO priority "
                          << io_sched_priority_ << " on the io thread: " << std::strerror(ret) << std::endl;
        }
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

bool FastDDSHLInterface::check(){
    return transport.areInputsReceivingData() && transport.areOutputsUnique();
}

void FastDDSHLInterface::read(GenericRobotInterface& source){
    // Only the newest reference matters: drain the queue and keep the last one.
    // If nothing arrived since the previous cycle, source keeps its reference.
    bool received = false;
    while (inputs_queue_.pop(latest_input_))
        received = true;
    if (!received)
        return;

    // Only the reference and the lost-references flag are filled by ioLoop().
    source.safety.setLostReferences(latest_input_.safety.anomalies.lost_references.load(std::memory_order_relaxed));
    source.ref.position = latest_input_.ref.position;
    source.ref.velocity = latest_input_.ref.velocity;
    source.ref.torque = latest_input_.ref.torque;
    source.ref.kp = latest_input_.ref.kp;
    source.ref.kd = latest_input_.ref.kd;
    source.ref.control_mode = latest_input_.ref.control_mode;
}

void FastDDSHLInterface::write(const GenericRobotInterface& source){
    // Non-blocking: if ioLoop() falls behind and the queue is full, the sample is dropped.
    outputs_queue_.push(source);
}

void FastDDSHLInterface::ioLoop(std::stop_token stop_token){
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(io_period_);
    auto next_wakeup = std::chrono::steady_clock::now();

    while (!stop_token.stop_requested()) {
        readInputs();

        // publish only the latest state, dropping any older ones that have not yet been published
        while (outputs_queue_.pop(latest_output_)){}
        publishOutputs(latest_output_);

        // Sleep until an absolute deadline, so the work time does not add to
        // the period; skip the expired slots after an overrun instead of
        // running a burst of catch-up iterations.
        next_wakeup += period;
        const auto now = std::chrono::steady_clock::now();
        if (now > next_wakeup)
            next_wakeup += period * ((now - next_wakeup) / period + 1);
        std::this_thread::sleep_until(next_wakeup);
    }
}

void FastDDSHLInterface::readInputs(){
    transport.read();

    // Checked every cycle so missing required inputs select damping recovery.
    // Once triggered the anomaly latches; resumed messages do not clear it.
    // A startup grace period is required: until the control signal has
    // arrived at least once, its absence is not yet a lost reference, only
    // a controller that has not activated/connected yet.
    input_robot_interface_.safety.setLostReferences(
        input_control_signal->hasStartedReceivingData() && !transport.areInputsReceivingData());
    input_robot_interface_.ref.position = input_control_signal->msg.joints_position();
    input_robot_interface_.ref.velocity = input_control_signal->msg.joints_velocity();
    input_robot_interface_.ref.torque = input_control_signal->msg.joints_torques();
    input_robot_interface_.ref.kp = input_control_signal->msg.kp();
    input_robot_interface_.ref.kd = input_control_signal->msg.kd();
    input_robot_interface_.ref.control_mode = input_control_signal->msg.control_mode();

    // Non-blocking: if read() is not being called and the queue is full, the
    // reference is dropped; read() catches up on the next pushed one.
    inputs_queue_.push(input_robot_interface_);
}

void FastDDSHLInterface::publishOutputs(const GenericRobotInterface& source){
    const auto& state = source.state;
    // mapping blind state to source state
    output_blind_state->msg.joints_position() = state.position;
    output_blind_state->msg.joints_velocity() = state.velocity;
    output_blind_state->msg.joints_acceleration() = state.acceleration;
    output_blind_state->msg.joints_effort() = state.torque;
    output_blind_state->msg.joints_temperature() = state.motor_temperature;
    output_blind_state->msg.feet_contact() = state.contacts;
    output_blind_state->msg.motors_current() = state.motors_current;
    output_blind_state->msg.motors_status_word() = state.motors_status_word;
    writeIMU(state);
    // publish battery status
    for (size_t i = 0; i < state.battery.size(); ++i) {
        output_battery_status->msg.id() = state.battery[i].id;
        output_battery_status->msg.voltage() = state.battery[i].v_batt;
        output_battery_status->msg.temperature() = state.battery[i].temp_batt;
        output_battery_status->msg.state_of_charge() = state.battery[i].charge_percentage;
        output_battery_status->msg.status() = state.battery[i].status;
        output_battery_status->msg.fault() = state.battery[i].fault;
    }

    // publish hal control signal feedback
    output_hal_commands_feedback->msg.control_mode() = static_cast<std::uint8_t>(source.low_level_control.control_mode);
    output_hal_commands_feedback->msg.joints_position() = source.low_level_control.commands.position;
    output_hal_commands_feedback->msg.joints_velocity() = source.low_level_control.commands.velocity;
    output_hal_commands_feedback->msg.joints_torques() = source.low_level_control.commands.torque;
    output_hal_commands_feedback->msg.kp() = source.ref.kp;
    output_hal_commands_feedback->msg.kd() = source.ref.kd;

    // Publish only this class's outputs rather than transport.write(): derived
    // interfaces may own further outputs that they publish from their own thread.
    output_blind_state->publish();
    output_imu->publish();
    output_battery_status->publish();
    output_hal_commands_feedback->publish();
}

void FastDDSHLInterface::writeEvents(const GenericRobotInterface& source){
    robot_interface_queue.push( source );
}

void FastDDSHLInterface::eventLoggingLoop(std::stop_token stop_token){
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(event_check_period_);
    auto next_wakeup = std::chrono::steady_clock::now() + period;
    std::this_thread::sleep_until(next_wakeup);

    while (!stop_token.stop_requested()) {
        std::vector<GenericRobotInterface> robot_interface_snapshots;
        GenericRobotInterface robot_interface;
        while ( robot_interface_queue.pop(robot_interface) )
            robot_interface_snapshots.push_back(robot_interface);

        // check for anomalies
        for(auto ri : robot_interface_snapshots){
            checkEvents(ri);
        }

        // Same absolute-deadline sleep as ioLoop().
        next_wakeup += period;
        const auto now = std::chrono::steady_clock::now();
        if (now > next_wakeup)
            next_wakeup += period * ((now - next_wakeup) / period + 1);
        std::this_thread::sleep_until(next_wakeup);
    }
}

void FastDDSHLInterface::writeIMU(const RobotState& state){
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
}
