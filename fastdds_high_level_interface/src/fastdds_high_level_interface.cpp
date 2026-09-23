#include "dls_hal/fastdds_high_level_interface.hpp"
#include "dls2/topics/topics.hpp"
#include "dls2/domains/domains.hpp"
#include <cstdint>
#include <iostream>

FastDDSHLInterface::FastDDSHLInterface(const std::string& ID)
    : event_checker_(ID)
{}

FastDDSHLInterface::~FastDDSHLInterface(){
    {
        std::lock_guard<std::mutex> lock(event_thread_control_mutex_);
        event_thread_running_ = false;
    }
    event_thread_control_cv_.notify_all(); // awake eventLoggingLoop() without waiting for the next period
    if (event_thread_.joinable())
        event_thread_.join();
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

    // Event checking/logging runs in its own thread, decoupled from the HAL's
    // real-time frequency, since AppLogger can block on DDS/file I/O.
    // robot_interface.events_frequency is validated and defaulted by
    // RobotSafety::init(), which has already run by this point.
    const double events_frequency = source.safety.getEventsFrequency();
    event_check_period_ = std::chrono::duration<double>(1.0 / events_frequency);

    if (!event_thread_.joinable()) {
        event_thread_running_ = true;
        event_thread_ = std::thread(&FastDDSHLInterface::eventLoggingLoop, this);
    }
}

bool FastDDSHLInterface::check(){
    return transport.areInputsReceivingData() && transport.areOutputsUnique();
}

void FastDDSHLInterface::read(GenericRobotInterface& source){
    transport.read();
    // Checked every cycle so missing required inputs select damping recovery.
    // Once triggered the anomaly latches; resumed messages do not clear it.
    // A startup grace period is required: until the control signal has
    // arrived at least once, its absence is not yet a lost reference, only
    // a controller that has not activated/connected yet.
    if (input_control_signal->hasStartedReceivingData() && !transport.areInputsReceivingData())
        source.safety.setLostReferences(true);
    else
        source.safety.setLostReferences(false);
    source.ref.position = input_control_signal->msg.joints_position();
    source.ref.velocity = input_control_signal->msg.joints_velocity();
    source.ref.torque = input_control_signal->msg.joints_torques();
    source.ref.kp = input_control_signal->msg.kp();
    source.ref.kd = input_control_signal->msg.kd();
    source.ref.control_mode = input_control_signal->msg.control_mode();
}

void FastDDSHLInterface::write(const GenericRobotInterface& source){
    // mapping blind state to source state
    output_blind_state->msg.joints_position() = source.state.position;
    output_blind_state->msg.joints_velocity() = source.state.velocity;
    output_blind_state->msg.joints_acceleration() = source.state.acceleration;
    output_blind_state->msg.joints_effort() = source.state.torque;
    output_blind_state->msg.joints_temperature() = source.state.motor_temperature;
    output_blind_state->msg.feet_contact() = source.state.contacts;
    output_blind_state->msg.motors_current() = source.state.motors_current;
    output_blind_state->msg.motors_status_word() = source.state.motors_status_word;
    writeIMU(source);
    // publish battery status
    for (size_t i = 0; i < source.state.battery.size(); ++i) {
        output_battery_status->msg.id() = source.state.battery[i].id;
        output_battery_status->msg.voltage() = source.state.battery[i].v_batt;
        output_battery_status->msg.temperature() = source.state.battery[i].temp_batt;
        output_battery_status->msg.state_of_charge() = source.state.battery[i].charge_percentage;
        output_battery_status->msg.status() = source.state.battery[i].status;
        output_battery_status->msg.fault() = source.state.battery[i].fault;
    }

    // publish hal control signal feedback
    output_hal_commands_feedback->msg.control_mode() = static_cast<std::uint8_t>(source.low_level_control.control_mode);
    output_hal_commands_feedback->msg.joints_position() = source.low_level_control.commands.position;
    output_hal_commands_feedback->msg.joints_velocity() = source.low_level_control.commands.velocity;
    output_hal_commands_feedback->msg.joints_torques() = source.low_level_control.commands.torque;
    output_hal_commands_feedback->msg.kp() = source.ref.kp;
    output_hal_commands_feedback->msg.kd() = source.ref.kd;

    transport.write();
}

void FastDDSHLInterface::writeEvents(const GenericRobotInterface& source){
    // Called at the HAL's real-time frequency: keep this to a bounded, non-
    // blocking snapshot copy. The actual (potentially blocking) event
    // checking/logging happens on event_thread_, at its own lower frequency.
    std::lock_guard<std::mutex> lock(event_snapshot_mutex_);
    event_snapshot_ = source;
    event_snapshot_valid_ = true;
}

void FastDDSHLInterface::eventLoggingLoop(){
    GenericRobotInterface snapshot;
    std::unique_lock<std::mutex> control_lock(event_thread_control_mutex_);
    while (!event_thread_control_cv_.wait_for(
        control_lock, event_check_period_, [this] { return !event_thread_running_; })) {
        control_lock.unlock();
        bool has_snapshot = false;
        {
            std::lock_guard<std::mutex> lock(event_snapshot_mutex_);
            has_snapshot = event_snapshot_valid_;
            if (has_snapshot)
                snapshot = event_snapshot_;
        }
        if (has_snapshot)
            checkEvents(snapshot);
        control_lock.lock();
    }
}

void FastDDSHLInterface::writeIMU(const GenericRobotInterface& source){
    for(size_t i = 0; i < source.state.imu.size(); ++i){
        output_imu->msg.frame_id() = source.state.imu[i].frame_id;
        output_imu->msg.orientation() = source.state.imu[i].orientation;
        output_imu->msg.orientation_rpy() = source.state.imu[i].orientation_rpy;
        output_imu->msg.orientation_covariance() = source.state.imu[i].orientation_covariance;
        output_imu->msg.angular_velocity() = source.state.imu[i].angular_velocity;
        output_imu->msg.angular_velocity_covariance() = source.state.imu[i].angular_velocity_covariance;
        output_imu->msg.linear_acceleration() = source.state.imu[i].linear_acceleration;
        output_imu->msg.linear_acceleration_covariance() = source.state.imu[i].linear_acceleration_covariance;
    }
}
