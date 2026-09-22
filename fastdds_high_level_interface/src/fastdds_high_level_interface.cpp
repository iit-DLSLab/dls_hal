#include "dls_hal/fastdds_high_level_interface.hpp"
#include "dls2/topics/topics.hpp"
#include "dls2/domains/domains.hpp"
#include <cstdint>

FastDDSHLInterface::FastDDSHLInterface(const std::string& ID)
    : event_checker_(ID)
{}

FastDDSHLInterface::~FastDDSHLInterface(){};

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
}
    
bool FastDDSHLInterface::check(){
    return transport.areInputsReceivingData() && transport.areOutputsUnique();
}

void FastDDSHLInterface::read(GenericRobotInterface& source){
    transport.read();
    // Checked every cycle so missing required inputs select damping recovery.
    // Once triggered the anomaly latches; resumed messages do not clear it.
    source.safety.setReferencesNotReceived(!transport.areInputsReceivingData());
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
    checkEvents(source);
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
