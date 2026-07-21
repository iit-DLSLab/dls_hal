#include "dls_hal/fastdds_high_level_interface.hpp"
#include "dls2/topics/topics.hpp"
#include "dls2/domains/domains.hpp"

FastDDSHLInterface::FastDDSHLInterface(){};

FastDDSHLInterface::~FastDDSHLInterface(){};

void FastDDSHLInterface::init(const GenericRobotInterface& source){
    // input trajectory generator
    input_traj_gen = transport.buildInput<dls2_interface::msg::TrajectoryGenerator>(
    dls::topics::trajectory_generator);
    input_desired_torques = transport.buildInput<dls2_interface::msg::DesiredTorques>(
    dls::topics::desired_torques);
    output_blind_state = transport.buildOutput<dls2_interface::msg::BlindState>(
    dls::topics::low_level_estimation::blind_state);
    output_imu = transport.buildOutput<dls2_interface::msg::Imu>(
    dls::topics::low_level_estimation::imu);
}

void FastDDSHLInterface::read(GenericRobotInterface& source){
    transport.read();
    source.ref.position = input_traj_gen->msg.joints_position();
    source.ref.velocity = input_traj_gen->msg.joints_velocity();
    source.ref.torque = input_desired_torques->msg.desired_torques(); // input_traj_gen->msg.joints_effort();
    source.ref.kp = input_traj_gen->msg.kp();
    source.ref.kd = input_traj_gen->msg.kd();
}

void FastDDSHLInterface::write(const GenericRobotInterface& source){
    // mapping blind state to source state
    output_blind_state->msg.joints_position() = source.state.position;
    output_blind_state->msg.joints_velocity() = source.state.velocity;
    output_blind_state->msg.joints_acceleration() = source.state.acceleration;
    output_blind_state->msg.joints_effort() = source.state.torques;
    output_blind_state->msg.joints_temperature() = source.state.motor_temperature;
    output_blind_state->msg.feet_contact() = source.state.contacts;
    writeIMU(source);
    // TODO add motor current, status word and battery status to blind state
    transport.write();
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