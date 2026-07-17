#include "dls_hal/fastdds_high_level_interface.hpp"
#include "dls2/topics/topics.hpp"
#include "dls2/domains/domains.hpp"

FastDDSHLInterface::FastDDSHLInterface(){};

FastDDSHLInterface::~FastDDSHLInterface(){};

void FastDDSHLInterface::init(){
    // input trajectory generator
    input_traj_gen = transport.buildInput<dls2_interface::msg::TrajectoryGenerator>(
    dls::topics::trajectory_generator);
    input_desired_torques = transport.buildInput<dls2_interface::msg::DesiredTorques>(
    dls::topics::desired_torques);
    output_blind_state = transport.buildOutput<dls2_interface::msg::BlindState>(
    dls::topics::low_level_estimation::blind_state);
}

void FastDDSHLInterface::consumeInputs(const GenericRobotInterface& source){
    // consume inputs from the transport layer and update the source
}

void FastDDSHLInterface::produceOutputs(const GenericRobotInterface& source){
    // produce outputs to the transport layer from the source

}