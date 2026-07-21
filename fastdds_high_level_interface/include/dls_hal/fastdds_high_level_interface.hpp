#pragma once

#include "transport_interfaces/transport_interface.hpp"
#include "transports/fastdds/fastdds_transport.hpp"
#include "dls_hal/generic_robot_interface.hpp"
#include "dls2/signal/reader.hpp"
#include "dls2/signal/writer.hpp"

class FastDDSHLInterface : public TransportInterface<GenericRobotInterface, FastDDSTransport> {
    public:
        FastDDSHLInterface();
        ~FastDDSHLInterface();

        void init(const GenericRobotInterface& source) override;

        void read(GenericRobotInterface& source) override;
        void write(const GenericRobotInterface& source) override;

        void writeIMU(const GenericRobotInterface& source);

        bool check() override;

        dls::ReaderPtr<dls2_interface::msg::TrajectoryGenerator> input_traj_gen;
        dls::ReaderPtr<dls2_interface::msg::DesiredTorques> input_desired_torques;
        dls::WriterPtr<dls2_interface::msg::BlindState> output_blind_state;
        dls::WriterPtr<dls2_interface::msg::Imu> output_imu;
};