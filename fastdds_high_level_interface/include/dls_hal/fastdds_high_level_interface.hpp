#pragma once

#include "transport_interfaces/transport_interface.hpp"
#include "transports/fastdds/fastdds_transport.hpp"
#include "dls_hal/generic_robot_interface.hpp"
#include "dls_hal/fastdds_event_checker.hpp"
#include "dls2/signal/reader.hpp"
#include "dls2/signal/writer.hpp"

#include "yaml-cpp/yaml.h"
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <stop_token>
#include <thread>
#include <boost/lockfree/spsc_queue.hpp>


class FastDDSHLInterface : public TransportInterface<GenericRobotInterface, FastDDSTransport> {
    public:
        explicit FastDDSHLInterface(const std::string& ID = "FastDDSHLInterface");
        ~FastDDSHLInterface();

        void init(GenericRobotInterface& source, const YAML::Node& config) override;

        // Fill the robot interface with the latest inputs information from the io thread
        void read(GenericRobotInterface& source) override;
        // Communicate the robot interface state to the io thread, which will publish it to the high-level nodes
        void write(const GenericRobotInterface& source) override;
        // Communicate the robot interface events to the io thread, which will publish them to the high-level nodes
        void writeEvents(const GenericRobotInterface& source) override;

        bool check() override;

        // Optionally override the owned logger before running.
        // The external logger must outlive this interface.
        void setEventLogger(dls::logging::AppLogger& logger);
        void checkEvents(const GenericRobotInterface& source);

        // Inputs
        dls::ReaderPtr<dls2_interface::msg::ControlSignal> input_control_signal;
        // Outputs
        dls::WriterPtr<dls2_interface::msg::BlindState> output_blind_state;
        dls::WriterPtr<dls2_interface::msg::Imu> output_imu;
        dls::WriterPtr<dls2_interface::msg::BatteryStatus> output_battery_status;
        dls::WriterPtr<dls2_interface::msg::ControlSignal> output_hal_commands_feedback;
    protected:
        // Publish the outputs through the fastdds transport
        virtual void publishOutputs(const GenericRobotInterface& source);
        // Read the latest inputs from the fastdds transport
        virtual void readInputs();
    private:
        // Runs on io_thread_, decoupled from the HAL's real-time frequency:
        // reads the DDS inputs into reference_queue_ and publishes the
        // snapshots queued in state_queue_.
        void ioLoop(std::stop_token stop_token);
        // Runs on event_thread_, decoupled from the HAL's real-time frequency:
        // processes the event queue and logs events.
        void eventLoggingLoop(std::stop_token stop_token);

        void writeIMU(const RobotState& state);

        FastDDSEventChecker event_checker_;

        // Overridden by robot_interface.events_frequency (source.safety.getEventsFrequency()) in init().
        std::chrono::duration<double> event_check_period_{std::chrono::duration<double>(0.1)};
        // Stopped and joined explicitly in the destructor: the members below are
        // destroyed before it, while eventLoggingLoop() may still be using them.
        std::jthread event_thread_;
        
        // Filled by ioLoop() with the received inputs;
        boost::lockfree::spsc_queue<GenericRobotInterface, boost::lockfree::capacity< 100 > > inputs_queue_;
        // Filled by ioLoop() with the received outputs;
        boost::lockfree::spsc_queue<GenericRobotInterface, boost::lockfree::capacity< 100 > > outputs_queue_;
        
        boost::lockfree::spsc_queue<GenericRobotInterface, boost::lockfree::capacity< 2000 > > robot_interface_queue;

        // Overridden by the optional io_frequency key of the high-level config in init().
        std::chrono::duration<double> io_period_{std::chrono::duration<double>(0.001)};
        // SCHED_FIFO priority of io_thread_, overridden by the optional
        // io_sched_priority key of the high-level config in init(); 0 keeps SCHED_OTHER.
        // Keep it below the HAL's own priority so the io thread never preempts it.
        int io_sched_priority_{40};
        // Stopped and joined explicitly in the destructor: the members below are
        // destroyed before it, while ioLoop() may still be using them.
        std::jthread io_thread_;

        // Filled by ioLoop() with the received reference; copied from the source
        // in init() so its vectors already have the robot's sizes.
        GenericRobotInterface input_robot_interface_;
        // Last input popped by read(), reused to avoid reallocating each cycle.
        GenericRobotInterface latest_input_;
        // Last output popped by ioLoop() and published, reused to avoid reallocating
        // each cycle; copied from the source in init() so it is valid before the
        // first write().
        GenericRobotInterface latest_output_;
};
