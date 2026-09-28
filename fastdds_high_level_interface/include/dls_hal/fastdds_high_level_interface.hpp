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
#include <condition_variable>
#include <mutex>
#include <thread>
#include <boost/lockfree/spsc_queue.hpp>


class FastDDSHLInterface : public TransportInterface<GenericRobotInterface, FastDDSTransport> {
    public:
        explicit FastDDSHLInterface(const std::string& ID = "FastDDSHLInterface");
        ~FastDDSHLInterface();

        void init(GenericRobotInterface& source, const YAML::Node& config) override;

        void read(GenericRobotInterface& source) override;
        void write(const GenericRobotInterface& source) override;
        // Publish the events detected in the source's robot interface (e.g. anomalies/safety events).
        void writeEvents(const GenericRobotInterface& source) override;

        void writeIMU(const GenericRobotInterface& source);

        bool check() override;

        // Optionally override the owned logger before running.
        // The external logger must outlive this interface.
        void setEventLogger(dls::logging::AppLogger& logger);
        void checkEvents(const GenericRobotInterface& source);

        dls::ReaderPtr<dls2_interface::msg::ControlSignal> input_control_signal;
        dls::WriterPtr<dls2_interface::msg::BlindState> output_blind_state;
        dls::WriterPtr<dls2_interface::msg::Imu> output_imu;
        dls::WriterPtr<dls2_interface::msg::BatteryStatus> output_battery_status;
        dls::WriterPtr<dls2_interface::msg::ControlSignal> output_hal_commands_feedback;
    private:
        // Runs on event_thread_, decoupled from the HAL's real-time frequency:
        // processes the event queue and logs events.
        void eventLoggingLoop();

        FastDDSEventChecker event_checker_;

        // Overridden by robot_interface.events_frequency (source.safety.getEventsFrequency()) in init().
        std::chrono::duration<double> event_check_period_{std::chrono::duration<double>(0.1)};
        std::thread event_thread_;
        std::mutex event_thread_control_mutex_;
        std::condition_variable event_thread_control_cv_;
        bool event_thread_running_{false};

        boost::lockfree::spsc_queue<GenericRobotInterface, boost::lockfree::capacity< 2000 > > robot_interface_queue;
};
