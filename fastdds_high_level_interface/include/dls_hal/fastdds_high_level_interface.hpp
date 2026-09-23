#pragma once

#include "transport_interfaces/transport_interface.hpp"
#include "transports/fastdds/fastdds_transport.hpp"
#include "dls_hal/generic_robot_interface.hpp"
#include "dls_hal/fastdds_event_checker.hpp"
#include "dls2/signal/reader.hpp"
#include "dls2/signal/writer.hpp"

#include "yaml-cpp/yaml.h"
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

class FastDDSHLInterface : public TransportInterface<GenericRobotInterface, FastDDSTransport> {
    public:
        explicit FastDDSHLInterface(const std::string& ID = "FastDDSHLInterface");
        ~FastDDSHLInterface();

        void init(GenericRobotInterface& source, const YAML::Node& config) override;

        void read(GenericRobotInterface& source) override;
        void write(const GenericRobotInterface& source) override;
        // Cheap: only updates the snapshot consumed by the background event
        // logging thread. Does not itself check or publish events, so it is
        // safe to call at the HAL's real-time frequency.
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
        // wakes on its own period, copies out the latest snapshot and runs
        // the (potentially blocking, e.g. DDS-publishing) event checks on it.
        void eventLoggingLoop();

        FastDDSEventChecker event_checker_;

        // Snapshot of the event-relevant robot state, written by writeEvents()
        // (real-time thread) and read by eventLoggingLoop() (background
        // thread). All access is guarded by event_snapshot_mutex_.
        GenericRobotInterface event_snapshot_;
        mutable std::mutex event_snapshot_mutex_;
        // False until writeEvents() has populated event_snapshot_ at least
        // once; guards against eventLoggingLoop() checking a default-
        // constructed (empty) snapshot before the first HAL cycle runs.
        bool event_snapshot_valid_{false};

        // Overridden by robot_interface.events_frequency (source.safety.getEventsFrequency()) in init().
        std::chrono::duration<double> event_check_period_{std::chrono::duration<double>(0.1)};
        std::thread event_thread_;
        std::mutex event_thread_control_mutex_;
        std::condition_variable event_thread_control_cv_;
        bool event_thread_running_{false};
};
