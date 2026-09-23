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

// The type of GenericRobotInterface::safety.anomalies, named here so
// FastDDSHLInterface can keep an accumulator of just the anomalies (see
// below) without depending on RobotSafety's template parameters.
using RobotAnomaliesType = decltype(GenericRobotInterface().safety.anomalies);

class FastDDSHLInterface : public TransportInterface<GenericRobotInterface, FastDDSTransport> {
    public:
        explicit FastDDSHLInterface(const std::string& ID = "FastDDSHLInterface");
        ~FastDDSHLInterface();

        void init(GenericRobotInterface& source, const YAML::Node& config) override;

        void read(GenericRobotInterface& source) override;
        void write(const GenericRobotInterface& source) override;
        // Cheap: if this cycle has no anomaly, returns immediately. Otherwise
        // OR-merges it into accumulated_anomalies_ and mirrors source into
        // latest_source_, both consumed by the background event logging
        // thread. Does not itself check or publish events, so it is safe to
        // call at the HAL's real-time frequency.
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
        // wakes on its own period; if an anomaly is pending, takes
        // latest_source_ with accumulated_anomalies_ substituted in, resets
        // accumulated_anomalies_ for the next window, and runs the
        // (potentially blocking, e.g. DDS-publishing) event checks on it.
        void eventLoggingLoop();

        FastDDSEventChecker event_checker_;

        // The full robot interface as of the most recent HAL cycle that had
        // any anomaly, giving eventLoggingLoop() the state/commands/limits it
        // needs to build event messages from. Anomalies are OR-merged into
        // accumulated_anomalies_ (see RobotAnomalies::mergeFrom()) across
        // cycles instead, so a flag only true for a single HAL cycle is not
        // lost between the event thread's (lower-frequency) wake-ups. Both
        // are only touched by writeEvents() when the current cycle actually
        // has an anomaly (RobotAnomalies::any()); a clean cycle leaves them
        // untouched. Access is guarded by event_snapshot_mutex_.
        GenericRobotInterface latest_source_;
        RobotAnomaliesType accumulated_anomalies_;
        mutable std::mutex event_snapshot_mutex_;
        // True while there is an accumulated anomaly eventLoggingLoop() has
        // not yet consumed.
        bool event_snapshot_valid_{false};

        // Overridden by robot_interface.events_frequency (source.safety.getEventsFrequency()) in init().
        std::chrono::duration<double> event_check_period_{std::chrono::duration<double>(0.1)};
        std::thread event_thread_;
        std::mutex event_thread_control_mutex_;
        std::condition_variable event_thread_control_cv_;
        bool event_thread_running_{false};
};
