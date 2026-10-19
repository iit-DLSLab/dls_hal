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
#include <type_traits>
#include <boost/lockfree/spsc_queue.hpp>


// RI is the robot interface type the HAL runs on. Robot-specific interfaces
// derived from GenericRobotInterface (e.g. carrying extra data) instantiate this
// with their own type, so the snapshots exchanged with the io thread keep that
// extra data instead of being sliced to GenericRobotInterface.
template <typename RI = GenericRobotInterface>
class FastDDSHLInterface : public TransportInterface<RI, FastDDSTransport> {
    static_assert(std::is_base_of_v<GenericRobotInterface, RI>,
                  "FastDDSHLInterface: Robot Interface must derive from GenericRobotInterface");
    public:
        explicit FastDDSHLInterface(const std::string& ID = "FastDDSHLInterface");
        ~FastDDSHLInterface();

        void init(RI& source, const YAML::Node& config) override;

        // Fill the robot interface with the latest inputs information from the io thread
        void read(RI& source) override;
        // Communicate the robot interface state to the io thread, which will publish it to the high-level nodes
        void write(const RI& source) override;
        // Communicate the robot interface events to the io thread, which will publish them to the high-level nodes
        void writeEvents(const RI& source) override;

        bool check() override;

        // Optionally override the owned logger before running.
        // The external logger must outlive this interface.
        void setEventLogger(dls::logging::AppLogger& logger);
        // Checks and logs the events of a robot interface snapshot. Runs on the event thread.
        // Override to add robot-specific checks, calling the base for the generic ones.
        virtual void checkEvents(const RI& source);

        // Inputs
        dls::ReaderPtr<dls2_interface::msg::ControlSignal> input_control_signal;
        // Outputs
        dls::WriterPtr<dls2_interface::msg::BlindState> output_blind_state;
        dls::WriterPtr<dls2_interface::msg::Imu> output_imu;
        dls::WriterPtr<dls2_interface::msg::BatteryStatus> output_battery_status;
        dls::WriterPtr<dls2_interface::msg::ControlSignal> output_hal_commands_feedback;
        dls::WriterPtr<dls2_interface::msg::HalSafetyDebug> output_hal_safety_debug;

        protected:
        // Publish the outputs through the fastdds transport. Runs on the io thread.
        virtual void publishOutputs(const RI& source);
        // Read the latest inputs from the fastdds transport into current_input_. Runs on the io thread.
        virtual void readInputs();

        //! Filled by readInputs() with the received reference; copied from the source
        // in init() so its vectors already have the robot's sizes.
        RI current_input_;

        //! Event checker for anomalies and safety events
        FastDDSEventChecker event_checker_;

        // Reused every write() to collect the virtual joint limit state for output_hal_safety_debug.
        VirtualJointLimitDebug virtual_joint_limit_debug_;

        /**
         * @brief Stops and joins the io and event threads. Idempotent.
         * Derived classes overriding publishOutputs(), readInputs() or checkEvents() must call it
         * in their destructor: the threads would otherwise keep calling the overrides while
         * the derived members are being destroyed, before ~FastDDSHLInterface() stops them.
         */
        void stopThreads();
    private:

        //! Input/output thread
        std::jthread io_thread_; 
        //! Period of the io_thread_ thread
        std::chrono::duration<double> io_period_{std::chrono::duration<double>(0.001)}; // 1 kHz unless io_frequency is set
        /**
         * @brief Runs on io_thread_. Reads the DDS inputs into reference_queue_ and publishes the snapshots queued in state_queue_.
        */
        void ioLoop(std::stop_token stop_token);
        // Filled by ioLoop() with the received inputs;
        boost::lockfree::spsc_queue<RI, boost::lockfree::capacity< 100 > > inputs_queue_;
        // Filled by ioLoop() with the received outputs from the HAL;
        boost::lockfree::spsc_queue<RI, boost::lockfree::capacity< 100 > > outputs_queue_;

        //! Event logging thread, decoupled from the HAL's real-time frequency
        std::jthread event_thread_;
        //! Period of the eventLoggingLoop() thread
        std::chrono::duration<double> event_check_period_{std::chrono::duration<double>(0.01)};
        /**
         * @brief Runs on event_thread_, decoupled from the HAL's real-time frequency:
         * Checks the events queued in events_queue_ and logs them.
         */
        void eventLoggingLoop(std::stop_token stop_token);
        //! Filled by writeEvents() with the received events from the HAL;
        boost::lockfree::spsc_queue<RI, boost::lockfree::capacity< 2000 > > events_queue_;

        //! Last input popped by read(), reused to avoid reallocating each cycle.
        RI latest_input_;
        //! Last output popped by ioLoop() and published, reused to avoid reallocating
        // each cycle; copied from the source in init() so it is valid before the
        // first write().
        RI latest_output_;
};

#include "dls_hal/fastdds_high_level_interface.tpp"
