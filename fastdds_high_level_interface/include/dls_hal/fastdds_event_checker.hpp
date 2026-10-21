#pragma once

#include <memory>
#include <string>

namespace dls { namespace logging { class AppLogger; } }
class GenericRobotInterface;

// Formats and publishes safety events for the high-level interface.
class FastDDSEventChecker {
public:
    explicit FastDDSEventChecker(const std::string& ID = "FastDDSHLInterface");

    // Optionally override the owned logger before checking events.
    // The external logger must outlive this checker and retains its own ID.
    void setEventLogger(dls::logging::AppLogger& logger);
    // Logger used for the events, so that robot-specific checks can log alongside the generic ones.
    dls::logging::AppLogger& eventLogger() const;
    void check(const GenericRobotInterface& source);

private:
    void checkJointLimitEvents(const GenericRobotInterface& source);
    void checkInvalidValueEvents(const GenericRobotInterface& source);
    void checkMotorFaultEvents(const GenericRobotInterface& source);
    void checkBatteryEvents(const GenericRobotInterface& source);
    void checkEmergencyAndRecoveryEvents(const GenericRobotInterface& source);
    void checkVirtualEndstopEvents(const GenericRobotInterface& source);
    void checkSafetyStateEvents(const GenericRobotInterface& source);

    std::shared_ptr<dls::logging::AppLogger> event_logger_;
};
