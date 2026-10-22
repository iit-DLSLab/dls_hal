#ifndef ROBOTS_INVARIANTS_HPP
#define ROBOTS_INVARIANTS_HPP

#include "recovery_modes.hpp"
#include "vector"
#include "stdint.h"
#include "yaml-cpp/yaml.h"
#include <atomic>
#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <optional>
#include <string>

// std::atomic_bool with no copy constructor/assignment operator of its own
// (copying it is not itself atomic), which forces any enclosing struct to
// hand-write its own copy operations. This wrapper keeps the relaxed
// load/store semantics used throughout this file but is copyable: a copy
// takes an independent, consistent (if possibly stale) snapshot of the
// flag's value at that instant, which is exactly what's needed to hand a
// struct like RobotAnomalies off to another thread as a snapshot.
class RelaxedAtomicBool{
public:
    RelaxedAtomicBool() noexcept = default;
    RelaxedAtomicBool(bool value) noexcept : value_(value) {}
    RelaxedAtomicBool(const RelaxedAtomicBool& other) noexcept
        : value_(other.value_.load(std::memory_order_relaxed)) {}
    RelaxedAtomicBool& operator=(const RelaxedAtomicBool& other) noexcept {
        value_.store(other.value_.load(std::memory_order_relaxed), std::memory_order_relaxed);
        return *this;
    }

    bool load(std::memory_order order = std::memory_order_seq_cst) const noexcept { return value_.load(order); }
    void store(bool desired, std::memory_order order = std::memory_order_seq_cst) noexcept { value_.store(desired, order); }

private:
    std::atomic_bool value_{false};
};

class IMU{
public:
    IMU();
    void init();
    std::string frame_id;
    std::array<double, 4> orientation;
    std::array<double, 3> orientation_rpy;
    std::array<double, 9> orientation_covariance;
    std::array<double, 3> angular_velocity;
    std::array<double, 9> angular_velocity_covariance;
    std::array<double, 3> linear_acceleration;
    std::array<double, 9> linear_acceleration_covariance;
};

class RobotBattery{
public:
    RobotBattery();
    void init();

    double v_batt;
    double v_load;
    double i_load;
    double temp_batt;
    double temp_heatsink;
    double temp_pcb;
    double full_v_batt;
    double charge_percentage;
    uint32_t status;
    uint32_t fault;
    int id;
};

// Fixed size so that copying RobotState in the real-time thread never allocates.
using MotorFaultInfo = std::array<char, 64>;

class MotorStatusWords{
public:
    unsigned int fungo_pressed=0;
    // Status word bits set by an operational motor; 0 disables the motor operational check.
    unsigned int operational_mask=0;
};

class RobotState{
public:
    RobotState();

    void init(const YAML::Node& config);

    std::string robot_name;
    std::vector<std::string> names;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    std::vector<double> torque;
    std::vector<uint32_t> motors_status_word;
    std::vector<MotorFaultInfo> motor_fault_info; // null-terminated fault description reported by each motor
    std::vector<double> motor_temperature;
    std::vector<double> motors_current;
    std::vector<bool> contacts;
    std::vector<IMU> imu;
    double trunk_temperature;
    std::vector<RobotBattery> battery;
    MotorStatusWords const_motors_status_words;
};

class RobotCommands{
public:
    RobotCommands();
    void init(const YAML::Node& config);
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> torque;
};

class RobotRef{
public:
    RobotRef();
    void init(const YAML::Node& config);

    void initReference(const RobotState& state);

    uint32_t control_mode;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> torque;
    std::vector<double> kp;
    std::vector<double> ki;
    std::vector<double> kd;
};

class RobotLimits{
public:
    RobotLimits();
    void init(const YAML::Node& config);
    std::vector<double> position_min;
    std::vector<double> position_max;
    std::vector<double> velocity_min;
    std::vector<double> velocity_max;
    std::vector<double> torque_min;
    std::vector<double> torque_max;
    std::vector<double> current_min;
    std::vector<double> current_max;
    std::vector<double> motor_temperature_min;
    std::vector<double> motor_temperature_max;
    std::vector<double> joint_inertia;
    double percentage_battery_charge_min;
};

class RobotAnomalies{
public:
    
    std::vector<bool> joint_pos_limit_violation;
    std::vector<bool> joint_pos_limit_violation_predicted;
    std::vector<bool> joint_vel_limit_violation;
    std::vector<bool> joint_torque_limit_violation;
    std::vector<bool> joint_current_limit_violation;
    std::vector<bool> nan_position_detected;
    std::vector<bool> nan_velocity_detected;
    std::vector<bool> nan_torque_detected;
    std::vector<bool> inf_position_detected;
    std::vector<bool> inf_velocity_detected;
    std::vector<bool> inf_torque_detected;
    // Command anomalies are separate from measured-state anomalies.
    std::vector<bool> joint_pos_clamping_anomaly;
    std::vector<bool> joint_vel_clamping_anomaly;
    std::vector<bool> joint_torque_clamping_anomaly;
    // The pre-clamp command that triggered joint_*_clamping_anomaly above
    // (only meaningful where that flag is true this cycle): checkCommand()
    // clamps the command in place, and by the time an anomalous cycle's
    // snapshot is taken for logging, the live command has already been
    // overwritten with the clamped (safe) value.
    std::vector<double> joint_pos_command_trigger_value;
    std::vector<double> joint_vel_command_trigger_value;
    std::vector<double> joint_torque_command_trigger_value;
    std::vector<bool> nan_position_command_detected;
    std::vector<bool> nan_velocity_command_detected;
    std::vector<bool> nan_torque_command_detected;
    std::vector<bool> inf_position_command_detected;
    std::vector<bool> inf_velocity_command_detected;
    std::vector<bool> inf_torque_command_detected;
    std::vector<bool> motor_overheat_detected;
    std::vector<bool> motor_under_temperature_detected;
    // The motor reports a fault_info different from robot_interface.safety.motor_no_fault_info.
    std::vector<bool> motor_fault_detected;
    // The motor status word does not have all the bits of MotorStatusWords::operational_mask set.
    std::vector<bool> motor_not_operational_detected;
    std::vector<bool> battery_charge_low;
    // At least one node expected by the low-level interface was not discovered (RobotSafety::setDiscoveredNodes()).
    bool nodes_not_discovered = false;
    // The low-level interface detected a node detached while running (RobotSafety::setNodesDetached()).
    // Latched until init(); it switches the safety state to EMERGENCY_STOP (zero torque).
    bool nodes_detached = false;

    // One-way latch written by the command thread and read by the real-time
    // control thread; no blocking lock is needed.
    RelaxedAtomicBool emergency_stop_triggered;
    // Per-cycle status flag written by the control thread and read by the
    // event thread; it does not publish any accompanying state.
    RelaxedAtomicBool fungo_pressed;
    RelaxedAtomicBool lost_references;
};

// Per-joint snapshot of the virtual joint limit, for debugging.
struct VirtualJointLimitDebug{
    std::vector<bool> predicted; // joint_pos_limit_violation_predicted
    std::vector<int> side;       // +1 braking toward position_max, -1 toward position_min, 0 inactive
    std::vector<double> anchor;  // [rad], NaN when inactive
    std::vector<double> k;       // [Nm/rad]
    std::vector<double> d;       // [Nm*s/rad]
    std::vector<bool> damping_only; // latched damping only (damping_after_stop)
    std::vector<double> torque;  // [Nm] computed before the torque limits clamp, NaN when not overridden
};

enum class CONTROL_MODE{
    TORQUE_CONTROL=0,
    IMPEDANCE_CONTROL,
    POSITION_CONTROL,
    VELOCITY_CONTROL,
    CURRENT_CONTROL
};

template <typename TState, typename TRef, typename TCommands, typename TLimits>
class RobotSafety{
public:
    RobotSafety();
    
    void init(const YAML::Node& config);
    
    virtual void checkState(const TState& state);

    // Checks only the commands sent to the hardware in control_mode (position,
    // velocity, or torque for TORQUE/IMPEDANCE); the others are not computed.
    virtual void checkCommands(TCommands& commands, CONTROL_MODE control_mode);

    virtual bool checkJointStateLimit(double value, double min_limit, double max_limit);
    virtual bool checkJointStateNanInf(double value);

    void displayAnomalies() const;

    bool isFungoPressed() const;

    bool isAnyMotorNotOperational() const;

    bool isAnyNodeNotDiscovered() const;

    // How long the HAL notifies the startup failure events (motors not in operational state,
    // nodes not discovered) before going down.
    double getStartupFailureNotifyDuration() const;

    void triggerEmergencyStop();

    // Latching: true sets the anomaly, false is ignored; only init() clears it.
    void setLostReferences(bool lost);

    // Latching: true sets the nodes_detached anomaly, false is ignored; only init() clears it.
    void setNodesDetached(bool detached);

    /**
     * @brief Stores the ids of the nodes (e.g. EtherCAT slaves) discovered by the low-level interface;
     * raises the nodes_not_discovered anomaly if not all the expected nodes were discovered.
     * Called once by the low-level interface init().
     */
    void setDiscoveredNodes(const std::vector<int>& discovered, bool all_expected_discovered);
    const std::vector<int>& getDiscoveredNodes() const;

    bool isEmergencyStopActive() const;

    double getReferencesTimeout() const;

    double getEventsFrequency() const;

    // Fills debug with the current virtual joint limit state (resized to one entry per joint).
    void getVirtualJointLimitDebug(VirtualJointLimitDebug& debug) const;

    // Robot anomalies
    RobotAnomalies anomalies;

    enum class SafetyState{
        NOMINAL=0,
        EMERGENCY_STOP,
        DAMPING,
        VIRTUAL_ENDSTOP_ENFORCING,
    } safety_state;

    SafetyState getSafetyState() const;

    // Limits are loaded once by init() and cannot be changed afterwards.
    const TLimits& getLimits() const;

    void manageAnomalies(const TState& state, TCommands& commands, CONTROL_MODE control_mode);

    protected:
        TLimits limits;

    private:
        void setFungoPressed(bool pressed);
        /**
         * @brief Position thresholds shrunk by the distance needed to brake the joint 
         *        at its current velocity with the torque left after torque_command.
        
         * 
         * @param joint_index 
         * @param velocity 
         * @param torque_command 
         * @param threshold_min 
         * @param threshold_max 
         */
        void computePredictedPositionThresholds(size_t joint_index, 
                                                const double velocity, 
                                                const double torque,
                                                double& threshold_min, 
                                                double& threshold_max) const;
        void checkCommand(std::vector<double>& command , std::vector<double> min_limit, std::vector<double> max_limit, std::vector<bool>& anomaly_nan, std::vector<bool>& anomaly_inf, std::vector<bool>& anomaly_limit_min, std::vector<bool>& anomaly_limit_max, std::vector<double>& trigger_value);
        void setSafetyState();
        void executeRecoveryMode(const TState& state, TCommands& commands);

        // Anomaly auxiliary variables for anomaly detection
        // Timeout for detecting missing required high-level input messages.
        double references_timeout; // seconds since the latest required high-level input message
        // Rate at which the high-level interface's background thread checks
        // for events to log, decoupled from the control loop's frequency.
        double events_frequency; // Hz
        // Control loop period, needed for delay compensation
        double control_period; // seconds; 0 disables delay compensation
        // Below this joint speed the virtual joint limit is not activated.
        double virtual_joint_limit_min_velocity; // rad/s
        // Fraction of the torque limit the virtual joint limit trigger is sized on, in (0, 1].
        // Below 1 it triggers earlier, leaving the rest as headroom for model errors.
        double virtual_joint_limit_torque_scale;
        // Per joint: the virtual joint limit triggers only toward position_max (e.g. the knees,
        // stretched leg); approaching position_min never activates it.
        std::vector<bool> virtual_joint_limit_upper_only;
        // Nodes discovered by the low-level interface, see setDiscoveredNodes()
        std::vector<int> discovered_nodes;
        // fault_info a motor reports when it has no fault; empty disables the motor fault check.
        std::string motor_no_fault_info;
        double startup_failure_notify_duration; // seconds

        // Recovery procedures
        RecoveryESTOP recovery_estop;
        RecoveryDamping recovery_damping;
        RecoveryVirtualJointLimit recovery_virtual_joint_limit;
};

template <typename TState, typename TRef, typename TCommands>
class RobotLowLevelControl{
public:

    RobotLowLevelControl();

    void init(const YAML::Node& config);

    void computeCommands(const TState& state, const TRef& ref);

    void positionControl(const TState& state, const TRef& ref);
    void velocityControl(const TState& state, const TRef& ref);
    void torqueControl(const TState& state, const TRef& ref);
    void impedanceControl(const TState& state, const TRef& ref);
    void currentControl(const TState& state, const TRef& ref);

    CONTROL_MODE control_mode;

    TCommands commands;          // Accepted values sent to the hardware.
};

#include "robot_invariants.tpp"

#endif
