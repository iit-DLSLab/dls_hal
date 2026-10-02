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

class MotorStatusWords{
public:
    unsigned int fungo_pressed=0;
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
    double percentage_battery_charge_min;
};

class RobotAnomalies{
public:
    
    std::vector<bool> joint_pos_limit_violation;
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
    std::vector<bool> battery_charge_low;

    // One-way latch written by the command thread and read by the real-time
    // control thread; no blocking lock is needed.
    RelaxedAtomicBool emergency_stop_triggered;
    // Per-cycle status flag written by the control thread and read by the
    // event thread; it does not publish any accompanying state.
    RelaxedAtomicBool fungo_pressed;
    RelaxedAtomicBool lost_references;
    bool breaking_mechanics{false};
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

    void triggerEmergencyStop();

    void setLostReferences(bool lost);

    bool isEmergencyStopActive() const;

    double getReferencesTimeout() const;

    double getEventsFrequency() const;

    // Each joint's first triggering torque while breaking_mechanics is
    // latched (see mechanical_stress_trigger_torque below).
    const std::vector<std::optional<double>>& getMechanicalStressTriggerTorque() const;

    // Robot anomalies
    RobotAnomalies anomalies;

    enum class SafetyState{
        NOMINAL=0,
        EMERGENCY_STOP,
        DAMPING
    } safety_state;

    SafetyState getSafetyState() const;

    TLimits limits;

    void manageAnomalies(const TState& state, TCommands& commands, CONTROL_MODE control_mode);

    private:
        void setFungoPressed(bool pressed);
        void checkBreakingMechanics(size_t joint_index, double torque);
        void checkCommand(std::vector<double>& command , std::vector<double> min_limit, std::vector<double> max_limit, std::vector<bool>& anomaly_nan, std::vector<bool>& anomaly_inf, std::vector<bool>& anomaly_limit_min, std::vector<bool>& anomaly_limit_max, std::vector<double>& trigger_value);
        void setSafetyState();
        void executeRecoveryMode(const TState& state, TCommands& commands);

        // Anomaly auxiliary variables for anomaly detection
        // breaking_mechanics aux variables
        double mechanical_stress_exp_time_scale;
        double mechanical_stress_exp_decay;
        double mechanical_stress_max_torque;
        std::vector<std::optional<std::chrono::steady_clock::time_point>> mechanical_stress_start_per_joint;
        // Preserve each joint's first triggering torque while breaking_mechanics is latched.
        std::vector<std::optional<double>> mechanical_stress_trigger_torque;
        // Timeout for detecting missing required high-level input messages.
        double references_timeout; // seconds since the latest required high-level input message
        // Rate at which the high-level interface's background thread checks
        // for events to log, decoupled from the control loop's frequency.
        double events_frequency; // Hz

        // Recovery procedures
        RecoveryESTOP recovery_estop;
        RecoveryDamping recovery_damping;
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
