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
    double torso_temperature;
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

template <typename TState, typename TRef, typename TCommands, typename TLimits>
class RobotSafety{
public:
    RobotSafety();
    
    void init(const YAML::Node& config);
    
    virtual void checkState(const TState& state);

    virtual void checkCommands(TCommands& commands);

    virtual bool checkJointStateLimit(double value, double min_limit, double max_limit);
    virtual bool checkJointStateNanInf(double value);

    void displayAnomalies() const;

    bool isFungoPressed() const;

    void triggerEmergencyStop();

    void setReferencesNotReceived(bool not_received);

    bool isEmergencyStopActive() const;

    // Robot anomalies
    class RobotAnomalies{
    public:
        std::vector<bool> joint_pos_limit_violation; //std::map<RECOVERY_MODE, bool>
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
        std::atomic_bool emergency_stop_triggered;
        // Per-cycle status flag written by the control thread and read by the
        // event thread; it does not publish any accompanying state.
        std::atomic_bool fungo_pressed;
        std::atomic_bool references_not_received;
        bool breaking_mechanics{false};
        // Preserve each joint's first triggering torque while the fault is latched.
        std::vector<std::optional<double>> mechanical_stress_trigger_torque;
    } anomalies;

    TLimits limits;

    bool manageAnomalies(const TState& state, TCommands& commands);

    private:
        void setFungoPressed(bool pressed);
        void checkBreakingMechanics(size_t joint_index, double torque);
        void checkCommand(std::vector<double>& command , std::vector<double> min_limit, std::vector<double> max_limit, std::vector<bool>& anomaly_nan, std::vector<bool>& anomaly_inf);

        // Anomaly auxiliary variables for anomaly detection
        // breaking_mechanics aux variables
        double mechanical_stress_max_time;
        double mechanical_stress_max_torque;
        std::vector<std::optional<std::chrono::steady_clock::time_point>> mechanical_stress_start_per_joint;

        // Recovery procedures
        RecoveryESTOP recovery_estop;
        RecoveryDamping recovery_damping;
};

enum class CONTROL_MODE{
    TORQUE_CONTROL=0,
    IMPEDANCE_CONTROL,
    POSITION_CONTROL,
    VELOCITY_CONTROL,
    CURRENT_CONTROL
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
