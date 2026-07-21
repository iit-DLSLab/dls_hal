#ifndef ROBOTS_INVARIANTS_HPP
#define ROBOTS_INVARIANTS_HPP

#include "vector"
#include "stdint.h"
#include "yaml-cpp/yaml.h"

class IMU{
public:
    IMU();
    void init(const YAML::Node& config);
    std::string frame_id;
    std::array<double, 4> orientation;
    std::array<double, 3> orientation_rpy;
    std::array<double, 9> orientation_covariance;
    std::array<double, 3> angular_velocity;
    std::array<double, 9> angular_velocity_covariance;
    std::array<double, 3> linear_acceleration;
    std::array<double, 9> linear_acceleration_covariance;
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
    std::vector<double> torques;
    std::vector<uint32_t> motor_status_word;
    std::vector<double> motor_temperature;
    std::vector<double> motor_current;
    std::vector<bool> contacts;
    std::vector<IMU> imu;
    double battery_status;
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
    std::vector<double> velocity_max;
    std::vector<double> torque_max;
    std::vector<double> current_max;
    std::vector<double> motor_temperature_min;
    std::vector<double> motor_temperature_max;
    double battery_charge_min;
};

template <typename TState, typename TRef, typename TCommands, typename TLimits>
class RobotSafety{
public:
    RobotSafety();
    
    void init(const YAML::Node& config);

    bool e_stop_triggered;
    
    virtual void eSTOP(TCommands& commands);
    virtual bool checkState(const TState& state);
    virtual bool checkReference(const TRef& ref);
    virtual bool checkCommands(const TCommands& commands);

    virtual bool checkJointStateLimit(double value, double min_limit, double max_limit);
    virtual bool checkJointStateNanInf(double value);

    void displayAnomalies() const;
    // Robot anomalies
    class RobotAnomalies{
    public:
        std::vector<bool> joint_pos_limit_violation;
        std::vector<bool> joint_vel_limit_violation;
        std::vector<bool> joint_torque_limit_violation;
        std::vector<bool> nan_position_detected;
        std::vector<bool> nan_velocity_detected;
        std::vector<bool> nan_torque_detected;
        std::vector<bool> inf_position_detected;
        std::vector<bool> inf_velocity_detected;
        std::vector<bool> inf_torque_detected;
        std::vector<bool> motor_overheat_detected;
        bool battery_charge_low;
    } anomalies;

    TLimits limits;
};

template <typename TState, typename TRef, typename TCommands>
class RobotLowLevelControl{
public:
    enum class CONTROL_MODE{
        POSITION_CONTROL,
        VELOCITY_CONTROL,
        TORQUE_CONTROL,
        IMPEDANCE_CONTROL,
        CURRENT_CONTROL
    };

    RobotLowLevelControl();

    void init(const YAML::Node& config);

    void computeCommands(const TState& state, const TRef& ref);

    void positionControl(const TState& state, const TRef& ref);
    void velocityControl(const TState& state, const TRef& ref);
    void torqueControl(const TState& state, const TRef& ref);
    void impedanceControl(const TState& state, const TRef& ref);
    void currentControl(const TState& state, const TRef& ref);

    CONTROL_MODE control_mode;

    TCommands commands;
};

#include "robot_invariants.tpp"

#endif