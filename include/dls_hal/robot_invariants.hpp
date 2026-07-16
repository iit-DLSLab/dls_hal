#ifndef ROBOTS_INVARIANTS_HPP
#define ROBOTS_INVARIANTS_HPP

#include "vector"
#include "stdint.h"
#include "yaml-cpp/yaml.h"

class RobotState{
public:
    RobotState();

    void init(YAML::Node& config);

    std::vector<double> joints_position;
    std::vector<double> joints_velocity;
    std::vector<double> joints_acceleration;
    std::vector<double> joints_torques;
    std::vector<uint32_t> motor_status_word;
    std::vector<double> motor_temperature;
    std::vector<double> motor_current;
};

class RobotCommands{
public:
    RobotCommands();
    void init(YAML::Node& config);
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> torque;
};

class RobotRef{
public:
    RobotRef();
    void init(YAML::Node& config);
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> torque;
};

class RobotLimits{
public:
    RobotLimits();
    void init(YAML::Node& config);
    std::vector<double> position_min;
    std::vector<double> position_max;
    std::vector<double> velocity_max;
    std::vector<double> torque_max;
    std::vector<double> current_max;
};

template <typename TState, typename TRef, typename TCommands, typename TLimits>
class RobotSafety{
public:
    RobotSafety();
    
    void init(YAML::Node& config);

    bool e_stop_triggered;

    virtual bool checkState(TState state, TLimits limits);
    bool checkReference(TRef ref, TLimits limits);
    bool checkCommands(TCommands commands, TLimits limits);
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

    void init(YAML::Node& config);

    void computeCommands(TState state, TRef ref);

    void positionControl();
    void velocityControl();
    void torqueControl();
    void impedanceControl();
    void currentControl();

    CONTROL_MODE control_mode;

    TCommands commands;
};

#include "robot_invariants.tpp"

#endif