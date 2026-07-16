#include "robot_invariants.hpp"
#include <iostream>

#define ROBOT_SAFETY_TEMPLATE template <typename TState, typename TRef, typename TCommands, typename TLimits>
#define ROBOT_SAFETY_TYPE RobotSafety<TState, TRef, TCommands, TLimits>

#define ROBOT_LLC_TEMPLATE template <typename TState, typename TRef, typename TCommands>
#define ROBOT_LLC_TYPE RobotLowLevelControl<TState, TRef, TCommands>

// RobotSafety
ROBOT_SAFETY_TEMPLATE
ROBOT_SAFETY_TYPE::RobotSafety(){
    e_stop_triggered = false;
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::init(const YAML::Node& config){
    e_stop_triggered = config["safety"]["e_stop"].as<int>();
}

ROBOT_SAFETY_TEMPLATE
bool ROBOT_SAFETY_TYPE::checkState(const TState& state, const TLimits& limits) {return true;}

ROBOT_SAFETY_TEMPLATE
bool ROBOT_SAFETY_TYPE::checkReference(const TRef& ref, const TLimits& limits) { return true; }

ROBOT_SAFETY_TEMPLATE
bool ROBOT_SAFETY_TYPE::checkCommands(const TCommands& commands, const TLimits& limits) { return true; }

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::eSTOP(TCommands& commands) {
    // Set all commands to zero or safe values
    std::fill(commands.position.begin(), commands.position.end(), 0.0);
    std::fill(commands.velocity.begin(), commands.velocity.end(), 0.0);
    std::fill(commands.torque.begin(), commands.torque.end(), 0.0);
    std::cout << "Emergency stop activated. Commands set to safe values." << std::endl;
}

// RobotLowLevelControl
ROBOT_LLC_TEMPLATE
ROBOT_LLC_TYPE::RobotLowLevelControl(){}

ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::init(const YAML::Node& config){
    // Initialization logic for low-level control based on the configuration
    control_mode = static_cast<CONTROL_MODE>(config["low_level_control"]["mode"].as<int>());
    commands.init(config);
}

ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::computeCommands(const TState& state, const TRef& ref){
    // Implementation for computing commands based on state and reference
    if (control_mode == CONTROL_MODE::POSITION_CONTROL) {
        positionControl(state, ref);
    } else if (control_mode == CONTROL_MODE::VELOCITY_CONTROL) {
        velocityControl(state, ref);
    } else if (control_mode == CONTROL_MODE::TORQUE_CONTROL) {
        torqueControl(state, ref);
    } else if (control_mode == CONTROL_MODE::IMPEDANCE_CONTROL) {
        impedanceControl(state, ref);
    } else if (control_mode == CONTROL_MODE::CURRENT_CONTROL) {
        currentControl(state, ref);
    }
}

ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::positionControl(const TState& state, const TRef& ref) {
    commands.position = ref.position; // Simple position control logic    
}
ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::velocityControl(const TState& state, const TRef& ref) {
    commands.velocity = ref.velocity; // Simple velocity control logic
}
ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::torqueControl(const TState& state, const TRef& ref) {
    commands.torque = ref.torque; // Simple torque control logic
}
ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::impedanceControl(const TState& state, const TRef& ref) {
    // Impedance control logic (to be implemented)
    for (int id = 0; id < state.position.size(); ++id) {
        commands.torque[id] =   ref.torque[id] 
                                + ref.kp[id] * (ref.position[id] - state.position[id])
                                + ref.kd[id] * (ref.velocity[id] - state.velocity[id]);
    }
}
ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::currentControl(const TState& state, const TRef& ref) { 
    // Current control logic (to be implemented)
    std::cout << "Current control logic not implemented yet." << std::endl;
}

#undef ROBOT_SAFETY_TEMPLATE
#undef ROBOT_SAFETY_TYPE
#undef ROBOT_LLC_TEMPLATE
#undef ROBOT_LLC_TYPE