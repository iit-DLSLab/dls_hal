#include "robot_invariants.hpp"
#include <iostream>
// RobotSafety
template <typename TState, typename TRef, typename TCommands, typename TLimits>
RobotSafety<TState, TRef, TCommands, TLimits>::RobotSafety(){
    e_stop_triggered = false;
}

template <typename TState, typename TRef, typename TCommands, typename TLimits>
void RobotSafety<TState, TRef, TCommands, TLimits>::init(const YAML::Node& config){
    e_stop_triggered = config["Safety"]["e_stop"].as<int>();
}

template <typename TState, typename TRef, typename TCommands, typename TLimits>
bool RobotSafety<TState, TRef, TCommands, TLimits>::checkState(const TState& state, const TLimits& limits) {return true;}
template <typename TState, typename TRef, typename TCommands, typename TLimits>
bool RobotSafety<TState, TRef, TCommands, TLimits>::checkReference(const TRef& ref, const TLimits& limits) { return true; }
template <typename TState, typename TRef, typename TCommands, typename TLimits>
bool RobotSafety<TState, TRef, TCommands, TLimits>::checkCommands(const TCommands& commands, const TLimits& limits) { return true; }

template <typename TState, typename TRef, typename TCommands, typename TLimits>
void RobotSafety<TState, TRef, TCommands, TLimits>::eSTOP(TCommands& commands) {
    // Set all commands to zero or safe values
    std::fill(commands.position.begin(), commands.position.end(), 0.0);
    std::fill(commands.velocity.begin(), commands.velocity.end(), 0.0);
    std::fill(commands.torque.begin(), commands.torque.end(), 0.0);
    std::cout << "Emergency stop activated. Commands set to safe values." << std::endl;
}

// RobotLowLevelControl
template <typename TState, typename TRef, typename TCommands>
RobotLowLevelControl<TState, TRef, TCommands>::RobotLowLevelControl(){}

template <typename TState, typename TRef, typename TCommands>
void RobotLowLevelControl<TState, TRef, TCommands>::init(const YAML::Node& config){
    // Initialization logic for low-level control based on the configuration
    control_mode = static_cast<CONTROL_MODE>(config["LowLevelControl"]["mode"].as<int>());
    commands.init(config);
}

template <typename TState, typename TRef, typename TCommands>
void RobotLowLevelControl<TState, TRef, TCommands>::computeCommands(const TState& state, const TRef& ref){
    // Implementation for computing commands based on state and reference
    if (control_mode == CONTROL_MODE::POSITION_CONTROL) {
        positionControl();
    } else if (control_mode == CONTROL_MODE::VELOCITY_CONTROL) {
        velocityControl();
    } else if (control_mode == CONTROL_MODE::TORQUE_CONTROL) {
        torqueControl();
    } else if (control_mode == CONTROL_MODE::IMPEDANCE_CONTROL) {
        impedanceControl();
    } else if (control_mode == CONTROL_MODE::CURRENT_CONTROL) {
        currentControl();
    }
}

template <typename TState, typename TRef, typename TCommands>
void RobotLowLevelControl<TState, TRef, TCommands>::positionControl() { std::cout << "Position control logic not implemented yet." << std::endl; }
template <typename TState, typename TRef, typename TCommands>
void RobotLowLevelControl<TState, TRef, TCommands>::velocityControl() { std::cout << "Velocity control logic not implemented yet." << std::endl; }
template <typename TState, typename TRef, typename TCommands>
void RobotLowLevelControl<TState, TRef, TCommands>::torqueControl() { std::cout << "Torque control logic not implemented yet." << std::endl; }
template <typename TState, typename TRef, typename TCommands>
void RobotLowLevelControl<TState, TRef, TCommands>::impedanceControl() { std::cout << "Impedance control logic not implemented yet." << std::endl; }
template <typename TState, typename TRef, typename TCommands>
void RobotLowLevelControl<TState, TRef, TCommands>::currentControl() { std::cout << "Current control logic not implemented yet." << std::endl; }
