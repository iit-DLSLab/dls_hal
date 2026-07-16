#include "robot_invariants.hpp"
#include <iostream>
// RobotSafety
template <typename TState, typename TRef, typename TCommands, typename TLimits>
RobotSafety<TState, TRef, TCommands, TLimits>::RobotSafety(){
    e_stop_triggered = false;
}

template <typename TState, typename TRef, typename TCommands, typename TLimits>
void RobotSafety<TState, TRef, TCommands, TLimits>::init(YAML::Node& config){
    e_stop_triggered = config["Safety"]["e_stop"].as<int>();
}

template <typename TState, typename TRef, typename TCommands, typename TLimits>
bool RobotSafety<TState, TRef, TCommands, TLimits>::checkState(TState state, TLimits limits) { 
    if (state.joints_position[0] < limits.position_min[0])
        std::cout << "Position limit violated for joint 0: " << state.joints_position[0] << " < " << limits.position_min[0] << std::endl;
    else if (state.joints_position[0] > limits.position_max[0])
        std::cout << "Position limit violated for joint 0: " << state.joints_position[0] << " > " << limits.position_max[0] << std::endl;
    else if (state.joints_velocity[0] > limits.velocity_max[0])
        std::cout << "Velocity limit violated for joint 0: " << state.joints_velocity[0] << " > " << limits.velocity_max[0] << std::endl;
    else if (state.joints_torques[0] > limits.torque_max[0])
        std::cout << "Torque limit violated for joint 0: " << state.joints_torques[0] << " > " << limits.torque_max[0] << std::endl;
    else if (state.motor_current[0] > limits.current_max[0])
        std::cout << "Current limit violated for joint 0: " << state.motor_current[0] << " > " << limits.current_max[0] << std::endl;
    else
        return true;
    return true;
}
template <typename TState, typename TRef, typename TCommands, typename TLimits>
bool RobotSafety<TState, TRef, TCommands, TLimits>::checkReference(TRef ref, TLimits limits) { return true; }
template <typename TState, typename TRef, typename TCommands, typename TLimits>
bool RobotSafety<TState, TRef, TCommands, TLimits>::checkCommands(TCommands commands, TLimits limits) { return true; }

// RobotLowLevelControl
template <typename TState, typename TRef, typename TCommands>
RobotLowLevelControl<TState, TRef, TCommands>::RobotLowLevelControl(){}

template <typename TState, typename TRef, typename TCommands>
void RobotLowLevelControl<TState, TRef, TCommands>::init(YAML::Node& config){
    // Initialization logic for low-level control based on the configuration
    control_mode = static_cast<CONTROL_MODE>(config["LowLevelControl"]["mode"].as<int>());
    commands.init(config);
}

template <typename TState, typename TRef, typename TCommands>
void RobotLowLevelControl<TState, TRef, TCommands>::computeCommands(TState state, TRef ref){
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
