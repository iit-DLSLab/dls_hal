#include "yaml-cpp/yaml.h"
#include "robot_interface.hpp"

#define ROBOT_INTERFACE_TEMPLATE template <typename TState, typename TRef, typename TCommands, typename TLimits, typename TSafety, typename TLowLevelControl>
#define ROBOT_INTERFACE_TYPE RobotInterface<TState, TRef, TCommands, TLimits, TSafety, TLowLevelControl>

ROBOT_INTERFACE_TEMPLATE
ROBOT_INTERFACE_TYPE::RobotInterface(){
    // check that the types are compatible
    static_assert(std::is_base_of<RobotState, TState>::value, "TState must be derived from RobotState");
    static_assert(std::is_base_of<RobotRef, TRef>::value, "TRef must be derived from RobotRef");
    static_assert(std::is_base_of<RobotCommands, TCommands>::value, "TCommands must be derived from RobotCommands");
    static_assert(std::is_base_of<RobotSafety<TState, TRef, TCommands, TLimits>, TSafety>::value, "TSafety must be derived from RobotSafety<TState, TRef, TCommands, TLimits>");
    static_assert(std::is_base_of<RobotLowLevelControl<TState, TRef, TCommands>, TLowLevelControl>::value, "TLowLevelControl must be derived from RobotLowLevelControl<TState, TRef, TCommands>");
};
ROBOT_INTERFACE_TEMPLATE
ROBOT_INTERFACE_TYPE::~RobotInterface(){};

ROBOT_INTERFACE_TEMPLATE
void ROBOT_INTERFACE_TYPE::computeCommands(){
    // check safety status for e-stop
    if(safety.isEmergencyStopActive()){
        safety.eSTOP(low_level_control.commands);
    }
    else{
        low_level_control.computeCommands(state, ref);
    }

    safety.checkCommands(low_level_control.commands);
}

ROBOT_INTERFACE_TEMPLATE
bool ROBOT_INTERFACE_TYPE::init(const YAML::Node& config){
    state.init(config);
    ref.init(config);
    safety.init(config);
    low_level_control.init(config);
    return true;
}

ROBOT_INTERFACE_TEMPLATE
void ROBOT_INTERFACE_TYPE::checkReference(){
    safety.checkReference(ref);
}

ROBOT_INTERFACE_TEMPLATE
void ROBOT_INTERFACE_TYPE::checkState(){
    safety.checkState(state);
}

ROBOT_INTERFACE_TEMPLATE
void ROBOT_INTERFACE_TYPE::checkCommands(){
    safety.checkCommands(low_level_control.commands);
}