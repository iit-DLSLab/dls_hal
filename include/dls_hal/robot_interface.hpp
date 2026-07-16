#ifndef ROBOT_INTERFACE_HPP
#define ROBOT_INTERFACE_HPP

#include "yaml-cpp/yaml.h"
#include "robot_invariants.hpp"

template <typename TState, typename TRef, typename TCommands, typename TLimits, typename TSafety, typename TLowLevelControl>
class RobotInterface{
public:
    RobotInterface(){
        // check that the types are compatible
        static_assert(std::is_base_of<RobotState, TState>::value, "TState must be derived from RobotState");
        static_assert(std::is_base_of<RobotRef, TRef>::value, "TRef must be derived from RobotRef");
        static_assert(std::is_base_of<RobotCommands, TCommands>::value, "TCommands must be derived from RobotCommands");
        static_assert(std::is_base_of<RobotLimits, TLimits>::value, "TLimits must be derived from RobotLimits");
        static_assert(std::is_base_of<RobotSafety<TState, TRef, TCommands, TLimits>, TSafety>::value, "TSafety must be derived from RobotSafety<TState, TRef, TCommands, TLimits>");
        static_assert(std::is_base_of<RobotLowLevelControl<TState, TRef, TCommands>, TLowLevelControl>::value, "TLowLevelControl must be derived from RobotLowLevelControl<TState, TRef, TCommands>");
    };
    ~RobotInterface(){};

    // computations of commands based on actual state, reference and configuration
    void computeCommands(){
        // check safety status for e-stop
        if(safety.e_stop_triggered){
            std::cout << "Emergency stop triggered. Using E-stop reference." << std::endl;
            safety.eSTOP(low_level_control.commands);
        }
        else{
            low_level_control.computeCommands(state, ref);
        }

        safety.checkCommands(low_level_control.commands, limits);
    }

    // function initialization
    bool init(YAML::Node& config){
        state.init(config);
        ref.init(config);
        limits.init(config);
        safety.init(config);
        low_level_control.init(config);
        return true;
    }

    void initReferences(TRef& ref){
        this->ref = ref;
    }

    void setReference(TRef& ref){
        this->ref = ref;
        safety.checkReference(ref, limits);
    }

    void setState(TState& state){
        this->state = state;
        // check safety conditions based on the new state
        safety.checkState(state, limits);
    }

    void setLimits(TLimits& limits){
        this->limits = limits;
    }

    TState& getState(){
        return state;
    }

    TRef& getReference(){
        return ref;
    }

    TCommands& getCommands(){
        return low_level_control.commands;
    }

    TLimits& getLimits(){
        return limits;
    }

    TLowLevelControl& getLowLevelControl(){
        return low_level_control;
    }

private:
    TSafety safety; // E-stop, clamping, limit checks
    TRef ref;
    TState state;
    TLimits limits;
    TLowLevelControl low_level_control;
};

#endif