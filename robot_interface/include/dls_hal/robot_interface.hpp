#ifndef ROBOT_INTERFACE_HPP
#define ROBOT_INTERFACE_HPP

#include "yaml-cpp/yaml.h"
#include "robot_invariants.hpp"

template <typename TState, typename TRef, typename TCommands, typename TLimits, typename TSafety, typename TLowLevelControl>
class RobotInterface{
public:
    RobotInterface();
    ~RobotInterface();

    // computations of commands based on actual state, reference and configuration
    void computeCommands();

    // function initialization
    virtual bool init(const YAML::Node& config);

    virtual void initReferences(const TRef& ref);

    virtual void checkReference();

    virtual void checkState();

    virtual void checkCommands();
    
    TSafety safety; // E-stop, clamping, limit checks
    TRef ref;
    TState state;
    TLowLevelControl low_level_control;
};

#include "robot_interface.tpp"
#endif

