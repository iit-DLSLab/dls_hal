#ifndef ROBOT_INTERFACE_HPP
#define ROBOT_INTERFACE_HPP

#include "yaml-cpp/yaml.h"
#include "dls_hal/config.hpp"
#include "robot_invariants.hpp"

template <typename TState, typename TRef, typename TCommands, typename TLimits, typename TSafety, typename TLowLevelControl>
class RobotInterface{
public:
    RobotInterface();
    ~RobotInterface();

    // computations of commands based on actual state, reference and configuration
    void computeCommands();

    // function initialization
    bool init(const std::string& config_file);

    void initReferences(const TRef& ref);

    void checkReference();

    void checkState();

    void checkCommands();
    
    TSafety safety; // E-stop, clamping, limit checks
    TRef ref;
    TState state;
    TLowLevelControl low_level_control;
};

#include "robot_interface.tpp"
#endif

