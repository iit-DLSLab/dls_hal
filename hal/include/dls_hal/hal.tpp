#include "dls_hal/hal.hpp"

#define HAL_TEMPLATE template <typename LLI, typename RI, typename HLI>
#define HAL_TYPE Hal<LLI, RI, HLI>

HAL_TEMPLATE
HAL_TYPE::Hal()
    : lli()
    , ri()
    , hli()
{
    //TODO check base class of LLI, RI, HLI to ensure they are derived from the correct base classes
}

template <typename LLI, typename RI, typename HLI>
Hal<LLI, RI, HLI>::~Hal()
{}

template <typename LLI, typename RI, typename HLI>
void Hal<LLI, RI, HLI>::init(   const YAML::Node& robot_interface_config,
                                const YAML::Node& low_level_interface_config,
                                const YAML::Node& high_level_interface_config)
{
    ri.init(robot_interface_config);
    lli.init(ri, low_level_interface_config);
    hli.init(ri, high_level_interface_config);

    // initialize robot reference position with actual robot position, to avoid sudden jumps in the robot motion when starting the application
    if(checkLowLevelInterface()){
        lli.read(ri);
        ri.checkState();
        ri.ref.initReference(ri.state);
    }
}

template <typename LLI, typename RI, typename HLI>
bool Hal<LLI, RI, HLI>::run()
{
    // checking if low level communication is working properly, if not return false
    if(!checkLowLevelInterface()){
        return false;
    }

    lli.read(ri);
    ri.checkState();
    hli.write(ri);

    hli.read(ri);
    ri.checkReference();

    ri.computeCommands();
    ri.checkCommands();

    lli.write(ri);

    return true;
}

template <typename LLI, typename RI, typename HLI>
bool Hal<LLI, RI, HLI>::checkHighLevelInterface()
{
    return hli.check();
}

template <typename LLI, typename RI, typename HLI>
bool Hal<LLI, RI, HLI>::checkLowLevelInterface()
{
    return lli.check();
}