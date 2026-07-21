#include "dls_hal/hal.hpp"

#define HAL_TEMPLATE template <typename LLI, typename RI, typename HLI>
#define HAL_TYPE Hal<LLI, RI, HLI>

HAL_TEMPLATE
HAL_TYPE::Hal()
    : lli()
    , ri()
    , hli()
{}

template <typename LLI, typename RI, typename HLI>
Hal<LLI, RI, HLI>::~Hal()
{}

template <typename LLI, typename RI, typename HLI>
void Hal<LLI, RI, HLI>::init(const std::string& robot_interface_config_file)
{
    ri.init(robot_interface_config_file);
    lli.init(ri);
    hli.init(ri);
}

template <typename LLI, typename RI, typename HLI>
void Hal<LLI, RI, HLI>::run()
{
    lli.read(ri);
    ri.checkState();
    hli.write(ri);

    hli.read(ri);
    ri.checkReference();

    ri.computeCommands();
    ri.checkCommands();

    lli.write(ri);
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