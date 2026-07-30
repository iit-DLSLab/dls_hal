#include "dls_hal/hal.hpp"
#include "dls_hal/config_interface_path.hpp"

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
void Hal<LLI, RI, HLI>::init(const std::string& hal_interface_config_file)
{
    // get yaml file and pass to the interfaces
    char * val;
    val = getenv("DLS_HAL_INTERFACE_PATH");
    std::string hal_interface_path = DEFAULT_DLS_HAL_INTERFACE_PATH + std::string("/") + hal_interface_config_file + ".yaml";
    if (val != NULL) {
        hal_interface_path = std::string(val) + "/" + hal_interface_config_file + ".yaml";
    }
    std::cout << "Loading robot interface configuration from: " << hal_interface_path << std::endl;
    YAML::Node config = YAML::LoadFile(hal_interface_path);

    ri.init(config);
    lli.init(ri, config);
    hli.init(ri, config);
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