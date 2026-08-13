# pragma once

#include <iostream>
// LLI: Low-Level Interface
// RI: Robot Interface
// HLI: High-Level Interface
class HalBase{
public:
    HalBase(){};
    ~HalBase(){};

    virtual void init(   const YAML::Node& robot_interface_config,
                                const YAML::Node& low_level_interface_config,
                                const YAML::Node& high_level_interface_config) = 0;

    virtual void run() = 0;

    virtual bool checkHighLevelInterface() = 0;

    virtual bool checkLowLevelInterface() = 0;
};