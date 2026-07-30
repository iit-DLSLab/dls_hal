# pragma once

#include "dls_hal/hal_base.hpp"

// LLI: Low-Level Interface
// RI: Robot Interface
// HLI: High-Level Interface
template <typename LLI, typename RI, typename HLI>
class Hal : public HalBase{
public:
    Hal();
    ~Hal();

    void init(const std::string& robot_interface_config_file) override;

    void run() override;

    bool checkHighLevelInterface() override;

    bool checkLowLevelInterface() override;

    LLI lli; // Low-Level Interface for reading/writing to the robot
    RI ri; // Robot Interface for managing robot state and commands
    HLI hli; // High-Level Interface for communication with external systems
};

#include "dls_hal/hal.tpp"