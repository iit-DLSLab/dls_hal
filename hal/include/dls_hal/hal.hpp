# pragma once

// LLI: Low-Level Interface
// RI: Robot Interface
// HLI: High-Level Interface
template <typename LLI, typename RI, typename HLI>
class Hal{
public:
    Hal();
    ~Hal();

    void init();

    void run();

    LLI lli; // Low-Level Interface for reading/writing to the robot
    RI ri; // Robot Interface for managing robot state and commands
    HLI hli; // High-Level Interface for communication with external systems
};

#include "dls_hal/hal.tpp"