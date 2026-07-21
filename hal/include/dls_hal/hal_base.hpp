# pragma once

// LLI: Low-Level Interface
// RI: Robot Interface
// HLI: High-Level Interface
class HalBase{
public:
    HalBase(){};
    ~HalBase(){};

    virtual void init() = 0;

    virtual void run() = 0;
};

#include "dls_hal/hal.tpp"