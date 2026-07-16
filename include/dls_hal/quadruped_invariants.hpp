#ifndef QUADRUPED_INVARIANTS_HPP
#define QUADRUPED_INVARIANTS_HPP

#include "dls_hal/robot_invariants.hpp"

#include "vector"
#include "stdint.h"
#include "yaml-cpp/yaml.h"

class QuadrupedState : public RobotState{};

class QuadrupedCommands : public RobotCommands{};

class QuadrupedRef : public RobotRef{};

class QuadrupedLimits : public RobotLimits{};

class QuadrupedSafety : public RobotSafety<QuadrupedState, QuadrupedRef, QuadrupedCommands, QuadrupedLimits>{
public:
    bool checkState(const QuadrupedState& state, const QuadrupedLimits& limits) override{
        RobotSafety<QuadrupedState, QuadrupedRef, QuadrupedCommands, QuadrupedLimits>::checkState(state, limits);
        return true;
    }
};

class QuadrupedLowLevelControl : public RobotLowLevelControl<QuadrupedState, QuadrupedRef, QuadrupedCommands>{};

#endif