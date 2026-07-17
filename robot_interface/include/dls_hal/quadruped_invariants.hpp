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

class QuadrupedSafety : public RobotSafety<QuadrupedState, QuadrupedRef, QuadrupedCommands, QuadrupedLimits>{};

class QuadrupedLowLevelControl : public RobotLowLevelControl<QuadrupedState, QuadrupedRef, QuadrupedCommands>{};

#endif