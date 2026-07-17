#ifndef ARM_INVARIANTS_HPP
#define ARM_INVARIANTS_HPP

#include "dls_hal/robot_invariants.hpp"

#include "vector"
#include "stdint.h"
#include "yaml-cpp/yaml.h"

class ArmState : public RobotState{};

class ArmCommands : public RobotCommands{};

class ArmRef : public RobotRef{};

class ArmLimits : public RobotLimits{};

class ArmSafety : public RobotSafety<ArmState, ArmRef, ArmCommands, ArmLimits>{};

class ArmLowLevelControl : public RobotLowLevelControl<ArmState, ArmRef, ArmCommands>{};

#endif