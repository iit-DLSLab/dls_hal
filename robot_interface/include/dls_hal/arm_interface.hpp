
#include "dls_hal/robot_interface.hpp"
#include "dls_hal/arm_invariants.hpp"

class ArmInterface : public   RobotInterface<
                                        ArmState,
                                        ArmRef,
                                        ArmCommands,
                                        ArmLimits,
                                        ArmSafety,
                                        ArmLowLevelControl>{};