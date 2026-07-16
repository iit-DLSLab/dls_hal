
#include "dls_hal/robot_interface.hpp"
#include "dls_hal/quadruped_invariants.hpp"

class QuadrupedInterface : public   RobotInterface<
                                        QuadrupedState,
                                        QuadrupedRef,
                                        QuadrupedCommands,
                                        QuadrupedLimits,
                                        QuadrupedSafety,
                                        QuadrupedLowLevelControl>{};