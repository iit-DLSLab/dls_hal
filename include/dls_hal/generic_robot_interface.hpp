
#include "dls_hal/robot_interface.hpp"
#include "dls_hal/robot_invariants.hpp"

class GenericRobotInterface : public   RobotInterface<
                                        RobotState,
                                        RobotRef,
                                        RobotCommands,
                                        RobotLimits,
                                        RobotSafety<RobotState, RobotRef, RobotCommands, RobotLimits>,
                                        RobotLowLevelControl<RobotState, RobotRef, RobotCommands>>{};