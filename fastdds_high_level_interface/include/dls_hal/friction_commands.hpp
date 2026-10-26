#pragma once

#include "dls_hal/hl_command.hpp"

#include <cstddef>

// setFrictionCompensation <0|1>: disables/enables the joint friction compensation of the impedance control.
class SetFrictionCompensationCommand : public HLCommand<GenericRobotInterface, bool, bool> {
public:
    SetFrictionCompensationCommand();

protected:
    bool parse(bool enabled, bool& data) override;
    void apply(const bool& enabled, GenericRobotInterface& source) override;
};

struct FrictionGains {
    int joint;  // -1 for all joints
    double k_static;
    double k_dynamic;
};

// setFrictionGains <joint> <k_static> <k_dynamic>: sets the static (Coulomb) and dynamic (viscous)
// friction gains of a joint, or of all joints with joint = -1.
class SetFrictionGainsCommand : public HLCommand<GenericRobotInterface, FrictionGains, int, double, double> {
public:
    // num_joints bounds the accepted joint index.
    explicit SetFrictionGainsCommand(std::size_t num_joints);

protected:
    bool parse(int joint, double k_static, double k_dynamic, FrictionGains& data) override;
    void apply(const FrictionGains& gains, GenericRobotInterface& source) override;

private:
    int num_joints_;
};
