#include "dls_hal/friction_commands.hpp"

#include <iostream>
#include <string>

SetFrictionCompensationCommand::SetFrictionCompensationCommand()
    : HLCommand("setFrictionCompensation",
                "Enable (1) or disable (0) the joint friction compensation of the impedance control")
{}

bool SetFrictionCompensationCommand::parse(bool enabled, bool& data)
{
    data = enabled;
    std::cout << name() << ": friction compensation " << (enabled ? "enabled" : "disabled") << std::endl;
    return true;
}

void SetFrictionCompensationCommand::apply(const bool& enabled, GenericRobotInterface& source)
{
    source.low_level_control.setFrictionCompensation(enabled);
}

SetFrictionGainsCommand::SetFrictionGainsCommand(std::size_t num_joints)
    : HLCommand("setFrictionGains",
                "Set the static (Coulomb) and dynamic (viscous) friction gains of a joint: joint (-1 for all), k_static, k_dynamic")
    , num_joints_(static_cast<int>(num_joints))
{}

bool SetFrictionGainsCommand::parse(int joint, double k_static, double k_dynamic, FrictionGains& data)
{
    if (joint < -1 || joint >= num_joints_) {
        std::cerr << name() << ": joint " << joint << " out of range [-1, " << num_joints_ - 1 << "]" << std::endl;
        return false;
    }
    if (k_static < 0.0 || k_dynamic < 0.0) {
        std::cerr << name() << ": gains must be non-negative" << std::endl;
        return false;
    }
    data = {joint, k_static, k_dynamic};
    std::cout << name() << ": friction gains of " << (joint == -1 ? std::string("all joints") : "joint " + std::to_string(joint))
              << " set to k_static=" << k_static << ", k_dynamic=" << k_dynamic << std::endl;
    return true;
}

void SetFrictionGainsCommand::apply(const FrictionGains& gains, GenericRobotInterface& source)
{
    source.low_level_control.setFrictionGains(gains.joint, gains.k_static, gains.k_dynamic);
}
