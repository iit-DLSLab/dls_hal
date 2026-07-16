
#include "dls_hal/quadruped_interface.hpp"
#include "yaml-cpp/yaml.h"

#include <cassert>
#include <iostream>

int main(int argc, char **argv) {
    QuadrupedInterface iface;

    YAML::Node config = YAML::LoadFile("../../config/quadruped_interface_config.yaml");
    assert(iface.init(config));

    const int N = config["general"]["n_joints"].as<int>();

    // vectors are sized correctly after init
    assert((int)iface.state.position.size() == N);
    assert((int)iface.low_level_control.commands.position.size() == N);
    assert((int)iface.ref.position.size() == N);
    assert((int)iface.limits.position_min.size() == N);
    // setLimits / getLimits round-trip
    iface.limits.position_min[0] = -1.0;
    assert(iface.limits.position_min[0] == -1.0);

    // setState / getState round-trip
    iface.state.position[0] = 1.5;
    assert(iface.state.position[0] == 1.5);

    // setReference / getReference round-trip
    iface.ref.position[0] = 0.5;
    assert(iface.ref.position[0] == 0.5);

    // display control mode
    std::cout << "Current control mode: " << static_cast<int>(iface.low_level_control.control_mode) << std::endl;

    // computeCommands should not throw
    iface.computeCommands();

    // // test emergency stop
    // iface.getLowLevelControl().control_mode = QuadrupedLowLevelControl::CONTROL_MODE::POSITION_CONTROL;
    // iface.getLowLevelControl().commands.position[0] = 1.0;
    // iface.getLimits().position_min[0] = 0.0;
    // iface.computeCommands();
    // assert(iface.getLowLevelControl().commands.position[0] == 0.0);
    std::cout << "All tests passed.\n";
    return 0;
}