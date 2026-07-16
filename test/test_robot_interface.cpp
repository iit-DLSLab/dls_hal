
#include "dls_hal/quadruped_interface.hpp"
#include "yaml-cpp/yaml.h"

#include <cassert>
#include <iostream>

int main(int argc, char **argv) {
    QuadrupedInterface iface;

    YAML::Node config = YAML::LoadFile("quadruped_interface_config.yaml");
    assert(iface.init(config));

    const int N = config["general"]["n_joints"].as<int>();

    // vectors are sized correctly after init
    assert((int)iface.getState().joints_position.size() == N);
    assert((int)iface.getCommands().position.size() == N);
    assert((int)iface.getReference().position.size() == N);
    assert((int)iface.getLimits().position_min.size() == N);
    // setLimits / getLimits round-trip
    QuadrupedLimits limits;
    limits.init(config);
    limits.position_min[0] = -1.0;
    iface.setLimits(limits);
    assert(iface.getLimits().position_min[0] == -1.0);

    // setState / getState round-trip
    QuadrupedState state;
    state.init(config);
    state.joints_position[0] = 1.5;
    iface.setState(state);
    assert(iface.getState().joints_position[0] == 1.5);

    // setReference / getReference round-trip
    QuadrupedRef ref;
    ref.init(config);
    ref.position[0] = 0.5;
    iface.setReference(ref);
    assert(iface.getReference().position[0] == 0.5);

    // displaty control mode
    std::cout << "Current control mode: " << static_cast<int>(iface.getLowLevelControl().control_mode) << std::endl;

    // computeCommands should not throw
    iface.computeCommands();

    // test emergency stop
    iface.getLowLevelControl().control_mode = QuadrupedLowLevelControl::CONTROL_MODE::POSITION_CONTROL;
    iface.getLowLevelControl().commands.position[0] = 1.0;
    iface.getLimits().position_min[0] = 0.0;
    iface.computeCommands();
    assert(iface.getLowLevelControl().commands.position[0] == 0.0);
    std::cout << "All tests passed.\n";
    return 0;
}