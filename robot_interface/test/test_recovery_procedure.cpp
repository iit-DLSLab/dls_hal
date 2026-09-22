#include "dls_hal/robot_invariants.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using Safety = RobotSafety<RobotState, RobotRef, RobotCommands, RobotLimits>;

void require(bool condition, const std::string& message) {
    // Keep checks active in Release builds, where assert() is disabled.
    if (!condition) throw std::runtime_error(message);
}

struct Fixture {
    Safety safety;
    RobotState state;
    RobotCommands commands;

    Fixture(double mechanical_stress_max_time = -1.0) {
        auto config = YAML::Load(R"(
general:
  n_joints: 3
  n_contacts: 0
  n_batteries: 0
  n_imus: 0
robot_interface:
  safety:
    limits:
      torque_max: [100, 100, 100]
      current_max: [100, 100, 100]
)");
        config["robot_interface"]["safety"]["mechanical_stress_max_time"] = mechanical_stress_max_time;
        config["robot_interface"]["safety"]["mechanical_stress_max_torque"] = 10.0;
        if (mechanical_stress_max_time > 0.0) {
            config["robot_interface"]["safety"]["limits"]["position_min"] = std::vector<double>(3, -1.0);
            config["robot_interface"]["safety"]["limits"]["position_max"] = std::vector<double>(3, 1.0);
        }
        state.init(config);
        commands.init(config);
        safety.init(config);
        state.const_motors_status_words.fungo_pressed = 1;
        state.velocity = {2.0, -3.0, 0.0};
        resetCommands();
    }

    void resetCommands() {
        commands.position = {1.0, 2.0, 3.0};
        commands.velocity = {4.0, 5.0, 6.0};
        commands.torque = {7.0, 8.0, 9.0};
    }

    void recover() { safety.manageAnomalies(state, commands); }

    void expectRecovery(const std::vector<double>& torque) {
        require(commands.position == std::vector<double>(3, 0.0), "position commands must be zero");
        require(commands.velocity == std::vector<double>(3, 0.0), "velocity commands must be zero");
        require(commands.torque == torque, "unexpected recovery torque");
    }
};

void healthyCycle() {
    Fixture f;
    const auto original = f.commands;
    f.recover();
    require(f.commands.position == original.position &&
            f.commands.velocity == original.velocity &&
            f.commands.torque == original.torque, "healthy commands changed");
    require(!f.safety.isEmergencyStopActive(), "unexpected emergency stop");
}

void emergencyStopLatches() {
    Fixture f;
    f.safety.triggerEmergencyStop();
    for (int cycle = 0; cycle < 2; ++cycle) {
        f.resetCommands();
        f.recover();
        f.expectRecovery({0.0, 0.0, 0.0});
        require(f.safety.isEmergencyStopActive(), "emergency stop did not stay latched");
    }
}

void missingReferences() {
    Fixture f;
    f.safety.anomalies.references_not_received.store(true, std::memory_order_relaxed);
    f.recover();
    f.expectRecovery({-2.0, 3.0, 0.0});
    require(!f.safety.isEmergencyStopActive(), "damping latched emergency stop");
    f.safety.anomalies.references_not_received.store(false, std::memory_order_relaxed);
    f.resetCommands();
    f.recover();
    require(f.commands.torque == std::vector<double>({7.0, 8.0, 9.0}),
            "damping continued after references returned");
}

void mechanicalFault() {
    Fixture f;
    f.safety.anomalies.breaking_mechanics = true;
    f.recover();
    f.expectRecovery({-2.0, 3.0, 0.0});
}

void mechanicalFaultCapturesTrigger() {
    Fixture f(0.001);
    f.state.position = {0.0, -2.0, 2.0};
    f.state.torque = {0.0, -12.5, 15.0};
    f.safety.checkState(f.state);
    require(!f.safety.anomalies.breaking_mechanics, "stress latched before the time threshold");
    const auto& triggers = f.safety.anomalies.mechanical_stress_trigger_torque;
    require(triggers == std::vector<std::optional<double>>(3), "trigger recorded before recovery");
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    f.recover();
    f.expectRecovery({-2.0, 3.0, 0.0});
    require(f.safety.anomalies.breaking_mechanics, "missing stress anomaly");
    require(!triggers[0] && triggers[1] == -12.5 && triggers[2] == 15.0,
            "must record all affected joints and exclude healthy joints");

    // Add a later fault without replacing the original samples or duplicating joints.
    f.state.position = {2.0, -2.0, 0.0};
    f.state.torque = {30.0, -20.0, 0.0};
    f.safety.checkState(f.state);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    f.recover();
    require(triggers.size() == 3 && triggers[0] == 30.0 && triggers[1] == -12.5 && triggers[2] == 15.0,
            "later joint missing or latched torques overwritten");

    f.safety.init(YAML::Load(
        "general: {n_joints: 3, n_batteries: 0}\nrobot_interface: {safety: {limits: {}}}"));
    require(!f.safety.anomalies.breaking_mechanics &&
            triggers == std::vector<std::optional<double>>(3), "reinitialization retained stress triggers");
}

void mechanicalFaultRequiresPositionAndTorque() {
    Fixture f(0.001);
    // Excessive torque inside the limits, position violation alone, and exact boundary.
    f.state.position = {0.0, 2.0, 1.0};
    f.state.torque = {20.0, 0.0, -20.0};
    f.safety.checkState(f.state);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    f.safety.checkState(f.state);
    require(!f.safety.anomalies.breaking_mechanics, "stress requires both position and torque violations");
    f.state.position[2] = -1.0;
    f.safety.checkState(f.state);
    require(!f.safety.anomalies.breaking_mechanics, "position at the lower limit must not trigger stress");
    require(f.safety.anomalies.mechanical_stress_trigger_torque ==
            std::vector<std::optional<double>>(3), "ineligible joint recorded a stress trigger");
}

void mechanicalFaultTimerResets() {
    for (bool reset_position : {true, false}) {
        Fixture f(0.001);
        f.state.position[0] = 2.0;
        f.state.torque[0] = 20.0;
        f.safety.checkState(f.state);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        if (reset_position) f.state.position[0] = 0.0;
        else f.state.torque[0] = 0.0;
        f.safety.checkState(f.state);
        f.state.position[0] = 2.0;
        f.state.torque[0] = 20.0;
        f.safety.checkState(f.state);
        require(!f.safety.anomalies.breaking_mechanics, "stress timer did not reset");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        f.safety.checkState(f.state);
        require(f.safety.anomalies.breaking_mechanics, "sustained combined violation must trigger stress");
    }
}

void emergencyStopTakesPriority() {
    Fixture f;
    f.safety.anomalies.references_not_received.store(true, std::memory_order_relaxed);
    f.safety.anomalies.breaking_mechanics = true;
    f.safety.triggerEmergencyStop();
    f.recover();
    f.expectRecovery({0.0, 0.0, 0.0});
}

void fungoTakesPriorityAndClears() {
    Fixture f;
    f.safety.anomalies.references_not_received.store(true, std::memory_order_relaxed);
    // A press on any joint must stop all joints.
    f.state.motors_status_word[1] = f.state.const_motors_status_words.fungo_pressed;
    f.recover();
    require(f.safety.isFungoPressed(), "fungo press was not detected");
    f.expectRecovery({0.0, 0.0, 0.0});
    f.state.motors_status_word[1] = 0;
    f.resetCommands();
    f.recover();
    require(!f.safety.isFungoPressed(), "fungo flag stayed set after release");
    f.expectRecovery({-2.0, 3.0, 0.0});
}

void invalidCommandsTriggerStop() {
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()}) {
        for (int field = 0; field < 3; ++field) {
            Fixture f;
            f.safety.anomalies.references_not_received.store(true, std::memory_order_relaxed);
            auto& values = field == 0 ? f.commands.position :
                           field == 1 ? f.commands.velocity : f.commands.torque;
            values[1] = invalid;
            f.recover();
            require(f.safety.isEmergencyStopActive(), "invalid command did not trigger stop");
            f.expectRecovery({0.0, 0.0, 0.0});
        }
    }
}

void invalidVelocityDoesNotProduceInvalidTorque() {
    Fixture f;
    f.safety.anomalies.references_not_received.store(true, std::memory_order_relaxed);
    f.state.velocity = {std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::infinity(), -3.0};
    f.recover();
    f.expectRecovery({0.0, 0.0, 3.0});
}
} // namespace

int main() {
    const std::pair<const char*, void (*)()> tests[] = {
        {"healthy cycle", healthyCycle},
        {"emergency stop latch", emergencyStopLatches},
        {"missing references and restoration", missingReferences},
        {"mechanical fault", mechanicalFault},
        {"mechanical fault trigger details", mechanicalFaultCapturesTrigger},
        {"mechanical fault position and torque conditions", mechanicalFaultRequiresPositionAndTorque},
        {"mechanical fault timer reset", mechanicalFaultTimerResets},
        {"emergency stop priority", emergencyStopTakesPriority},
        {"fungo priority and release", fungoTakesPriorityAndClears},
        {"invalid commands", invalidCommandsTriggerStop},
        {"invalid measured velocity", invalidVelocityDoesNotProduceInvalidTorque},
    };
    int failures = 0;
    for (const auto& [name, run] : tests) {
        try {
            run();
            std::cout << "PASS: " << name << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL: " << name << ": " << error.what() << '\n';
        }
    }
    return failures == 0 ? 0 : 1;
}
