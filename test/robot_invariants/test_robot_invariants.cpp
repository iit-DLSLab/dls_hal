// Unit tests for the anomaly detection in robot_invariants.hpp.
//
// Every field of RobotAnomalies is covered by at least one test named after
// it, so the gtest output maps one-to-one to the anomaly list. RobotSafety is
// exercised directly (no HAL loop, no transport): state and commands are
// filled by hand, then checkState / checkCommands / manageAnomalies are run.

#include "dls_hal/robot_invariants.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

using Safety = RobotSafety<RobotState, RobotRef, RobotCommands, RobotLimits>;
using SafetyState = Safety::SafetyState;
// Control mode of the fixture config (mode: 1), used by the safety-state tests.
constexpr CONTROL_MODE kMode = CONTROL_MODE::IMPEDANCE_CONTROL;
constexpr CONTROL_MODE kCheckedModes[] = {
    CONTROL_MODE::POSITION_CONTROL, CONTROL_MODE::VELOCITY_CONTROL, CONTROL_MODE::IMPEDANCE_CONTROL};

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr unsigned int kFungoStatusWord = 4145;
constexpr const char* kNoFaultInfo = "NoFault";

// Distinct limits per joint, so a wrong index shows up as a wrong flag.
constexpr const char* kConfig = R"(
general:
  n_joints: 3
  n_contacts: 4
  n_imus: 1
  n_batteries: 2
  battery_full_voltage: 50.0
robot_interface:
  references_timeout: 0.5
  events_frequency: 10.0
  low_level_control:
    mode: 1
  safety:
    mechanical_stress_max_torque: 30.0
    motor_no_fault_info: NoFault
    limits:
      position_min_with_margin: [-1.0, -2.0, -3.0]
      position_max_with_margin: [1.0, 2.0, 3.0]
      velocity_max: [10.0, 20.0, 30.0]
      torque_max: [40.0, 50.0, 60.0]
      current_max: [5.0, 6.0, 7.0]
      motor_temperature_min: [-20.0, -20.0, -20.0]
      motor_temperature_max: [80.0, 80.0, 80.0]
      percentage_battery_charge_min: 20.0
)";

// Renders a flag vector as e.g. "010" for readable failure messages.
std::string bits(const std::vector<bool>& flags)
{
    std::string out;
    for (bool flag : flags) out += flag ? '1' : '0';
    return out;
}

class RobotSafetyTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        config = YAML::Load(kConfig);
        init();
    }

    // Re-run after editing `config` to apply the change.
    void init()
    {
        safety.init(config);
        state.init(config);
        commands.init(config);
        setNominal();
    }

    void enableMechanicalStress(double a, double b)
    {
        config["robot_interface"]["safety"]["mechanical_stress_exp_coefficients"]["a"] = a;
        config["robot_interface"]["safety"]["mechanical_stress_exp_coefficients"]["b"] = b;
        init();
    }

    // A state and command set that is inside every limit.
    void setNominal()
    {
        state.names = {"j0", "j1", "j2"};
        state.position = {0.1, 0.2, 0.3};
        state.velocity = {1.0, 2.0, 3.0};
        state.torque = {10.0, 10.0, 10.0};
        state.motors_current = {1.0, 1.0, 1.0};
        state.motor_temperature = {25.0, 25.0, 25.0};
        state.motors_status_word = {0, 0, 0};
        state.const_motors_status_words.fungo_pressed = kFungoStatusWord;
        for (auto& fault_info : state.motor_fault_info) setFaultInfo(fault_info, kNoFaultInfo);
        for (auto& battery : state.battery) battery.charge_percentage = 80.0;

        commands.position = {0.1, 0.2, 0.3};
        commands.velocity = {1.0, 2.0, 3.0};
        commands.torque = {5.0, 5.0, 5.0};
    }

    static void setFaultInfo(MotorFaultInfo& fault_info, const std::string& text)
    {
        fault_info.fill('\0');
        text.copy(fault_info.data(), fault_info.size() - 1);
    }

    const RobotAnomalies& anomalies() const { return safety.anomalies; }

    // Every per-joint/per-battery flag, by name, for "nothing raised" checks.
    std::vector<std::pair<const char*, const std::vector<bool>*>> allFlagVectors() const
    {
        const auto& a = safety.anomalies;
        return {
            {"joint_pos_limit_violation", &a.joint_pos_limit_violation},
            {"joint_vel_limit_violation", &a.joint_vel_limit_violation},
            {"joint_torque_limit_violation", &a.joint_torque_limit_violation},
            {"joint_current_limit_violation", &a.joint_current_limit_violation},
            {"nan_position_detected", &a.nan_position_detected},
            {"nan_velocity_detected", &a.nan_velocity_detected},
            {"nan_torque_detected", &a.nan_torque_detected},
            {"inf_position_detected", &a.inf_position_detected},
            {"inf_velocity_detected", &a.inf_velocity_detected},
            {"inf_torque_detected", &a.inf_torque_detected},
            {"joint_pos_clamping_anomaly", &a.joint_pos_clamping_anomaly},
            {"joint_vel_clamping_anomaly", &a.joint_vel_clamping_anomaly},
            {"joint_torque_clamping_anomaly", &a.joint_torque_clamping_anomaly},
            {"nan_position_command_detected", &a.nan_position_command_detected},
            {"nan_velocity_command_detected", &a.nan_velocity_command_detected},
            {"nan_torque_command_detected", &a.nan_torque_command_detected},
            {"inf_position_command_detected", &a.inf_position_command_detected},
            {"inf_velocity_command_detected", &a.inf_velocity_command_detected},
            {"inf_torque_command_detected", &a.inf_torque_command_detected},
            {"motor_overheat_detected", &a.motor_overheat_detected},
            {"motor_under_temperature_detected", &a.motor_under_temperature_detected},
            {"motor_fault_detected", &a.motor_fault_detected},
            {"motor_not_operational_detected", &a.motor_not_operational_detected},
            {"battery_charge_low", &a.battery_charge_low},
        };
    }

    void expectNoAnomaly() const
    {
        for (const auto& [name, flags] : allFlagVectors())
            EXPECT_EQ(bits(*flags), std::string(flags->size(), '0')) << name;
        EXPECT_FALSE(safety.isFungoPressed());
        EXPECT_FALSE(safety.isEmergencyStopActive());
        EXPECT_FALSE(anomalies().lost_references.load());
        EXPECT_FALSE(anomalies().nodes_not_discovered);
        EXPECT_FALSE(anomalies().nodes_detached);
    }

    YAML::Node config;
    Safety safety;
    RobotState state;
    RobotCommands commands;
};

// --- Baseline ---------------------------------------------------------------

TEST_F(RobotSafetyTest, NominalStateAndCommandsRaiseNoAnomaly)
{
    safety.checkState(state);
    for (const auto mode : kCheckedModes) {
        safety.checkCommands(commands, mode);
        expectNoAnomaly();
    }
}

TEST_F(RobotSafetyTest, ValuesExactlyOnTheLimitsAreNotAnomalies)
{
    state.position = {1.0, -2.0, 3.0};
    state.velocity = {10.0, -20.0, 30.0};
    state.torque = {40.0, -50.0, 60.0};
    state.motors_current = {5.0, -6.0, 7.0};
    state.motor_temperature = {80.0, -20.0, 80.0};
    for (auto& battery : state.battery) battery.charge_percentage = 20.0;
    commands.position = {1.0, -2.0, 3.0};
    commands.velocity = {10.0, -20.0, 30.0};
    commands.torque = {40.0, -50.0, 60.0};

    safety.checkState(state);
    for (const auto mode : kCheckedModes) {
        safety.checkCommands(commands, mode);
        expectNoAnomaly();
    }
}

// --- Measured-state limits --------------------------------------------------

TEST_F(RobotSafetyTest, joint_pos_limit_violation)
{
    state.position = {1.5, 0.0, -3.5};  // above max, nominal, below min
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().joint_pos_limit_violation), "101");

    setNominal();
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().joint_pos_limit_violation), "000") << "flag must clear";
}

TEST_F(RobotSafetyTest, joint_vel_limit_violation)
{
    // velocity_min is derived as -velocity_max.
    state.velocity = {0.0, 20.5, -30.5};
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().joint_vel_limit_violation), "011");

    setNominal();
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().joint_vel_limit_violation), "000");
}

TEST_F(RobotSafetyTest, joint_torque_limit_violation)
{
    // torque_min is derived as -torque_max.
    state.torque = {-40.5, 0.0, 60.5};
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().joint_torque_limit_violation), "101");

    setNominal();
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().joint_torque_limit_violation), "000");
}

TEST_F(RobotSafetyTest, joint_current_limit_violation)
{
    // current_min is derived as -current_max.
    state.motors_current = {0.0, 6.5, -7.5};
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().joint_current_limit_violation), "011");

    setNominal();
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().joint_current_limit_violation), "000");
}

TEST_F(RobotSafetyTest, motor_overheat_detected)
{
    state.motor_temperature = {25.0, 25.0, 80.5};
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().motor_overheat_detected), "001");
    EXPECT_EQ(bits(anomalies().motor_under_temperature_detected), "000");

    setNominal();
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().motor_overheat_detected), "000");
}

TEST_F(RobotSafetyTest, motor_under_temperature_detected)
{
    state.motor_temperature = {-20.5, 25.0, 25.0};
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().motor_under_temperature_detected), "100");
    EXPECT_EQ(bits(anomalies().motor_overheat_detected), "000");

    setNominal();
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().motor_under_temperature_detected), "000");
}

TEST_F(RobotSafetyTest, battery_charge_low)
{
    state.battery[1].charge_percentage = 19.5;
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().battery_charge_low), "01");

    setNominal();
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().battery_charge_low), "00");
}

// --- Measured-state NaN / Inf -----------------------------------------------

TEST_F(RobotSafetyTest, nan_position_detected)
{
    state.position[0] = kNaN;
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().nan_position_detected), "100");
    EXPECT_EQ(bits(anomalies().inf_position_detected), "000");

    setNominal();
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().nan_position_detected), "000");
}

TEST_F(RobotSafetyTest, nan_velocity_detected)
{
    state.velocity[1] = kNaN;
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().nan_velocity_detected), "010");
    EXPECT_EQ(bits(anomalies().inf_velocity_detected), "000");

    setNominal();
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().nan_velocity_detected), "000");
}

TEST_F(RobotSafetyTest, nan_torque_detected)
{
    state.torque[2] = kNaN;
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().nan_torque_detected), "001");
    EXPECT_EQ(bits(anomalies().inf_torque_detected), "000");

    setNominal();
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().nan_torque_detected), "000");
}

TEST_F(RobotSafetyTest, inf_position_detected)
{
    state.position = {kInf, 0.0, -kInf};
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().inf_position_detected), "101");
    EXPECT_EQ(bits(anomalies().nan_position_detected), "000");

    setNominal();
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().inf_position_detected), "000");
}

TEST_F(RobotSafetyTest, inf_velocity_detected)
{
    state.velocity = {0.0, -kInf, kInf};
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().inf_velocity_detected), "011");
    EXPECT_EQ(bits(anomalies().nan_velocity_detected), "000");

    setNominal();
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().inf_velocity_detected), "000");
}

TEST_F(RobotSafetyTest, inf_torque_detected)
{
    state.torque = {-kInf, kInf, 0.0};
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().inf_torque_detected), "110");
    EXPECT_EQ(bits(anomalies().nan_torque_detected), "000");

    setNominal();
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().inf_torque_detected), "000");
}

// --- Fungo (hardware emergency button) --------------------------------------

TEST_F(RobotSafetyTest, fungo_pressed)
{
    state.motors_status_word[1] = kFungoStatusWord;
    safety.checkState(state);
    EXPECT_TRUE(safety.isFungoPressed());

    // Recomputed every cycle: released as soon as no motor reports it.
    setNominal();
    safety.checkState(state);
    EXPECT_FALSE(safety.isFungoPressed());
}

TEST_F(RobotSafetyTest, fungo_pressed_NotRaisedByOtherStatusWords)
{
    state.motors_status_word = {kFungoStatusWord - 1, kFungoStatusWord + 1, 0};
    safety.checkState(state);
    EXPECT_FALSE(safety.isFungoPressed());
}

// --- Motor fault --------------------------------------------------------------

TEST_F(RobotSafetyTest, motor_fault_detected)
{
    setFaultInfo(state.motor_fault_info[2], "OverCurrent");
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().motor_fault_detected), "001");

    // Recomputed every cycle: cleared as soon as the motor reports no fault again.
    setNominal();
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().motor_fault_detected), "000");
}

TEST_F(RobotSafetyTest, motor_fault_detected_NotRaisedByEmptyFaultInfo)
{
    // Motors that do not report fault_info (e.g. simulation) leave it empty.
    setFaultInfo(state.motor_fault_info[0], "");
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().motor_fault_detected), "000");
}

TEST_F(RobotSafetyTest, motor_fault_detected_DisabledWithoutNoFaultInfo)
{
    config["robot_interface"]["safety"].remove("motor_no_fault_info");
    init();
    setFaultInfo(state.motor_fault_info[1], "OverCurrent");
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().motor_fault_detected), "000");
}

TEST_F(RobotSafetyTest, motor_fault_detected_RejectsTooLongNoFaultInfo)
{
    config["robot_interface"]["safety"]["motor_no_fault_info"] = std::string(64, 'x');
    EXPECT_THROW(safety.init(config), std::invalid_argument);
}

// --- Motor not operational ---------------------------------------------------

TEST_F(RobotSafetyTest, motor_not_operational_detected)
{
    state.const_motors_status_words.operational_mask = 0x4;
    state.motors_status_word = {0x4, 0x3, 0x7};
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().motor_not_operational_detected), "010");
    EXPECT_TRUE(safety.isAnyMotorNotOperational());

    // Recomputed every cycle: cleared as soon as the motor reports operation enabled again.
    state.motors_status_word = {0x4, 0x4, 0x4};
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().motor_not_operational_detected), "000");
    EXPECT_FALSE(safety.isAnyMotorNotOperational());
}

TEST_F(RobotSafetyTest, motor_not_operational_detected_DisabledWithoutMask)
{
    state.motors_status_word = {0, 0, 0};
    safety.checkState(state);
    EXPECT_EQ(bits(anomalies().motor_not_operational_detected), "000");
}

TEST_F(RobotSafetyTest, motor_not_operational_detected_RejectsNegativeNotifyDuration)
{
    config["robot_interface"]["safety"]["startup_failure_notify_duration"] = -1.0;
    EXPECT_THROW(safety.init(config), std::invalid_argument);
}

// --- Nodes not discovered ----------------------------------------------------

TEST_F(RobotSafetyTest, nodes_not_discovered)
{
    safety.setDiscoveredNodes({1, 2}, false);
    EXPECT_TRUE(anomalies().nodes_not_discovered);
    EXPECT_TRUE(safety.isAnyNodeNotDiscovered());
    EXPECT_EQ(safety.getDiscoveredNodes(), (std::vector<int>{1, 2}));

    // Not recomputed by checkState(): kept until the next setDiscoveredNodes() or init().
    safety.checkState(state);
    EXPECT_TRUE(anomalies().nodes_not_discovered);

    safety.setDiscoveredNodes({1, 2, 3}, true);
    EXPECT_FALSE(anomalies().nodes_not_discovered);
    EXPECT_FALSE(safety.isAnyNodeNotDiscovered());

    safety.setDiscoveredNodes({1, 2}, false);
    safety.init(config);
    EXPECT_FALSE(anomalies().nodes_not_discovered);
    EXPECT_TRUE(safety.getDiscoveredNodes().empty());
}

// --- Command clamping -------------------------------------------------------

TEST_F(RobotSafetyTest, joint_pos_clamping_anomaly)
{
    commands.position = {-1.5, 0.0, 3.5};  // below min, nominal, above max
    safety.checkCommands(commands, CONTROL_MODE::POSITION_CONTROL);

    EXPECT_EQ(bits(anomalies().joint_pos_clamping_anomaly), "101");
    EXPECT_DOUBLE_EQ(commands.position[0], -1.0) << "clamped to min";
    EXPECT_DOUBLE_EQ(commands.position[1], 0.0) << "untouched";
    EXPECT_DOUBLE_EQ(commands.position[2], 3.0) << "clamped to max";
    EXPECT_DOUBLE_EQ(anomalies().joint_pos_command_trigger_value[0], -1.5);
    EXPECT_DOUBLE_EQ(anomalies().joint_pos_command_trigger_value[2], 3.5);
    EXPECT_FALSE(safety.isEmergencyStopActive()) << "clamping alone is not an e-stop";

    setNominal();
    safety.checkCommands(commands, CONTROL_MODE::POSITION_CONTROL);
    EXPECT_EQ(bits(anomalies().joint_pos_clamping_anomaly), "000");
}

TEST_F(RobotSafetyTest, joint_vel_clamping_anomaly)
{
    commands.velocity = {0.0, -25.0, 35.0};
    safety.checkCommands(commands, CONTROL_MODE::VELOCITY_CONTROL);

    EXPECT_EQ(bits(anomalies().joint_vel_clamping_anomaly), "011");
    EXPECT_DOUBLE_EQ(commands.velocity[1], -20.0);
    EXPECT_DOUBLE_EQ(commands.velocity[2], 30.0);
    EXPECT_DOUBLE_EQ(anomalies().joint_vel_command_trigger_value[1], -25.0);
    EXPECT_DOUBLE_EQ(anomalies().joint_vel_command_trigger_value[2], 35.0);
    EXPECT_FALSE(safety.isEmergencyStopActive());

    setNominal();
    safety.checkCommands(commands, CONTROL_MODE::VELOCITY_CONTROL);
    EXPECT_EQ(bits(anomalies().joint_vel_clamping_anomaly), "000");
}

TEST_F(RobotSafetyTest, joint_torque_clamping_anomaly)
{
    commands.torque = {100.0, 0.0, -70.0};
    safety.checkCommands(commands, CONTROL_MODE::IMPEDANCE_CONTROL);

    EXPECT_EQ(bits(anomalies().joint_torque_clamping_anomaly), "101");
    EXPECT_DOUBLE_EQ(commands.torque[0], 40.0);
    EXPECT_DOUBLE_EQ(commands.torque[2], -60.0);
    EXPECT_DOUBLE_EQ(anomalies().joint_torque_command_trigger_value[0], 100.0);
    EXPECT_DOUBLE_EQ(anomalies().joint_torque_command_trigger_value[2], -70.0);
    EXPECT_FALSE(safety.isEmergencyStopActive());

    setNominal();
    safety.checkCommands(commands, CONTROL_MODE::IMPEDANCE_CONTROL);
    EXPECT_EQ(bits(anomalies().joint_torque_clamping_anomaly), "000");
}

TEST_F(RobotSafetyTest, OnlyTheCommandOfTheControlModeIsChecked)
{
    // Impedance mode sends torques only: position and velocity commands are
    // not computed, so they are not checked (and not clamped).
    commands.position = {-1.5, kNaN, 3.5};
    commands.velocity = {kInf, -25.0, 0.0};
    commands.torque = {100.0, 0.0, 0.0};
    safety.checkCommands(commands, CONTROL_MODE::IMPEDANCE_CONTROL);

    EXPECT_EQ(bits(anomalies().joint_torque_clamping_anomaly), "100");
    EXPECT_EQ(bits(anomalies().joint_pos_clamping_anomaly), "000");
    EXPECT_EQ(bits(anomalies().joint_vel_clamping_anomaly), "000");
    EXPECT_EQ(bits(anomalies().nan_position_command_detected), "000");
    EXPECT_EQ(bits(anomalies().inf_velocity_command_detected), "000");
    EXPECT_FALSE(safety.isEmergencyStopActive());
    EXPECT_DOUBLE_EQ(commands.position[0], -1.5) << "unchecked commands are left as they are";
}

TEST_F(RobotSafetyTest, FlagsOfUncheckedCommandsAreCleared)
{
    commands.position = {-1.5, 0.0, 0.0};
    safety.checkCommands(commands, CONTROL_MODE::POSITION_CONTROL);
    ASSERT_EQ(bits(anomalies().joint_pos_clamping_anomaly), "100");

    commands.position = {-1.5, 0.0, 0.0};
    safety.checkCommands(commands, CONTROL_MODE::IMPEDANCE_CONTROL);
    EXPECT_EQ(bits(anomalies().joint_pos_clamping_anomaly), "000");
}

TEST_F(RobotSafetyTest, UnknownControlModeChecksEveryCommand)
{
    commands.position = {-1.5, 0.0, 0.0};
    commands.velocity = {0.0, -25.0, 0.0};
    commands.torque = {0.0, 0.0, -70.0};
    safety.checkCommands(commands, CONTROL_MODE::CURRENT_CONTROL);
    EXPECT_EQ(bits(anomalies().joint_pos_clamping_anomaly), "100");
    EXPECT_EQ(bits(anomalies().joint_vel_clamping_anomaly), "010");
    EXPECT_EQ(bits(anomalies().joint_torque_clamping_anomaly), "001");
}

// Regression: in impedance mode the recovery modes zero the (unused) position
// commands, and a joint whose range excludes 0, like a knee, was reported as
// clamped on every cycle of EMERGENCY_STOP/DAMPING.
TEST_F(RobotSafetyTest, RecoveryZeroesRaiseNoClampingInImpedanceMode)
{
    // Knee-like range for j2, excludes 0
    config["robot_interface"]["safety"]["limits"]["position_min_with_margin"][2] = -3.0;
    config["robot_interface"]["safety"]["limits"]["position_max_with_margin"][2] = -1.0;
    init();
    safety.triggerEmergencyStop();
    for (int cycle = 0; cycle < 3; ++cycle) {
        safety.manageAnomalies(state, commands, CONTROL_MODE::IMPEDANCE_CONTROL);
        EXPECT_EQ(bits(anomalies().joint_pos_clamping_anomaly), "000") << "cycle " << cycle;
    }
    EXPECT_EQ(safety.getSafetyState(), SafetyState::EMERGENCY_STOP);
}

// --- Command NaN / Inf (each one latches the emergency stop) ----------------

class InvalidCommandTest : public RobotSafetyTest
{
protected:
    void expectInvalidCommand(std::vector<double> RobotCommands::*command,
                              std::vector<bool> RobotAnomalies::*flag,
                              size_t joint, double value)
    {
        const CONTROL_MODE mode = command == &RobotCommands::position ? CONTROL_MODE::POSITION_CONTROL
                                : command == &RobotCommands::velocity ? CONTROL_MODE::VELOCITY_CONTROL
                                : CONTROL_MODE::IMPEDANCE_CONTROL;
        (commands.*command)[joint] = value;
        safety.checkCommands(commands, mode);

        std::string expected(3, '0');
        expected[joint] = '1';
        EXPECT_EQ(bits(safety.anomalies.*flag), expected);
        EXPECT_TRUE(safety.isEmergencyStopActive());
        // Invalid values are not clamping anomalies.
        EXPECT_EQ(bits(anomalies().joint_pos_clamping_anomaly), "000");
        EXPECT_EQ(bits(anomalies().joint_vel_clamping_anomaly), "000");
        EXPECT_EQ(bits(anomalies().joint_torque_clamping_anomaly), "000");

        // The flag clears with a valid command; the e-stop stays latched.
        setNominal();
        safety.checkCommands(commands, mode);
        EXPECT_EQ(bits(safety.anomalies.*flag), "000");
        EXPECT_TRUE(safety.isEmergencyStopActive());
    }
};

TEST_F(InvalidCommandTest, nan_position_command_detected)
{
    expectInvalidCommand(&RobotCommands::position, &RobotAnomalies::nan_position_command_detected, 0, kNaN);
}

TEST_F(InvalidCommandTest, nan_velocity_command_detected)
{
    expectInvalidCommand(&RobotCommands::velocity, &RobotAnomalies::nan_velocity_command_detected, 1, kNaN);
}

TEST_F(InvalidCommandTest, nan_torque_command_detected)
{
    expectInvalidCommand(&RobotCommands::torque, &RobotAnomalies::nan_torque_command_detected, 2, kNaN);
}

TEST_F(InvalidCommandTest, inf_position_command_detected)
{
    expectInvalidCommand(&RobotCommands::position, &RobotAnomalies::inf_position_command_detected, 2, kInf);
}

TEST_F(InvalidCommandTest, inf_velocity_command_detected)
{
    expectInvalidCommand(&RobotCommands::velocity, &RobotAnomalies::inf_velocity_command_detected, 0, -kInf);
}

TEST_F(InvalidCommandTest, inf_torque_command_detected)
{
    expectInvalidCommand(&RobotCommands::torque, &RobotAnomalies::inf_torque_command_detected, 1, kInf);
}

// --- Emergency stop and lost references -------------------------------------

TEST_F(RobotSafetyTest, emergency_stop_triggered)
{
    EXPECT_FALSE(safety.isEmergencyStopActive());
    safety.triggerEmergencyStop();
    EXPECT_TRUE(safety.isEmergencyStopActive());

    // One-way latch: nominal cycles do not release it.
    safety.checkState(state);
    safety.checkCommands(commands, kMode);
    EXPECT_TRUE(safety.isEmergencyStopActive());
}

TEST_F(RobotSafetyTest, lost_references)
{
    safety.setLostReferences(true);
    EXPECT_TRUE(anomalies().lost_references.load());
    // Latched: false is ignored, only init() clears it.
    safety.setLostReferences(false);
    EXPECT_TRUE(anomalies().lost_references.load());
    init();
    EXPECT_FALSE(anomalies().lost_references.load());
}

TEST_F(RobotSafetyTest, nodes_detached)
{
    safety.setNodesDetached(true);
    EXPECT_TRUE(anomalies().nodes_detached);
    // Latched: false is ignored, only init() clears it.
    safety.setNodesDetached(false);
    EXPECT_TRUE(anomalies().nodes_detached);
    init();
    EXPECT_FALSE(anomalies().nodes_detached);
}

// --- Safety state machine and recovery (manageAnomalies) --------------------

TEST_F(RobotSafetyTest, SafetyState_NominalLeavesCommandsUntouched)
{
    const auto expected = commands.torque;
    safety.manageAnomalies(state, commands, kMode);
    EXPECT_EQ(safety.getSafetyState(), SafetyState::NOMINAL);
    EXPECT_EQ(commands.torque, expected);
}

TEST_F(RobotSafetyTest, SafetyState_FungoEntersEmergencyStopAndZeroesCommands)
{
    state.motors_status_word[0] = kFungoStatusWord;
    safety.manageAnomalies(state, commands, kMode);
    EXPECT_EQ(safety.getSafetyState(), SafetyState::EMERGENCY_STOP);
    EXPECT_EQ(commands.position, std::vector<double>(3, 0.0));
    EXPECT_EQ(commands.velocity, std::vector<double>(3, 0.0));
    EXPECT_EQ(commands.torque, std::vector<double>(3, 0.0));
}

TEST_F(RobotSafetyTest, SafetyState_InvalidCommandEntersEmergencyStopAndZeroesCommands)
{
    commands.torque[1] = kNaN;
    safety.manageAnomalies(state, commands, kMode);
    EXPECT_EQ(safety.getSafetyState(), SafetyState::EMERGENCY_STOP);
    EXPECT_EQ(commands.torque, std::vector<double>(3, 0.0)) << "NaN must not reach the hardware";
}

TEST_F(RobotSafetyTest, SafetyState_NodesDetachedEntersEmergencyStop)
{
    safety.setNodesDetached(true);
    safety.manageAnomalies(state, commands, kMode);
    EXPECT_EQ(safety.getSafetyState(), SafetyState::EMERGENCY_STOP);
    // Zero torque: the frozen measured velocities must not drive a damping torque.
    EXPECT_EQ(commands.torque, std::vector<double>(3, 0.0));
}

TEST_F(RobotSafetyTest, SafetyState_LostReferencesEntersDamping)
{
    safety.setLostReferences(true);
    safety.manageAnomalies(state, commands, kMode);
    EXPECT_EQ(safety.getSafetyState(), SafetyState::DAMPING);
    // Default damping gain is 1: torque = -velocity, position/velocity zeroed.
    EXPECT_EQ(commands.torque, (std::vector<double>{-1.0, -2.0, -3.0}));
    EXPECT_EQ(commands.position, std::vector<double>(3, 0.0));
    EXPECT_EQ(commands.velocity, std::vector<double>(3, 0.0));
}

TEST_F(RobotSafetyTest, SafetyState_EmergencyStopHasPriorityOverDamping)
{
    safety.setLostReferences(true);
    safety.triggerEmergencyStop();
    safety.manageAnomalies(state, commands, kMode);
    EXPECT_EQ(safety.getSafetyState(), SafetyState::EMERGENCY_STOP);
}

TEST_F(RobotSafetyTest, SafetyState_DampingEscalatesToEmergencyStop)
{
    safety.setLostReferences(true);
    safety.manageAnomalies(state, commands, kMode);
    ASSERT_EQ(safety.getSafetyState(), SafetyState::DAMPING);

    safety.triggerEmergencyStop();
    safety.manageAnomalies(state, commands, kMode);
    EXPECT_EQ(safety.getSafetyState(), SafetyState::EMERGENCY_STOP);
}

// Pins current behavior: setSafetyState() only escalates, so the robot stays
// in DAMPING after the references come back. Change this test if a recovery
// path back to NOMINAL is added.
TEST_F(RobotSafetyTest, SafetyState_IsLatchedAfterAnomalyClears)
{
    safety.setLostReferences(true);
    safety.manageAnomalies(state, commands, kMode);
    ASSERT_EQ(safety.getSafetyState(), SafetyState::DAMPING);

    safety.setLostReferences(false);
    setNominal();
    safety.manageAnomalies(state, commands, kMode);
    EXPECT_EQ(safety.getSafetyState(), SafetyState::DAMPING);
}

}  // namespace
