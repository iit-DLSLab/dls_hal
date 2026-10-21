#include "robot_invariants.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

#define ROBOT_SAFETY_TEMPLATE template <typename TState, typename TRef, typename TCommands, typename TLimits>
#define ROBOT_SAFETY_TYPE RobotSafety<TState, TRef, TCommands, TLimits>

#define ROBOT_LLC_TEMPLATE template <typename TState, typename TRef, typename TCommands>
#define ROBOT_LLC_TYPE RobotLowLevelControl<TState, TRef, TCommands>

// RobotSafety
ROBOT_SAFETY_TEMPLATE
ROBOT_SAFETY_TYPE::RobotSafety(){
    static_assert(std::is_base_of<RobotState, TState>::value, "TState must be derived from RobotState");
    static_assert(std::is_base_of<RobotRef, TRef>::value, "TRef must be derived from RobotRef");
    static_assert(std::is_base_of<RobotCommands, TCommands>::value, "TCommands must be derived from RobotCommands");
    static_assert(std::is_base_of<RobotLimits, TLimits>::value, "TLimits must be derived from RobotLimits");
    anomalies.emergency_stop_triggered.store(false, std::memory_order_relaxed);
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::init(const YAML::Node& config){
    anomalies.emergency_stop_triggered.store(false, std::memory_order_relaxed);
    const auto references_timeout_node = config["robot_interface"]["references_timeout"];
    references_timeout = references_timeout_node ? references_timeout_node.as<double>() : 1.0;
    if (!std::isfinite(references_timeout) || references_timeout <= 0.0)
        throw std::invalid_argument("robot_interface.references_timeout must be finite and positive (seconds)");
    const auto events_frequency_node = config["robot_interface"]["events_frequency"];
    events_frequency = events_frequency_node ? events_frequency_node.as<double>() : 10.0;
    if (!std::isfinite(events_frequency) || events_frequency <= 0.0)
        throw std::invalid_argument("robot_interface.events_frequency must be finite and strictly positive (Hz)");
    const auto control_period_node = config["robot_interface"]["safety"]["control_period"];
    control_period = control_period_node ? control_period_node.as<double>() : 0.0;
    if (!std::isfinite(control_period) || control_period < 0.0)
        control_period = 0.0;
    const auto min_velocity_node = config["robot_interface"]["safety"]["virtual_joint_limit_min_velocity"];
    virtual_joint_limit_min_velocity = min_velocity_node ? min_velocity_node.as<double>() : 0.3;
    if (!std::isfinite(virtual_joint_limit_min_velocity) || virtual_joint_limit_min_velocity < 0.0)
        throw std::invalid_argument("robot_interface.safety.virtual_joint_limit_min_velocity must be finite and non-negative (rad/s)");
    const auto torque_scale_node = config["robot_interface"]["safety"]["virtual_joint_limit_torque_scale"];
    virtual_joint_limit_torque_scale = torque_scale_node ? torque_scale_node.as<double>() : 1.0;
    if (!std::isfinite(virtual_joint_limit_torque_scale) || virtual_joint_limit_torque_scale <= 0.0
        || virtual_joint_limit_torque_scale > 1.0)
        throw std::invalid_argument("robot_interface.safety.virtual_joint_limit_torque_scale must be in (0, 1]");
    const auto n_joint = config["general"]["n_joints"].as<int>();
    anomalies.joint_pos_limit_violation.resize(n_joint, false);
    anomalies.joint_pos_limit_violation_predicted.resize(n_joint, false);
    anomalies.joint_vel_limit_violation.resize(n_joint, false);
    anomalies.joint_current_limit_violation.resize(n_joint, false);
    anomalies.joint_torque_limit_violation.resize(n_joint, false);
    anomalies.nan_position_detected.resize(n_joint, false);
    anomalies.nan_velocity_detected.resize(n_joint, false);
    anomalies.nan_torque_detected.resize(n_joint, false);
    anomalies.inf_position_detected.resize(n_joint, false);
    anomalies.inf_velocity_detected.resize(n_joint, false);
    anomalies.inf_torque_detected.resize(n_joint, false);
    anomalies.joint_pos_clamping_anomaly.assign(n_joint, false);
    anomalies.joint_vel_clamping_anomaly.assign(n_joint, false);
    anomalies.joint_torque_clamping_anomaly.assign(n_joint, false);
    anomalies.joint_pos_command_trigger_value.assign(n_joint, 0.0);
    anomalies.joint_vel_command_trigger_value.assign(n_joint, 0.0);
    anomalies.joint_torque_command_trigger_value.assign(n_joint, 0.0);
    anomalies.nan_position_command_detected.assign(n_joint, false);
    anomalies.nan_velocity_command_detected.assign(n_joint, false);
    anomalies.nan_torque_command_detected.assign(n_joint, false);
    anomalies.inf_position_command_detected.assign(n_joint, false);
    anomalies.inf_velocity_command_detected.assign(n_joint, false);
    anomalies.inf_torque_command_detected.assign(n_joint, false);
    anomalies.motor_under_temperature_detected.resize(n_joint, false);
    anomalies.motor_overheat_detected.resize(n_joint, false);
    anomalies.battery_charge_low.resize(config["general"]["n_batteries"].as<int>(), false);
    anomalies.fungo_pressed.store(false, std::memory_order_relaxed);

    anomalies.lost_references.store(false, std::memory_order_relaxed);
    limits.init(config);
    
    // Recovery modes keep their own copy: limits are fixed after init().
    const RecoveryLimits recovery_limits{
        limits.position_min, limits.position_max,
        limits.velocity_min, limits.velocity_max,
        limits.torque_min, limits.torque_max};
    recovery_estop.setLimits(recovery_limits);
    recovery_damping.setLimits(recovery_limits);
    // Damping recovery gain [Nm * s / rad]: a scalar for all joints or one value per joint; default 1.
    const auto damping_gain_node = config["robot_interface"]["safety"]["recovery_damping_gain"];
    std::vector<double> damping_gain(n_joint, 1.0);
    if (damping_gain_node) {
        if (damping_gain_node.IsScalar()) {
            damping_gain.assign(n_joint, damping_gain_node.as<double>());
        } else {
            damping_gain = damping_gain_node.as<std::vector<double>>();
            if (damping_gain.size() != static_cast<size_t>(n_joint))
                throw std::invalid_argument("robot_interface.safety.recovery_damping_gain must be a scalar or have n_joints elements");
        }
    }
    recovery_damping.setDampingGains(damping_gain);
    recovery_virtual_joint_limit.setLimits(recovery_limits);
    const auto release_tolerance_node = config["robot_interface"]["safety"]["virtual_joint_limit_release_tolerance"];
    double release_tolerance = 1e-3;
    if (release_tolerance_node){
        release_tolerance = release_tolerance_node.as<double>();
    }
    const auto damping_after_stop_node = config["robot_interface"]["safety"]["virtual_joint_limit_damping_after_stop"];
    const bool damping_after_stop = damping_after_stop_node ? damping_after_stop_node.as<bool>() : false;
    // Brake stiffness, set either as the damping ratio zeta in [0.5, 1] or as the stiffness scale
    // k / k(zeta = 1) = 1 / zeta^2 in [1, 4]; default 1 (critically damped).
    const auto damping_ratio_node = config["robot_interface"]["safety"]["virtual_joint_limit_damping_ratio"];
    const auto stiffness_scale_node = config["robot_interface"]["safety"]["virtual_joint_limit_stiffness_scale"];
    if (damping_ratio_node && stiffness_scale_node)
        throw std::invalid_argument("robot_interface.safety: set either virtual_joint_limit_damping_ratio or "
                                    "virtual_joint_limit_stiffness_scale, not both");
    double damping_ratio = 1.0;
    if (damping_ratio_node) {
        damping_ratio = damping_ratio_node.as<double>();
        if (!std::isfinite(damping_ratio) || damping_ratio < 0.5 || damping_ratio > 1.0)
            throw std::invalid_argument("robot_interface.safety.virtual_joint_limit_damping_ratio must be in [0.5, 1]");
    } else if (stiffness_scale_node) {
        const double stiffness_scale = stiffness_scale_node.as<double>();
        if (!std::isfinite(stiffness_scale) || stiffness_scale < 1.0 || stiffness_scale > 4.0)
            throw std::invalid_argument("robot_interface.safety.virtual_joint_limit_stiffness_scale must be in [1, 4]");
        damping_ratio = 1.0 / std::sqrt(stiffness_scale);
    }
    recovery_virtual_joint_limit.init(n_joint, release_tolerance, damping_after_stop, damping_ratio);
    // Damping of all the joints (gain recovery_damping_gain) once a joint triggers the virtual
    // joint limit, latched until the HAL restarts; the excluded joints (e.g. the knees) are
    // braked as usual but do not start it.
    const auto damp_all_node = config["robot_interface"]["safety"]["virtual_joint_limit_damp_all_joints"];
    const bool damp_all = damp_all_node ? damp_all_node.as<bool>() : false;
    std::vector<bool> triggers_damp_all(n_joint, true);
    const auto damp_all_excluded_node = config["robot_interface"]["safety"]["virtual_joint_limit_damp_all_excluded_joints"];
    if (damp_all_excluded_node) {
        for (const int joint : damp_all_excluded_node.as<std::vector<int>>()) {
            if (joint < 0 || joint >= n_joint)
                throw std::invalid_argument("robot_interface.safety.virtual_joint_limit_damp_all_excluded_joints must contain joint indices in [0, n_joints)");
            triggers_damp_all[joint] = false;
        }
    }
    recovery_virtual_joint_limit.setDampAll(damp_all, triggers_damp_all, damping_gain);
    virtual_joint_limit_upper_only.assign(n_joint, false);
    const auto upper_only_node = config["robot_interface"]["safety"]["virtual_joint_limit_upper_only_joints"];
    if (upper_only_node) {
        for (const int joint : upper_only_node.as<std::vector<int>>()) {
            if (joint < 0 || joint >= n_joint)
                throw std::invalid_argument("robot_interface.safety.virtual_joint_limit_upper_only_joints must contain joint indices in [0, n_joints)");
            virtual_joint_limit_upper_only[joint] = true;
        }
    }

    safety_state = SafetyState::NOMINAL;
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::computePredictedPositionThresholds(size_t joint_index, 
                                                           const double velocity, 
                                                           const double torque,
                                                           double& threshold_min, 
                                                           double& threshold_max) const {
    const size_t i = joint_index;
    // Considering controller torque moving towards the limit.
    // Ignore controller braking contribution (torque driving away from limit) for robustness.
    // Only a fraction of the torque limit is budgeted for braking: the rest is headroom
    // for model errors (inertia, velocity prediction, external torque).
    constexpr double tau_net_eps = 0.1; //1e-6; // Denominator regulation term
    const double tau_net_max = std::max(
        virtual_joint_limit_torque_scale * std::abs(limits.torque_min[i]) - std::max( torque, 0.0), tau_net_eps);
    const double tau_net_min = std::max(
        virtual_joint_limit_torque_scale * std::abs(limits.torque_max[i]) - std::max(-torque, 0.0), tau_net_eps);
    // Stopping distance of the virtual joint limit whose peak torque (at the trigger) equals
    // the available torque: c * I * v^2 / tau_net, with c = 2 / e when critically damped
    // (4/e times the full-torque braking distance), smaller for a stiffer, underdamped brake.
    // Plus the distance covered during the up to one control period between the threshold
    // crossing and its detection.
    const double c = recovery_virtual_joint_limit.getStoppingDistanceCoefficient();
    const double delta_q_max =
        velocity > 0.0
        ? c * limits.joint_inertia[i] * velocity * velocity / tau_net_max + velocity * control_period
        : 0.0;
    const double delta_q_min =
        velocity < 0.0
        ? c * limits.joint_inertia[i] * velocity * velocity / tau_net_min - velocity * control_period
        : 0.0;
    threshold_max = limits.position_max[i] - delta_q_max;
    threshold_min = limits.position_min[i] + delta_q_min;
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::checkState(const TState& state) {
    setFungoPressed(false);
    // Check if state is within limits, and setting anomalies if necessary
    for (size_t i = 0; i < state.position.size(); ++i) {
        double joint_position_threshold_min;
        double joint_position_threshold_max;
        // Velocity one control period ahead: until the trigger is detected, the torque applied
        // during the cycle (last measured) still accelerates the joint. Zero inertia disables
        // the prediction, so it also skips this step.
        const double velocity_next = limits.joint_inertia[i] > 0.0
            ? state.velocity[i] + state.torque[i] / limits.joint_inertia[i] * control_period
            : state.velocity[i];
        // Zero-torque passed instead of state.torque[i], recovery overriding the command
        computePredictedPositionThresholds(i, velocity_next, 0.0,
                                           joint_position_threshold_min, joint_position_threshold_max);
        // Upper-only joints: no threshold toward position_min, so neither the prediction nor
        // the activation (side chosen from the crossed threshold) can pick that side.
        if (virtual_joint_limit_upper_only[i])
            joint_position_threshold_min = -std::numeric_limits<double>::infinity();

        anomalies.joint_pos_limit_violation[i] = checkJointStateLimit(state.position[i], limits.position_min[i], limits.position_max[i]);
        // Slow joints do not activate the virtual joint limit, even past the position limits.
        const bool is_predicted = std::abs(state.velocity[i]) >= virtual_joint_limit_min_velocity
            && checkJointStateLimit(state.position[i], joint_position_threshold_min, joint_position_threshold_max);
        anomalies.joint_pos_limit_violation_predicted[i] = is_predicted;
        // Per joint: activated when the violation is predicted, then kept active, whatever
        // the prediction does, until the spring has brought the joint back to its anchor.
        if (!recovery_virtual_joint_limit.isActive(i)) {
            if (is_predicted) {
                const bool was_damp_all_latched = recovery_virtual_joint_limit.isDampAllLatched();
                // Zero-torque passed instead of state.torque[i], recovery overriding the command
                recovery_virtual_joint_limit
                    .setParams(i, limits.joint_inertia[i], state.position[i], state.velocity[i],
                               0.0, joint_position_threshold_min, joint_position_threshold_max);
                if (!was_damp_all_latched && recovery_virtual_joint_limit.isDampAllLatched())
                    std::cout << "[safety] virtual joint limit on joint " << i
                              << ": damping all joints until the HAL restarts" << std::endl;
            }
        } else {
            // With damping_after_stop, a joint moving away from the limit is damped from then on
            // and never released; otherwise it is released once back at its anchor.
            recovery_virtual_joint_limit.updateDampingOnly(i, state.velocity[i]);
            if (!recovery_virtual_joint_limit.isDampingOnly(i)
                && recovery_virtual_joint_limit.hasReturnedToAnchor(i, state.position[i], state.velocity[i])) {
                recovery_virtual_joint_limit.reset(i);
            }
        }
        anomalies.joint_vel_limit_violation[i] = checkJointStateLimit(state.velocity[i], limits.velocity_min[i], limits.velocity_max[i]);
        anomalies.joint_torque_limit_violation[i] = checkJointStateLimit(state.torque[i], limits.torque_min[i], limits.torque_max[i]);
        anomalies.joint_current_limit_violation[i] = checkJointStateLimit(state.motors_current[i], limits.current_min[i], limits.current_max[i]);
        anomalies.motor_overheat_detected[i] = state.motor_temperature[i] > limits.motor_temperature_max[i];
        anomalies.motor_under_temperature_detected[i] = state.motor_temperature[i] < limits.motor_temperature_min[i];
        anomalies.nan_position_detected[i] = std::isnan(state.position[i]);
        anomalies.inf_position_detected[i] = std::isinf(state.position[i]);
        anomalies.nan_velocity_detected[i] = std::isnan(state.velocity[i]);
        anomalies.inf_velocity_detected[i] = std::isinf(state.velocity[i]);
        anomalies.nan_torque_detected[i] = std::isnan(state.torque[i]);
        anomalies.inf_torque_detected[i] = std::isinf(state.torque[i]);
        if (state.motors_status_word[i] == state.const_motors_status_words.fungo_pressed) {
            setFungoPressed(true);
        }
    }
    for (size_t i = 0; i < state.battery.size(); ++i) {
        anomalies.battery_charge_low[i] = state.battery[i].charge_percentage < limits.percentage_battery_charge_min;
    }
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::setFungoPressed(bool pressed) {
    anomalies.fungo_pressed.store(pressed, std::memory_order_relaxed);
}

ROBOT_SAFETY_TEMPLATE
bool ROBOT_SAFETY_TYPE::checkJointStateLimit(double value, double min_limit, double max_limit) {
    if (value < min_limit || value > max_limit) {
        return true;
    } else {
        return false;
    }
}

ROBOT_SAFETY_TEMPLATE
bool ROBOT_SAFETY_TYPE::checkJointStateNanInf(double value) {
    if (std::isnan(value) || std::isinf(value)) {
        return true;
    } else {
        return false;
    }
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::checkCommand(std::vector<double>& command , std::vector<double> min_limit, std::vector<double> max_limit, std::vector<bool>& anomaly_nan, std::vector<bool>& anomaly_inf, std::vector<bool>& anomaly_limit_min, std::vector<bool>& anomaly_limit_max, std::vector<double>& trigger_value) {
    bool is_valid = true;
    for (size_t i = 0; i < command.size(); ++i) {
        // init anomalies to false
        anomaly_nan[i] = false;
        anomaly_inf[i] = false;
        anomaly_limit_min[i] = false;
        anomaly_limit_max[i] = false;
        if (std::isnan(command[i])) {
            anomaly_nan[i] = true;
            is_valid = false;
        }
        else if (std::isinf(command[i])) {
            anomaly_inf[i] = true;
            is_valid = false;
        }
        else if (command[i] < min_limit[i]) {
            anomaly_limit_min[i] = true;
            // Record the pre-clamp command that triggered this: it, not the
            // post-clamp value, is what the caller actually asked for.
            trigger_value[i] = command[i];
            command[i] = std::clamp(command[i], min_limit[i], max_limit[i]);
        }
        else if (command[i] > max_limit[i]) {
            anomaly_limit_max[i] = true;
            trigger_value[i] = command[i];
            command[i] = std::clamp(command[i], min_limit[i], max_limit[i]);
        }
    }
    if (!is_valid) {
        triggerEmergencyStop();
    }
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::checkCommands(TCommands& commands, CONTROL_MODE control_mode) {
    // The low-level interfaces send only the command of the control mode. The
    // other commands are not computed and keep their init or recovery values
    // (zero), which would raise false anomalies, e.g. a zero position command
    // is outside the knee range. Unknown modes check everything.
    bool check_position = false, check_velocity = false, check_torque = false;
    switch (control_mode) {
        case CONTROL_MODE::POSITION_CONTROL: check_position = true; break;
        case CONTROL_MODE::VELOCITY_CONTROL: check_velocity = true; break;
        case CONTROL_MODE::TORQUE_CONTROL:
        case CONTROL_MODE::IMPEDANCE_CONTROL: check_torque = true; break;
        default: check_position = check_velocity = check_torque = true; break;
    }
    const auto clear = [](std::vector<bool>& nan, std::vector<bool>& inf, std::vector<bool>& clamping) {
        std::fill(nan.begin(), nan.end(), false);
        std::fill(inf.begin(), inf.end(), false);
        std::fill(clamping.begin(), clamping.end(), false);
    };
    if (check_position)
        checkCommand(commands.position, limits.position_min, limits.position_max, anomalies.nan_position_command_detected, anomalies.inf_position_command_detected, anomalies.joint_pos_clamping_anomaly, anomalies.joint_pos_clamping_anomaly, anomalies.joint_pos_command_trigger_value);
     if (check_velocity)
        checkCommand(commands.velocity, limits.velocity_min, limits.velocity_max, anomalies.nan_velocity_command_detected, anomalies.inf_velocity_command_detected, anomalies.joint_vel_clamping_anomaly, anomalies.joint_vel_clamping_anomaly, anomalies.joint_vel_command_trigger_value);
   if (check_torque)
        checkCommand(commands.torque, limits.torque_min, limits.torque_max, anomalies.nan_torque_command_detected, anomalies.inf_torque_command_detected, anomalies.joint_torque_clamping_anomaly, anomalies.joint_torque_clamping_anomaly, anomalies.joint_torque_command_trigger_value);
}


ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::displayAnomalies() const {
    for (size_t i = 0; i < anomalies.joint_pos_limit_violation.size(); ++i) {
        if (anomalies.joint_pos_limit_violation[i])
            std::cout << "Joint " << i << " position limit violation detected." << std::endl;
        if(anomalies.joint_pos_limit_violation_predicted[i])
            std::cout << "Joint " << i << " possible position limit violation predicted." << std::endl;
        if (anomalies.joint_vel_limit_violation[i])
            std::cout << "Joint " << i << " velocity limit violation detected." << std::endl;
        if (anomalies.joint_torque_limit_violation[i])
            std::cout << "Joint " << i << " torque limit violation detected." << std::endl;
        if (anomalies.nan_position_detected[i])
            std::cout << "Joint " << i << " NaN position detected." << std::endl;
        if (anomalies.inf_position_detected[i])
            std::cout << "Joint " << i << " Inf position detected." << std::endl;
        if (anomalies.motor_under_temperature_detected[i])
            std::cout << "Motor " << i << " temperature is below minimum." << std::endl;
        if (anomalies.motor_overheat_detected[i])
            std::cout << "Joint " << i << " motor overheat detected." << std::endl;
    }
    if (std::any_of(anomalies.battery_charge_low.begin(), anomalies.battery_charge_low.end(),
                    [](bool low) { return low; }))
        std::cout << "Battery charge is low." << std::endl;
}


ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::triggerEmergencyStop() {
    anomalies.emergency_stop_triggered.store(true, std::memory_order_relaxed);
}

ROBOT_SAFETY_TEMPLATE
bool ROBOT_SAFETY_TYPE::isEmergencyStopActive() const {
    return anomalies.emergency_stop_triggered.load(std::memory_order_relaxed);
}

ROBOT_SAFETY_TEMPLATE
bool ROBOT_SAFETY_TYPE::isFungoPressed() const {
    return anomalies.fungo_pressed.load(std::memory_order_relaxed);
}

ROBOT_SAFETY_TEMPLATE
double ROBOT_SAFETY_TYPE::getReferencesTimeout() const {
    return references_timeout;
}

ROBOT_SAFETY_TEMPLATE
double ROBOT_SAFETY_TYPE::getEventsFrequency() const {
    return events_frequency;
}


ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::getVirtualJointLimitDebug(VirtualJointLimitDebug& debug) const {
    debug.predicted = anomalies.joint_pos_limit_violation_predicted;
    debug.side = recovery_virtual_joint_limit.getSide();
    debug.k = recovery_virtual_joint_limit.getK();
    debug.d = recovery_virtual_joint_limit.getD();
    debug.damping_only = recovery_virtual_joint_limit.getDampingOnly();
    debug.torque = recovery_virtual_joint_limit.getTorque();
    debug.anchor.resize(debug.side.size());
    for (size_t i = 0; i < debug.anchor.size(); ++i) {
        debug.anchor[i] = recovery_virtual_joint_limit.getAnchor(i);
    }
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::manageAnomalies(const TState& state, TCommands& commands, CONTROL_MODE control_mode) {
    checkState(state);
    checkCommands(commands, control_mode);
    // set safety state machine based on the anomalies detected
    setSafetyState();
    // Debug: report safety state transitions only (this runs every control cycle).
    static SafetyState last_safety_state = SafetyState::NOMINAL;
    if (safety_state != last_safety_state) {
        std::cout << "[safety] state " << static_cast<int>(last_safety_state)
                  << " -> " << static_cast<int>(safety_state) << std::endl;
        for (size_t i = 0; i < anomalies.joint_pos_limit_violation_predicted.size(); ++i) {
            if (anomalies.joint_pos_limit_violation_predicted[i])
                std::cout << "[safety]   joint " << i << " q=" << state.position[i] << std::endl;
        }
        last_safety_state = safety_state;
    }
    // execute the corresponding recovery mode based on the safety state machine
    if(safety_state != SafetyState::NOMINAL){
        executeRecoveryMode(state, commands);
    }
}

ROBOT_SAFETY_TEMPLATE
const TLimits& ROBOT_SAFETY_TYPE::getLimits() const {
    return limits;
}

ROBOT_SAFETY_TEMPLATE
typename ROBOT_SAFETY_TYPE::SafetyState ROBOT_SAFETY_TYPE::getSafetyState() const {
    return safety_state;
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::setSafetyState() {
    if (anomalies.emergency_stop_triggered.load(std::memory_order_relaxed) || 
        anomalies.fungo_pressed.load(std::memory_order_relaxed)) {
        safety_state = SafetyState::EMERGENCY_STOP;
    }else if (recovery_virtual_joint_limit.anyActive() || recovery_virtual_joint_limit.isDampAllLatched()) {
        safety_state = SafetyState::VIRTUAL_ENDSTOP_ENFORCING;
    }else if (anomalies.lost_references.load(std::memory_order_relaxed)) {
        safety_state = SafetyState::DAMPING;
    } else {
        safety_state = SafetyState::NOMINAL;
    }
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::executeRecoveryMode(const TState& state, TCommands& commands) {
    switch (safety_state) {
        case SafetyState::EMERGENCY_STOP:
            recovery_estop.apply(state, commands);
            break;
        case SafetyState::DAMPING:
            recovery_damping.apply(state, commands);
            break;
        case SafetyState::VIRTUAL_ENDSTOP_ENFORCING:
            recovery_virtual_joint_limit.apply(state, commands);
            break;
        default:
            break;
    }
}

ROBOT_SAFETY_TEMPLATE
void ROBOT_SAFETY_TYPE::setLostReferences(bool lost) {
    // Latched: once lost, references stay lost until init() (HAL restart).
    if (lost)
        anomalies.lost_references.store(true, std::memory_order_relaxed);
}

// RobotLowLevelControl
ROBOT_LLC_TEMPLATE
ROBOT_LLC_TYPE::RobotLowLevelControl(){}

ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::init(const YAML::Node& config){
    // Initialization logic for low-level control based on the configuration
    control_mode = static_cast<CONTROL_MODE>(config["robot_interface"]["low_level_control"]["mode"].as<int>());
    commands.init(config);
    const int num_joints = config["general"]["n_joints"].as<int>();
    k_static_friction.assign(num_joints, 0.0);
    k_dynamic_friction.assign(num_joints, 0.0);
    friction_compensation_enabled = false;
}

ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::computeCommands(const TState& state, const TRef& ref){
    // Implementation for computing commands based on state and reference
    if (control_mode == CONTROL_MODE::POSITION_CONTROL) {
        positionControl(state, ref);
    } else if (control_mode == CONTROL_MODE::VELOCITY_CONTROL) {
        velocityControl(state, ref);
    } else if (control_mode == CONTROL_MODE::TORQUE_CONTROL) {
        torqueControl(state, ref);
    } else if (control_mode == CONTROL_MODE::IMPEDANCE_CONTROL) {
        impedanceControl(state, ref);
    } else if (control_mode == CONTROL_MODE::CURRENT_CONTROL) {
        currentControl(state, ref);
    }
}

ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::positionControl(const TState& state, const TRef& ref) {
    commands.position = ref.position; // Simple position control logic    
}
ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::velocityControl(const TState& state, const TRef& ref) {
    commands.velocity = ref.velocity; // Simple velocity control logic
}
ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::torqueControl(const TState& state, const TRef& ref) {
    commands.torque = ref.torque; // Simple torque control logic
}
ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::impedanceControl(const TState& state, const TRef& ref) {
    // Impedance control logic (to be implemented)
    for (int id = 0; id < state.position.size(); ++id) {
        commands.torque[id] =   ref.torque[id] 
                                + ref.kp[id] * (ref.position[id] - state.position[id])
                                + ref.kd[id] * (ref.velocity[id] - state.velocity[id]);
        if (friction_compensation_enabled) {
            const double velocity = state.velocity[id];
            const double sign = (velocity > 0.0) - (velocity < 0.0);
            commands.torque[id] += k_static_friction[id] * sign + k_dynamic_friction[id] * velocity;
        }
    }
}
ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::setFrictionCompensation(bool enabled) {
    friction_compensation_enabled = enabled;
}
ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::setFrictionGains(int joint, double k_static, double k_dynamic) {
    const int num_joints = static_cast<int>(k_static_friction.size());
    if (joint < -1 || joint >= num_joints)
        return;
    const int first = joint == -1 ? 0 : joint;
    const int last = joint == -1 ? num_joints : joint + 1;
    for (int id = first; id < last; ++id) {
        k_static_friction[id] = k_static;
        k_dynamic_friction[id] = k_dynamic;
    }
}
ROBOT_LLC_TEMPLATE
void ROBOT_LLC_TYPE::currentControl(const TState& state, const TRef& ref) { 
    // Current control logic (to be implemented)
    std::cout << "Current control logic not implemented yet." << std::endl;
}

#undef ROBOT_SAFETY_TEMPLATE
#undef ROBOT_SAFETY_TYPE
#undef ROBOT_LLC_TEMPLATE
#undef ROBOT_LLC_TYPE
