#ifndef DLS_HAL_RECOVERY_MODES_HPP
#define DLS_HAL_RECOVERY_MODES_HPP

#include <cstddef>
#include <vector>

#include "yaml-cpp/yaml.h"

class RobotState;
class RobotCommands;

class RecoveryMode {
public:
    virtual ~RecoveryMode();
    virtual void execute(const RobotState& state, RobotCommands& commands) = 0;
};

class RecoveryESTOP : public RecoveryMode {
public:
    void execute(const RobotState& state, RobotCommands& commands) override;
};

class RecoveryDamping : public RecoveryMode {
public:
    // Torque is -damping_gain * measured velocity; position/velocity commands are zero.
    explicit RecoveryDamping(double damping_gain = 1.0);
    void execute(const RobotState& state, RobotCommands& commands) override;

private:
    double damping_gain_;
};

// Brakes a joint before it reaches its position limit. At constant
// deceleration a, a joint moving at speed |v| travels
//     delta_q = |v| * reaction_time + v^2 / (2 * a)
// before stopping. Braking is required when the joint moves toward a limit
// and that distance reaches the remaining distance to the limit (minus
// position_margin). Equivalently, |v| exceeds the safe velocity envelope
//     v_safe(d) = -a * t_r + sqrt((a * t_r)^2 + 2 * a * d).
// Only braking joints are overridden; the others keep their commands.
class RecoveryJointLimitHardStop : public RecoveryMode {
public:
    // Disabled unless robot_interface.safety.joint_limit_hard_stop.enabled is true.
    void init(const YAML::Node& config,
              const std::vector<double>& position_min,
              const std::vector<double>& position_max);

    bool isEnabled() const;

    bool isBrakingRequired(std::size_t joint, double position, double velocity) const;

    void execute(const RobotState& state, RobotCommands& commands) override;

private:
    // Command field that carries the braking action for the configured control mode.
    enum class BrakingAction { NONE, TORQUE, VELOCITY, POSITION };

    // Signed limit the joint is moving toward, reduced by the margin.
    double approachedLimit(std::size_t joint, double velocity) const;
    double remainingDistance(std::size_t joint, double position, double velocity) const;
    double safeVelocity(std::size_t joint, double distance) const;

    bool enabled_{false};
    BrakingAction action_{BrakingAction::NONE};
    double reaction_time_{0.0};
    double position_margin_{0.0};
    std::vector<double> braking_acceleration_;
    std::vector<double> braking_gain_;
    std::vector<double> position_min_;
    std::vector<double> position_max_;
};

#endif
