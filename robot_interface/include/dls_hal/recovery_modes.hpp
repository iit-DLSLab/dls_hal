#ifndef DLS_HAL_RECOVERY_MODES_HPP
#define DLS_HAL_RECOVERY_MODES_HPP

#include <vector>

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

class RecoveryVirtualJointLimit : public RecoveryMode {
public:
    // Copies the position limits (RobotLimits is incomplete here, as
    // robot_invariants.hpp includes this header). k and d are per-joint gains.
    void init(const std::vector<double>& position_min, const std::vector<double>& position_max,
              const std::vector<double>& k, const std::vector<double>& d);
    // Adds a spring-damper torque pushing each out-of-limit joint back inside.
    void execute(const RobotState& state, RobotCommands& commands) override;

private:
    std::vector<double> k_;
    std::vector<double> d_;
    std::vector<double> position_min_;
    std::vector<double> position_max_;
};

#endif
