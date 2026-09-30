#ifndef DLS_HAL_RECOVERY_MODES_HPP
#define DLS_HAL_RECOVERY_MODES_HPP

#include <vector>
#include <cmath>

class RobotState;
class RobotCommands;

// Command limits the recovery modes enforce. A plain copy of the relevant
// RobotLimits fields: RobotLimits is incomplete here, as robot_invariants.hpp
// includes this header.
struct RecoveryLimits {
    std::vector<double> position_min;
    std::vector<double> position_max;
    std::vector<double> velocity_min;
    std::vector<double> velocity_max;
    std::vector<double> torque_min;
    std::vector<double> torque_max;
};

class RecoveryMode {
public:
    virtual ~RecoveryMode();

    // Throws if any min/max pair differs in size, is NaN, or has min > max.
    void setLimits(const RecoveryLimits& limits);

    // Runs the recovery, then makes every command finite and within limits:
    // a non-finite position becomes the measured position, a non-finite velocity or torque becomes 0.
    void apply(const RobotState& state, RobotCommands& commands);

protected:
    virtual void execute(const RobotState& state, RobotCommands& commands) = 0;

    RecoveryLimits limits_;

private:
    void sanitize(const RobotState& state, RobotCommands& commands) const;
};

class RecoveryESTOP : public RecoveryMode {
protected:
    void execute(const RobotState& state, RobotCommands& commands) override;
};

class RecoveryDamping : public RecoveryMode {
public:
    // Torque is -damping_gain * measured velocity; position/velocity commands are zero.
    explicit RecoveryDamping(double damping_gain = 1.0);

protected:
    void execute(const RobotState& state, RobotCommands& commands) override;

private:
    double damping_gain_;
};

class RecoveryVirtualJointLimit : public RecoveryMode {
public:
    // k and d are per-joint gains; the position limits come from setLimits().
    void init(const std::vector<double>& k, const std::vector<double>& d);
    void setParams(size_t index, const double& inertia, const double& position, 
                   const double& velocity, const double& torque);

protected:
    // Adds a spring-damper torque pushing each out-of-limit joint back inside.
    void execute(const RobotState& state, RobotCommands& commands) override;

private:
    std::vector<double> k_;
    std::vector<double> d_;
};

#endif
