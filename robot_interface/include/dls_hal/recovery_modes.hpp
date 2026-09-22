#ifndef DLS_HAL_RECOVERY_MODES_HPP
#define DLS_HAL_RECOVERY_MODES_HPP

class RobotState;
class RobotCommands;

class RecoveryMode {
public:
    virtual ~RecoveryMode();
    virtual void run(const RobotState& state, RobotCommands& commands) = 0;
};

class RecoveryESTOP : public RecoveryMode {
public:
    void run(const RobotState& state, RobotCommands& commands) override;
};

class RecoveryDamping : public RecoveryMode {
public:
    // Torque is -damping_gain * measured velocity; position/velocity commands are zero.
    explicit RecoveryDamping(double damping_gain = 1.0);
    void run(const RobotState& state, RobotCommands& commands) override;

private:
    double damping_gain_;
};

#endif
