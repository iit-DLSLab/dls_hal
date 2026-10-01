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

    /**
     * @brief Call after setLimits(): the spring anchors start at the position limits
     * 
     * @param n_joint 
     * @param release_tolerance [rad]: how close to its anchor a joint must be back to deactivate.
     */
    void init(size_t n_joint, double release_tolerance);

    /**
     * @brief Activates a joint whose limit violation is predicted: anchors the spring at the
     *        current position (clamped within the position limits) on the side of the crossed
     *        threshold, with gains that stop the joint at that limit.
     * 
     * @param index 
     * @param inertia 
     * @param position 
     * @param velocity 
     * @param torque 
     * @param q_min_trigger 
     * @param q_max_trigger 
     */
    void setParams(size_t index, const double& inertia, const double& position, 
                   const double& velocity, const double& torque, 
                   const double& q_min_trigger, const double& q_max_trigger);

    /**
     * @brief Deactivates a joint: anchors back at the position limits, no gains.
     * 
     * @param index 
     */
    void reset(size_t index);
    bool isActive(size_t index) const;
    bool anyActive() const;

    /**
     * @brief True once an active joint is back at its anchor (within release_tolerance) and no
     *        longer moving toward the limit: the spring has brought it back where it triggered.
     * 
     * @param index 
     * @param position 
     * @param velocity 
     * @return true 
     * @return false 
     */
    bool hasReturnedToAnchor(size_t index, double position, double velocity) const;

protected:

    /**
     * @brief Replaces the torque of each triggered joint past its anchor with a spring-damper
     *        torque that stops it before the limit.
     * 
     * @param state 
     * @param commands 
     */
    void execute(const RobotState& state, RobotCommands& commands) override;

private:
    std::vector<double> k_;
    std::vector<double> d_;
    std::vector<double> q_min_trigger_;
    std::vector<double> q_max_trigger_;
    std::vector<int> side_; // +1 braking toward position_max, -1 toward position_min, 0 inactive
    double release_tolerance_ = 0.0;
};

#endif
