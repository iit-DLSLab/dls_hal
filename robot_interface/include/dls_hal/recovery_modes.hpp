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
     * @param damping_after_stop: once a braked joint starts moving away from the limit, only damp
     *        it from then on (latched until the HAL restarts) instead of springing it back.
     * @param damping_ratio: zeta of the spring-damper, in [0.5, 1]. The peak torque (pure damping,
     *        at activation) fixes d, so zeta only sets the stiffness: k = k(zeta = 1) / zeta^2.
     *        Below 1 the brake stops in a shorter distance; below 0.5 the peak torque is no longer
     *        at activation and the sizing below does not hold.
     * @param stop_retreat [rad]: with damping_after_stop, how far a braked joint must move back
     *        from the deepest position it reached before it is latched into damping only.
     */
    void init(size_t n_joint, double release_tolerance, bool damping_after_stop = false,
              double damping_ratio = 1.0, double stop_retreat = 1e-3);

    /**
     * @brief c(zeta) in the stopping distance c * I * w^2 / tau of the brake whose peak torque
     *        is tau: 2 * zeta * exp(-zeta * arccos(zeta) / sqrt(1 - zeta^2)), 2 / e at zeta = 1.
     *        The trigger must use it, so that it matches the braking law.
     */
    double getStoppingDistanceCoefficient() const;

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
     * @brief With damping_after_stop, latches an active joint into damping only once it has
     *        moved back by more than stop_retreat from the deepest position reached since its
     *        activation (impact avoided, now moving away from the limit). Decided on position,
     *        not on the velocity sign, so that velocity noise cannot latch a joint still moving
     *        toward the limit. Never released: the joint stays damped until the HAL restarts.
     *        No effect without damping_after_stop.
     * 
     * @param index 
     * @param position 
     */
    void updateDampingOnly(size_t index, double position);
    bool isDampingOnly(size_t index) const;
    bool isDampingAfterStop() const;
    const std::vector<bool>& getDampingOnly() const;

    /**
     * @brief Per-joint spring stiffness [Nm/rad], damping [Nm*s/rad] and side (+1 braking
     *        toward position_max, -1 toward position_min, 0 inactive) of the last activation.
     */
    const std::vector<double>& getK() const;
    const std::vector<double>& getD() const;
    const std::vector<int>& getSide() const;

    /**
     * @brief Per-joint torque [Nm] the last execute() computed, before the torque limits clamp
     *        it; NaN for joints whose controller torque was kept.
     */
    const std::vector<double>& getTorque() const;

    /**
     * @brief Anchor [rad] of an active joint, NaN when the joint is inactive.
     * 
     * @param index 
     */
    double getAnchor(size_t index) const;

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
    std::vector<bool> damping_only_; // latched: damped with d_ only, never released
    std::vector<double> peak_position_; // [rad] deepest position toward the limit since activation, NaN when inactive
    std::vector<double> tau_; // [Nm] computed by the last execute(), NaN when not overridden
    double release_tolerance_ = 0.0;
    bool damping_after_stop_ = false;
    double stop_retreat_ = 1e-3;
    double damping_ratio_ = 1.0;
    // Penetration from velocity w: w * sqrt(I / k) * penetration_factor_ (1 / e at zeta = 1).
    double penetration_factor_ = 1.0 / M_E;
    // Peak of the step response to a constant push tau, in units of tau / k (1 at zeta = 1).
    double push_overshoot_ = 1.0;
};

#endif
