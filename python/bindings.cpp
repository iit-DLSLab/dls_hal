#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "dls_hal/generic_robot_interface.hpp"
#include "dls_hal/hal.hpp"

namespace nb = nanobind;

using GenericSafety =
    RobotSafety<RobotState, RobotRef, RobotCommands, RobotLimits>;
using GenericLowLevelControl =
    RobotLowLevelControl<RobotState, RobotRef, RobotCommands>;
using GenericRobotInterfaceBase =
    RobotInterface<RobotState, RobotRef, RobotCommands, RobotLimits,
                   GenericSafety, GenericLowLevelControl>;
using RobotAnomalies = GenericSafety::RobotAnomalies;

namespace {

class PythonInterface {
public:
    PythonInterface() = default;

    void setImplementation(nb::handle implementation) {
        implementation_ = implementation;
    }

    void init(const GenericRobotInterface& robot) {
        call("init", robot);
    }

    bool check() {
        nb::gil_scoped_acquire guard;
        if (!hasMethod("check")) {
            return true;
        }
        return nb::cast<bool>(implementation_.attr("check")());
    }

    void read(GenericRobotInterface& robot) {
        call("read", robot);
    }

    void write(const GenericRobotInterface& robot) {
        call("write", robot);
    }

private:
    bool hasMethod(const char* name) const {
        return implementation_.is_valid() && nb::hasattr(implementation_, name);
    }

    template <typename Robot>
    void call(const char* name, Robot& robot) {
        nb::gil_scoped_acquire guard;
        if (hasMethod(name)) {
            implementation_.attr(name)(
                nb::cast(&robot, nb::rv_policy::reference));
        }
    }

    nb::handle implementation_;
};

class PythonHal
    : public Hal<PythonInterface, GenericRobotInterface, PythonInterface> {
public:
    PythonHal() = default;

    PythonHal(nb::handle low_level_interface,
              nb::handle high_level_interface) {
        this->lli.setImplementation(low_level_interface);
        this->hli.setImplementation(high_level_interface);
    }

    void setLowLevelInterface(nb::handle interface) {
        this->lli.setImplementation(interface);
    }

    void setHighLevelInterface(nb::handle interface) {
        this->hli.setImplementation(interface);
    }

    GenericRobotInterface& robotInterface() {
        return this->ri;
    }
};

}  // namespace

NB_MODULE(dls_hal, m) {
    m.doc() = "Python bindings for the DLS hardware abstraction layer";

    nb::class_<IMU>(m, "IMU")
        .def(nb::init<>())
        .def("init", &IMU::init)
        .def_rw("frame_id", &IMU::frame_id)
        .def_rw("orientation", &IMU::orientation)
        .def_rw("orientation_rpy", &IMU::orientation_rpy)
        .def_rw("orientation_covariance", &IMU::orientation_covariance)
        .def_rw("angular_velocity", &IMU::angular_velocity)
        .def_rw("angular_velocity_covariance", &IMU::angular_velocity_covariance)
        .def_rw("linear_acceleration", &IMU::linear_acceleration)
        .def_rw("linear_acceleration_covariance", &IMU::linear_acceleration_covariance);

    nb::class_<RobotState>(m, "RobotState")
        .def(nb::init<>())
        .def_rw("robot_name", &RobotState::robot_name)
        .def_rw("names", &RobotState::names)
        .def_rw("position", &RobotState::position)
        .def_rw("velocity", &RobotState::velocity)
        .def_rw("acceleration", &RobotState::acceleration)
        .def_rw("torques", &RobotState::torques)
        .def_rw("motor_status_word", &RobotState::motor_status_word)
        .def_rw("motor_temperature", &RobotState::motor_temperature)
        .def_rw("motor_current", &RobotState::motor_current)
        .def_rw("contacts", &RobotState::contacts)
        .def_rw("imu", &RobotState::imu)
        .def_rw("battery_status", &RobotState::battery_status);

    nb::class_<RobotRef>(m, "RobotRef")
        .def(nb::init<>())
        .def_rw("control_mode", &RobotRef::control_mode)
        .def_rw("position", &RobotRef::position)
        .def_rw("velocity", &RobotRef::velocity)
        .def_rw("torque", &RobotRef::torque)
        .def_rw("kp", &RobotRef::kp)
        .def_rw("ki", &RobotRef::ki)
        .def_rw("kd", &RobotRef::kd);

    nb::class_<RobotCommands>(m, "RobotCommands")
        .def(nb::init<>())
        .def_rw("position", &RobotCommands::position)
        .def_rw("velocity", &RobotCommands::velocity)
        .def_rw("torque", &RobotCommands::torque);

    nb::class_<RobotLimits>(m, "RobotLimits")
        .def(nb::init<>())
        .def_rw("position_min", &RobotLimits::position_min)
        .def_rw("position_max", &RobotLimits::position_max)
        .def_rw("velocity_max", &RobotLimits::velocity_max)
        .def_rw("torque_max", &RobotLimits::torque_max)
        .def_rw("current_max", &RobotLimits::current_max)
        .def_rw("motor_temperature_min", &RobotLimits::motor_temperature_min)
        .def_rw("motor_temperature_max", &RobotLimits::motor_temperature_max)
        .def_rw("battery_charge_min", &RobotLimits::battery_charge_min);

    nb::class_<RobotAnomalies>(m, "RobotAnomalies")
        .def(nb::init<>())
        .def_rw("joint_pos_limit_violation",
                &RobotAnomalies::joint_pos_limit_violation)
        .def_rw("joint_vel_limit_violation",
                &RobotAnomalies::joint_vel_limit_violation)
        .def_rw("joint_torque_limit_violation",
                &RobotAnomalies::joint_torque_limit_violation)
        .def_rw("nan_position_detected", &RobotAnomalies::nan_position_detected)
        .def_rw("nan_velocity_detected", &RobotAnomalies::nan_velocity_detected)
        .def_rw("nan_torque_detected", &RobotAnomalies::nan_torque_detected)
        .def_rw("inf_position_detected", &RobotAnomalies::inf_position_detected)
        .def_rw("inf_velocity_detected", &RobotAnomalies::inf_velocity_detected)
        .def_rw("inf_torque_detected", &RobotAnomalies::inf_torque_detected)
        .def_rw("motor_overheat_detected",
                &RobotAnomalies::motor_overheat_detected)
        .def_rw("battery_charge_low", &RobotAnomalies::battery_charge_low);

    nb::class_<GenericSafety>(m, "RobotSafety")
        .def(nb::init<>())
        .def("eSTOP", &GenericSafety::eSTOP)
        .def("checkState", &GenericSafety::checkState)
        .def("checkReference", &GenericSafety::checkReference)
        .def("checkCommands", &GenericSafety::checkCommands)
        .def_rw("e_stop_triggered", &GenericSafety::e_stop_triggered)
        .def_rw("anomalies", &GenericSafety::anomalies)
        .def_rw("limits", &GenericSafety::limits);

    nb::enum_<GenericLowLevelControl::CONTROL_MODE>(m, "ControlMode")
        .value("POSITION_CONTROL",
               GenericLowLevelControl::CONTROL_MODE::POSITION_CONTROL)
        .value("VELOCITY_CONTROL",
               GenericLowLevelControl::CONTROL_MODE::VELOCITY_CONTROL)
        .value("TORQUE_CONTROL",
               GenericLowLevelControl::CONTROL_MODE::TORQUE_CONTROL)
        .value("IMPEDANCE_CONTROL",
               GenericLowLevelControl::CONTROL_MODE::IMPEDANCE_CONTROL)
        .value("CURRENT_CONTROL",
               GenericLowLevelControl::CONTROL_MODE::CURRENT_CONTROL);

    nb::class_<GenericLowLevelControl>(m, "RobotLowLevelControl")
        .def(nb::init<>())
        .def("computeCommands", &GenericLowLevelControl::computeCommands)
        .def("positionControl", &GenericLowLevelControl::positionControl)
        .def("velocityControl", &GenericLowLevelControl::velocityControl)
        .def("torqueControl", &GenericLowLevelControl::torqueControl)
        .def("impedanceControl", &GenericLowLevelControl::impedanceControl)
        .def("currentControl", &GenericLowLevelControl::currentControl)
        .def_rw("control_mode", &GenericLowLevelControl::control_mode)
        .def_rw("commands", &GenericLowLevelControl::commands);

    nb::class_<GenericRobotInterfaceBase>(m, "RobotInterface")
        .def(nb::init<>())
        .def("init", &GenericRobotInterfaceBase::init,
             nb::arg("config_file"))
        .def("initReferences", &GenericRobotInterfaceBase::initReferences)
        .def("checkState", &GenericRobotInterfaceBase::checkState)
        .def("checkReference", &GenericRobotInterfaceBase::checkReference)
        .def("computeCommands", &GenericRobotInterfaceBase::computeCommands)
        .def("checkCommands", &GenericRobotInterfaceBase::checkCommands)
        .def_rw("safety", &GenericRobotInterfaceBase::safety)
        .def_rw("ref", &GenericRobotInterfaceBase::ref)
        .def_rw("state", &GenericRobotInterfaceBase::state)
        .def_rw("low_level_control",
                &GenericRobotInterfaceBase::low_level_control);

    nb::class_<GenericRobotInterface, GenericRobotInterfaceBase>(
        m, "GenericRobotInterface")
        .def(nb::init<>());

    nb::class_<PythonHal>(m, "Hal")
        .def(nb::init<>())
        .def(nb::init<nb::handle, nb::handle>(),
             nb::arg("low_level_interface"),
             nb::arg("high_level_interface"),
             nb::keep_alive<1, 2>(),
             nb::keep_alive<1, 3>())
        .def("init", &PythonHal::init, nb::arg("robot_interface_config_file"))
        .def("run", &PythonHal::run)
        .def("checkHighLevelInterface", &PythonHal::checkHighLevelInterface)
        .def("checkLowLevelInterface", &PythonHal::checkLowLevelInterface)
        .def("setLowLevelInterface", &PythonHal::setLowLevelInterface,
             nb::keep_alive<1, 2>())
        .def("setHighLevelInterface", &PythonHal::setHighLevelInterface,
             nb::keep_alive<1, 2>())
        .def_prop_ro("robot_interface", &PythonHal::robotInterface,
                     nb::rv_policy::reference_internal);
}
