"""Minimal example using the nanobind DLS HAL module."""

import dls_hal


class LowLevelInterface:
    """Reads simulated hardware state and receives computed commands."""

    def init(self, robot):
        print(f"Initialized low-level interface for {len(robot.state.position)} joints")

    def check(self):
        return True

    def read(self, robot):
        joint_count = len(robot.state.position)
        robot.state.position = [0.0] * joint_count
        robot.state.velocity = [0.0] * joint_count

    def write(self, robot):
        print("Torque commands:", robot.low_level_control.commands.torque)


class HighLevelInterface:
    """Publishes state and supplies a torque reference."""

    def check(self):
        return True

    def write(self, robot):
        print("Joint positions:", robot.state.position)

    def read(self, robot):
        joint_count = len(robot.ref.torque)
        robot.ref.torque = [1.0] * joint_count


def main():
    hal = dls_hal.Hal(LowLevelInterface(), HighLevelInterface())
    hal.init("quadruped_interface_config")
    if not hal.checkLowLevelInterface() or not hal.checkHighLevelInterface():
        raise RuntimeError("A HAL interface is not ready")

    hal.run()


if __name__ == "__main__":
    main()
