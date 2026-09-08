#!/usr/bin/env python3

from __future__ import annotations

import time

import numpy as np

from dls2_py.env_bootstrap import setup_environment


APP_ID = "test_hal"
SIGNALS_DOMAIN = 3

PeriodicAppPlugin = setup_environment(
    package_name=APP_ID,
    module_file=__file__,
    required_message_modules=("BlindState", "ControlSignal"),
)

import BlindState  # noqa: E402
import ControlSignal  # noqa: E402


class TestHalApp(PeriodicAppPlugin):
    def __init__(self) -> None:
        super().__init__(APP_ID, SIGNALS_DOMAIN)
        self._last_report_time = 0.0
        self._trajectory_start_ns: int | None = None
        self._initial_joint_position: np.ndarray | None = None
        self._sequence_id = 0

        # Example:
        self.reader_example = self.build_input(
            "rt/blind_state",
            BlindState.BlindStatePubSubType(),
            BlindState.BlindState(),
            required_on_activation=False,
        )

        self.writer_example = self.build_output(
            "rt/test_hal",
            ControlSignal.ControlSignalPubSubType(),
            ControlSignal.ControlSignal(),
        )
        self.writer_example.data.control_mode(1)
    # commenting to use default activation check from PeriodicAppPlugin
    # def check_activation(self) -> bool:
    #     return True

    # commenting to use default deactivation from PeriodicAppPlugin
    # def deactivation(self, timestamp_ns: int) -> bool:
    #     del timestamp_ns
    #     return True

    def run(self, timestamp_ns: int) -> None:
        now = time.time()
        if now - self._last_report_time >= 1.0:
            self._last_report_time = now
            print(f"[{APP_ID}] periodic app running")

        # Add your periodic logic here.
        # Example:
        self.read()

        if self._initial_joint_position is None:
            current_position = np.asarray(
                self.reader_example.getData().joints_position(),
                dtype=float,
            )
            if current_position.size != self.JOINT_COUNT:
                return

            self._initial_joint_position = current_position.copy()
            self._trajectory_start_ns = timestamp_ns
            print(
                f"[{APP_ID}] trajectory initialized around "
                f"{self._initial_joint_position.tolist()}"
            )

        assert self._trajectory_start_ns is not None
        elapsed_s = (timestamp_ns - self._trajectory_start_ns) * 1e-9
        position, velocity, acceleration = self._sine_trajectory(elapsed_s)

        msg = self.writer_example.data
        msg.timestamp(timestamp_ns)
        msg.sequence_id(self._sequence_id % 1000)
        self._sequence_id += 1
        msg.joints_position(self._as_double_vector(position))
        msg.joints_velocity(self._as_double_vector(velocity))
        # set gains
        msg.kp(self._as_double_vector(np.full(self.JOINT_COUNT, self.KP)))
        msg.kd(self._as_double_vector(np.full(self.JOINT_COUNT, self.KD)))

        self.writer_example.write()

    def _sine_trajectory(self, elapsed_s: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return a sine trajectory centered on the initial measured pose."""
        if self._initial_joint_position is None:
            raise RuntimeError("Joint trajectory has not been initialized")

        if not 0.0 <= elapsed_s < self.TRAJECTORY_DURATION_S:
            zeros = np.zeros(self.JOINT_COUNT)
            return self._initial_joint_position.copy(), zeros, zeros.copy()

        angular_frequency = 2.0 * np.pi * self.TRAJECTORY_FREQUENCY_HZ
        phase = angular_frequency * elapsed_s

        position = (
            self._initial_joint_position
            + self.TRAJECTORY_AMPLITUDE_RAD * np.sin(phase)
        )
        velocity = np.full(
            self.JOINT_COUNT,
            self.TRAJECTORY_AMPLITUDE_RAD * angular_frequency * np.cos(phase),
        )
        acceleration = np.full(
            self.JOINT_COUNT,
            -self.TRAJECTORY_AMPLITUDE_RAD * angular_frequency**2 * np.sin(phase),
        )
        return position, velocity, acceleration

    def _as_double_vector(self, values: np.ndarray) -> object:
        vector = ControlSignal.double_vector()
        for value in values:
            vector.push_back(float(value))
        return vector

    TRAJECTORY_DURATION_S = 1000
    TRAJECTORY_FREQUENCY_HZ = 0.5
    TRAJECTORY_AMPLITUDE_RAD = 0.25
    JOINT_COUNT = 12
    KP = 50
    KD = 5


def main() -> None:
    app = TestHalApp()
    app.execute()


if __name__ == "__main__":
    main()
