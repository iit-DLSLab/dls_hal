#!/usr/bin/env python3

from __future__ import annotations

import time
from threading import Lock

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
        self._trajectory_start_ns: int | None = None
        self._initial_joint_position: np.ndarray | None = None
        self._sequence_id = 0
        self._gains_lock = Lock()
        self._kp = np.full(self.JOINT_COUNT, self.KP, dtype=float)
        self._kd = np.full(self.JOINT_COUNT, self.KD, dtype=float)

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
        
        self.add_command(
            "set_torque",
            "Set joint torques: <joint_index>[;<joint_index>...]:<torque>[;<torque>...]",
            self._split_args(self._set_torque, 2),
            enabled=True,
        )

        self.add_command(
            "set_gains",
            "Set all joint gains: <kp>:<kd>",
            self._split_args(self._set_gains, 2),
            enabled=True,
        )
        self.add_command(
            "set_joint_gain",
            "Set one joint's gains: <joint_index>:<kp>:<kd>",
            self._split_args(self._set_joint_gain, 3),
            enabled=True,
        )
    # commenting to use default activation check from PeriodicAppPlugin
    # def check_activation(self) -> bool:
    #     return True

    # commenting to use default deactivation from PeriodicAppPlugin
    # def deactivation(self, timestamp_ns: int) -> bool:
    #     del timestamp_ns
    #     return True

    @staticmethod
    def _split_args(fn, count: int):
        """add_command only accepts one string arg: split it on ':' for fn."""
        def callback(args: str) -> bool:
            fields = args.split(":")
            if len(fields) != count:
                print(f"[{APP_ID}] expected {count} ':'-separated args, got {args!r}")
                return False
            return fn(*fields)
        return callback

    def _set_torque(self, joint_index: str, torque: str) -> bool:
        try:
            # Commas separate command arguments in the DDS transport.
            indices = [int(value.strip()) for value in joint_index.split(";")]
            values = [float(value.strip()) for value in torque.split(";")]
        except ValueError:
            print(f"[{APP_ID}] malformed set_torque args: {joint_index!r} {torque!r}")
            return False
        if len(indices) != len(values):
            print(f"[{APP_ID}] expected one torque per joint index")
            return False
        if any(not 0 <= idx < self.JOINT_COUNT for idx in indices) or not np.all(np.isfinite(values)):
            print(f"[{APP_ID}] invalid joint index or torque: {indices} {values}")
            return False

        torques = np.zeros(self.JOINT_COUNT)
        for idx, tau in zip(indices, values):
            torques[idx] = tau
        self.writer_example.data.joints_torques(self._as_double_vector(torques))
        return True
        
    def run(self, timestamp_ns: int) -> None:
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
        with self._gains_lock:
            kp = self._kp.copy()
            kd = self._kd.copy()
        msg.kp(self._as_double_vector(kp))
        msg.kd(self._as_double_vector(kd))

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

    def _set_gains(self, kp_value: str, kd_value: str) -> bool:
        """Set proportional and derivative gains for every joint."""
        gains = self._parse_gains(kp_value, kd_value)
        if gains is None:
            print(f"[{APP_ID}] usage: set_gains <kp> <kd>")
            return False

        kp, kd = gains
        with self._gains_lock:
            self._kp.fill(kp)
            self._kd.fill(kd)
        print(f"[{APP_ID}] set all gains: kp={kp}, kd={kd}")
        return True

    def _set_joint_gain(
        self,
        joint_index_value: str,
        kp_value: str,
        kd_value: str,
    ) -> bool:
        """Set proportional and derivative gains for one zero-indexed joint."""
        try:
            joint_index = int(joint_index_value)
        except ValueError:
            print(f"[{APP_ID}] joint index must be an integer: {joint_index_value!r}")
            return False

        if not 0 <= joint_index < self.JOINT_COUNT:
            print(
                f"[{APP_ID}] joint index must be between 0 and "
                f"{self.JOINT_COUNT - 1}: {joint_index}"
            )
            return False

        gains = self._parse_gains(kp_value, kd_value)
        if gains is None:
            print(f"[{APP_ID}] usage: set_joint_gain <joint_index> <kp> <kd>")
            return False

        kp, kd = gains
        with self._gains_lock:
            self._kp[joint_index] = kp
            self._kd[joint_index] = kd
        print(f"[{APP_ID}] set joint {joint_index} gains: kp={kp}, kd={kd}")
        return True

    @staticmethod
    def _parse_gains(kp_value: str, kd_value: str) -> tuple[float, float] | None:
        try:
            kp = float(kp_value)
            kd = float(kd_value)
        except ValueError:
            print(f"[{APP_ID}] gains must be numeric: {kp_value!r} {kd_value!r}")
            return None

        if not np.isfinite(kp) or not np.isfinite(kd) or kp < 0.0 or kd < 0.0:
            print(f"[{APP_ID}] gains must be finite, non-negative values")
            return None

        return kp, kd

    TRAJECTORY_DURATION_S = 1000
    TRAJECTORY_FREQUENCY_HZ = 0.5
    TRAJECTORY_AMPLITUDE_RAD = 0.1
    JOINT_COUNT = 12
    KP = 0
    KD = 0


def main() -> None:
    app = TestHalApp()
    app.execute()


if __name__ == "__main__":
    main()
