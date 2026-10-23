"""HAL anomaly tests: catalog and runner.

The HAL is used as it is: anomalies are provoked from outside only, by the
references this tester publishes and by the limits of the active test config
(hal_test_config). Detection is read from the HAL events (rt/log_events) and
its commands feedback (rt/hal_commands_feedback).

Pure Python, no DDS: hal_anomaly_tester.py feeds the runner every tick, which
keeps the logic unit testable (test_anomaly_tests.py).
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

# EventID values (dls2/log/event_config.hpp), used when the installed header
# cannot be read. Keep in sync: test_anomaly_tests.py checks it.
FALLBACK_EVENT_IDS = {
    "PROCESS_DIED": 0, "WRONG_PROCESS_STATE": 1, "WRONG_PROCESS_FREQUENCY": 2, "MISSING_INPUT": 3,
    "CPU_USAGE_TOO_HIGH": 4, "MEM_USAGE_TOO_HIGH": 5, "INPUTS_NOT_SYNCHRONIZED": 6,
    "WRONG_INPUT_FREQUENCY": 7, "WRONG_SEQUENCE_ID": 8, "JOINT_POS_OUT_OF_BOUND": 9,
    "JOINT_VEL_OUT_OF_BOUND": 10, "JOINT_ACC_OUT_OF_BOUND": 11, "JOINT_TORQUE_OUT_OF_BOUND": 12,
    "JOINT_CURRENT_OUT_OF_BOUND": 13, "REACHED_KINEMATIC_SINGULARITY": 14, "DETECTED_AUTOCOLLISION": 15,
    "NAN_VALUE": 16, "INF_VALUE": 17, "MOTOR_OVERHEAT": 18, "MOTOR_UNDER_TEMPERATURE": 19,
    "TOO_HIGH_VALUE": 20, "BATTERY_CHARGE_LOW": 21, "EMERGENCY_STOP_PRESSED": 22, "FUNGO_PRESSED": 23,
    "NOMINAL_OPERATION": 24, "GENERAL_WARNING": 25, "GENERAL_ERROR": 26, "BREAKING_MECHANICS": 27,
    "SAFETY_STATE_NOT_NOMINAL": 28, "JOINT_POS_CLAMPING_ANOMALY": 29, "JOINT_VEL_CLAMPING_ANOMALY": 30,
    "JOINT_TORQUE_CLAMPING_ANOMALY": 31,
}


def parse_event_ids(header_text: str) -> dict[str, int]:
    """EventID names to values, from the text of event_config.hpp."""
    body = re.search(r"enum\s+class\s+EventID[^{]*\{(.*?)\};", header_text, re.S)
    if body is None:
        raise ValueError("EventID enum not found")
    ids, value = {}, -1
    for line in body.group(1).splitlines():
        line = line.split("//")[0].strip().rstrip(",")
        if not line:
            continue
        name, _, explicit = line.partition("=")
        value = int(explicit.strip(), 0) if explicit.strip() else value + 1
        ids[name.strip()] = value
    return ids


CONTROL_MODES = {0: "TORQUE", 1: "IMPEDANCE", 2: "POSITION", 3: "VELOCITY", 4: "CURRENT"}

DEFAULT_CONFIG = {
    "target": "real",             # "sim" or "real"
    "tests": ["all"],             # "all" = every non-latching test of the active suite
    "joint": 0,                   # test joint: the one with the narrow limits in the test configs
    "hal_components": ["gazebo_sim", "pegasus_hal"],  # event sources accepted as the HAL
    "estop_command_owner": "",    # e.g. "pegasus_hal": owner of the "emergency" console command
    "freeze_base_owner": "",      # e.g. "gazebo_sim": call its "freezeBase" at start, legs hanging like the suspended robot
    "freeze_settle_time": 3.0,    # s of extra settling after freezing the base
    "hold_kp": 50.0,              # gains holding the posture
    "hold_kd": 5.0,
    "ramp_speed": 0.5,            # rad/s, test joint motions (keep below its velocity_max)
    "max_overshoot": 0.3,         # rad: how far past position_max the reference may push (friction, gravity)
    "settle_time": 2.0,           # s of holding before the first test
    "nominal_time": 3.0,          # s the nominal test must stay free of HAL events
    "quiet_time": 1.0,            # s without HAL events that count as "anomalies cleared"
    "clean_timeout": 5.0,         # s for the anomalies to clear before a test
    "recover_time": 1.0,          # s of holding after a test
    "test_timeout": 3.0,          # s for an anomaly to be reported
    "slow_test_timeout": 10.0,    # s for lost_references and mechanical_stress
    "manual_timeout": 60.0,       # s for the operator (fungo)
    "max_real_torque": 20.0,      # Nm: real robot, refuse torque pulses above this torque_max
    "max_real_range": 1.0,        # rad: real robot, refuse motions if the test joint range is wider
    "damping_gain": 1.0,          # RecoveryDamping gain, for the damping feedback check
    "strict_latching": False,     # True: one latching test per session, each state transition proven
    "report_file": "/tmp/hal_anomaly_tester_report.json",
    "coverage_file": "/tmp/hal_anomaly_tester_coverage.json",  # anomalies verified over all sessions; delete to reset
}



# --- Active HAL config ------------------------------------------------------

@dataclass
class HalConfig:
    """What the tester needs from the robot interface config the HAL loaded."""
    suite: str | None
    mode: int
    references_timeout: float
    position_min: list[float]
    position_max: list[float]
    velocity_max: list[float]
    torque_max: list[float]
    stress_time_scale: float      # mechanical_stress_exp_coefficients.a, 0 = disabled

    @classmethod
    def from_dict(cls, config: dict) -> "HalConfig":
        ri = config.get("robot_interface", {}) or {}
        safety = ri.get("safety", {}) or {}
        limits = safety.get("limits") or safety
        n = int(config["general"]["n_joints"])
        big = 1e300

        def vec(name, default):
            values = limits.get(name)
            return [float(v) for v in values] if values else [default] * n

        coefficients = safety.get("mechanical_stress_exp_coefficients") or {}
        return cls(
            suite=config.get("test_config"),
            mode=int((ri.get("low_level_control") or {}).get("mode", 1)),
            references_timeout=float(ri.get("references_timeout", 1.0)),
            position_min=vec("position_min", -big), position_max=vec("position_max", big),
            velocity_max=vec("velocity_max", big), torque_max=vec("torque_max", 0.0),
            stress_time_scale=float(coefficients.get("a", 0.0)),
        )


# --- Test catalog -----------------------------------------------------------

@dataclass
class Spec:
    name: str
    description: str
    suite: str                       # test config the test needs (hal_test_config enable <suite>)
    trigger: str                     # how the anomaly is provoked, see Runner._drive()
    expected: tuple[str, ...]        # HAL EventIDs that must be reported
    latching: bool = False           # leaves the HAL in EMERGENCY_STOP/DAMPING: restart it after
    final_state: str = "NOMINAL"
    feedback_check: str | None = None  # "zero" or "damping", once final_state is reached
    targets: tuple[str, ...] = ("sim", "real")
    moves_joint: bool = False        # real robot: needs the narrow test joint envelope
    slow: bool = False


CATALOG: list[Spec] = [
    # suite "base": narrow envelope on the test joint, quick timeouts
    Spec("position_limit", "test joint moved past position_max", "base", "excursion",
         ("JOINT_POS_OUT_OF_BOUND",), moves_joint=True),
    Spec("velocity_limit", "test joint oscillated faster than velocity_max", "base", "sine",
         ("JOINT_VEL_OUT_OF_BOUND",), moves_joint=True),
    Spec("torque_clamping", "torque pulses above torque_max are clamped", "base", "torque_pulses",
         ("JOINT_TORQUE_CLAMPING_ANOMALY",), moves_joint=True),
    Spec("current_limit", "torque pulses draw more than current_max", "base", "torque_pulses",
         ("JOINT_CURRENT_OUT_OF_BOUND",), targets=("real",), moves_joint=True),
    # thresholds no real value satisfies: active for the whole session
    Spec("overheat", "motor_temperature_max below any temperature", "base", "none",
         ("MOTOR_OVERHEAT",)),
    Spec("under_temperature", "motor_temperature_min above any temperature", "base", "none",
         ("MOTOR_UNDER_TEMPERATURE",)),
    Spec("battery_low", "percentage_battery_charge_min above any charge", "base", "none",
         ("BATTERY_CHARGE_LOW",)),
    # Latching, in the order they can share a session: mechanical_stress needs
    # the joint to follow the references (NOMINAL), the others only need the
    # HAL to keep checking, which it does in DAMPING and EMERGENCY_STOP too.
    Spec("mechanical_stress", "test joint held past position_max under load", "base", "hold_outside",
         ("JOINT_POS_OUT_OF_BOUND", "BREAKING_MECHANICS"), latching=True, final_state="DAMPING",
         feedback_check="damping", moves_joint=True, slow=True),
    Spec("lost_references", "references stop for longer than references_timeout", "base",
         "silence", ("MISSING_INPUT",), latching=True, final_state="DAMPING",
         feedback_check="damping", slow=True),
    Spec("estop", "emergency console command", "base", "estop_command",
         ("EMERGENCY_STOP_PRESSED",), latching=True, final_state="EMERGENCY_STOP",
         feedback_check="zero", targets=("real",)),
    Spec("invalid_torque_command", "NaN and Inf torque references", "base", "invalid_torque",
         ("NAN_VALUE", "INF_VALUE", "EMERGENCY_STOP_PRESSED"), latching=True,
         final_state="EMERGENCY_STOP", feedback_check="zero"),
    Spec("fungo", "operator presses the fungo button", "base", "operator",
         ("FUNGO_PRESSED",), latching=True, final_state="EMERGENCY_STOP", feedback_check="zero",
         targets=("real",)),
    # suite "position_mode": POSITION_CONTROL
    Spec("position_clamping", "position reference past position_max is clamped", "position_mode",
         "position_beyond", ("JOINT_POS_CLAMPING_ANOMALY",), moves_joint=True),
    Spec("invalid_position_command", "NaN and Inf position references", "position_mode",
         "invalid_position", ("NAN_VALUE", "INF_VALUE", "EMERGENCY_STOP_PRESSED"), latching=True,
         final_state="EMERGENCY_STOP", feedback_check="zero"),
    # suite "velocity_mode": VELOCITY_CONTROL
    Spec("velocity_clamping", "velocity reference pulses above velocity_max are clamped",
         "velocity_mode", "velocity_pulses", ("JOINT_VEL_CLAMPING_ANOMALY",), moves_joint=True),
    Spec("invalid_velocity_command", "NaN and Inf velocity references", "velocity_mode",
         "invalid_velocity", ("NAN_VALUE", "INF_VALUE", "EMERGENCY_STOP_PRESSED"), latching=True,
         final_state="EMERGENCY_STOP", feedback_check="zero"),
]
SPECS = {spec.name: spec for spec in CATALOG}

# Every anomaly flag of RobotAnomalies (robot_invariants.hpp), in declaration
# order. test_anomaly_tests.py checks it against the header.
ALL_ANOMALIES = (
    "joint_pos_limit_violation", "joint_vel_limit_violation", "joint_torque_limit_violation",
    "joint_current_limit_violation", "nan_position_detected", "nan_velocity_detected",
    "nan_torque_detected", "inf_position_detected", "inf_velocity_detected", "inf_torque_detected",
    "joint_pos_clamping_anomaly", "joint_vel_clamping_anomaly", "joint_torque_clamping_anomaly",
    "nan_position_command_detected", "nan_velocity_command_detected", "nan_torque_command_detected",
    "inf_position_command_detected", "inf_velocity_command_detected", "inf_torque_command_detected",
    "motor_overheat_detected", "motor_under_temperature_detected", "battery_charge_low",
    "emergency_stop_triggered", "fungo_pressed", "lost_references", "breaking_mechanics",
)

# RobotAnomalies flags a test verifies when it passes.
COVERS = {
    "position_limit": ("joint_pos_limit_violation",),
    "velocity_limit": ("joint_vel_limit_violation",),
    "torque_clamping": ("joint_torque_clamping_anomaly",),
    "current_limit": ("joint_current_limit_violation",),
    "estop": ("emergency_stop_triggered",),
    "fungo": ("fungo_pressed",),
    "invalid_torque_command": ("nan_torque_command_detected", "inf_torque_command_detected",
                               "emergency_stop_triggered"),
    "lost_references": ("lost_references",),
    "mechanical_stress": ("joint_pos_limit_violation", "breaking_mechanics"),
    "overheat": ("motor_overheat_detected",),
    "under_temperature": ("motor_under_temperature_detected",),
    "battery_low": ("battery_charge_low",),
    "position_clamping": ("joint_pos_clamping_anomaly",),
    "invalid_position_command": ("nan_position_command_detected", "inf_position_command_detected",
                                 "emergency_stop_triggered"),
    "velocity_clamping": ("joint_vel_clamping_anomaly",),
    "invalid_velocity_command": ("nan_velocity_command_detected", "inf_velocity_command_detected",
                                 "emergency_stop_triggered"),
}
# Anomalies no test can provoke from outside the HAL (covered by the gtests).
NOT_TESTABLE = {
    **{f"{kind}_{quantity}_detected": "measured state comes from the sensors/Gazebo"
       for kind in ("nan", "inf") for quantity in ("position", "velocity", "torque")},
    "joint_torque_limit_violation": "measured torque follows the clamped command (no test yet)",
}


def coverage_report(verified: set[str], title: str, reasons: dict[str, str] | None = None) -> str:
    """Lists the verified RobotAnomalies flags."""
    reasons = reasons or {}
    lines = [f"===== Anomalies verified ({title}): {len(verified & set(ALL_ANOMALIES))}/{len(ALL_ANOMALIES)} ====="]
    width = max(len(name) for name in ALL_ANOMALIES)
    for name in ALL_ANOMALIES:
        if name in verified:
            lines.append(f"  [x] {name:<{width}}  {reasons.get(name, '')}".rstrip())
    return "\n".join(lines)


SUITES = sorted({spec.suite for spec in CATALOG})
NOMINAL = Spec("nominal", "holding the posture raises no HAL event", "*", "none", ())

# Events always active in a suite, by construction of its config.
STATIC_EVENTS = {"base": {"MOTOR_OVERHEAT", "MOTOR_UNDER_TEMPERATURE", "BATTERY_CHARGE_LOW"}}

# Safety states, in the order the HAL escalates through them.
STATE_RANK = {"NOMINAL": 0, "DAMPING": 1, "EMERGENCY_STOP": 2}


def state_in(msg: str) -> str:
    """Safety state named in a SAFETY_STATE_NOT_NOMINAL event."""
    return "EMERGENCY_STOP" if "EMERGENCY_STOP" in msg else "DAMPING" if "DAMPING" in msg else "NOMINAL"


def select_tests(names: list[str], suite: str | None, strict: bool = False) -> list[Spec]:
    """Expands "all" for the active suite, drops duplicates, puts the latching tests
    last in catalog order. "all" includes the latching tests unless strict.

    Raises ValueError for unknown names, or more than one latching test if strict.
    """
    selected: list[Spec] = []
    for name in names:
        name = name.strip()
        if not name or name == "nominal":
            continue
        if name == "all":
            candidates = [s for s in CATALOG if s.suite == suite and not (strict and s.latching)]
        elif name in SPECS:
            candidates = [SPECS[name]]
        else:
            raise ValueError(f"unknown test '{name}' (available: all, {', '.join(SPECS)})")
        selected += [s for s in candidates if s not in selected]
    latching = sorted((s for s in selected if s.latching), key=CATALOG.index)
    if strict and len(latching) > 1:
        raise ValueError(f"strict_latching: at most one latching test per HAL session, got "
                         f"{', '.join(s.name for s in latching)}")
    return [NOMINAL] + [s for s in selected if not s.latching] + latching


# --- Runner -----------------------------------------------------------------

@dataclass
class Event:
    time: float          # s, monotonic, when received
    component: str
    event_id: int
    severity: int
    msg: str


@dataclass
class Observation:
    now: float                              # s, monotonic
    position: list[float] | None = None     # rt/blind_state, None until received
    velocity: list[float] | None = None
    feedback: dict | None = None            # rt/hal_commands_feedback: control_mode, position, velocity, torque
    events: list[Event] = field(default_factory=list)  # new events since the previous tick


@dataclass
class Output:
    publish_reference: bool
    position: list[float]
    velocity: list[float]
    torque: list[float]
    kp: list[float]
    kd: list[float]
    command_call: tuple[str, str] | None = None   # (owner, command) to call once


@dataclass
class Result:
    name: str
    outcome: str            # PASS, FAIL or SKIP
    detail: str
    duration: float = 0.0
    anomalies: list[str] = field(default_factory=list)  # RobotAnomalies flags it verified


PULSE_PERIOD = 0.3   # s between torque/velocity pulses (one tick each)


class Runner:
    """Runs the selected tests, one tick at a time."""

    def __init__(self, config: dict, hal: HalConfig, event_ids: dict[str, int] | None = None,
                 log=print) -> None:
        self.cfg = {**DEFAULT_CONFIG, **config}
        self.hal = hal
        self.ids = event_ids or FALLBACK_EVENT_IDS
        self.names = {value: name for name, value in self.ids.items()}
        self.log = log
        self.results: list[Result] = []
        self.queue: list[Spec] = []
        self.finished = False
        self.n = 0
        self.j = 0
        self.hold: list[float] | None = None
        self.goal = 0.0                 # test joint position goal, reached at ramp_speed
        self._phase = "idle"
        self._phase_start = 0.0
        self._last_now: float | None = None
        self._last_event_at = -math.inf
        self._events_received = 0
        self._spec: Spec | None = None
        self._seen: list[Event] = []
        self._state_reached = False
        self._feedback_ok = False
        self._last_command_call = -math.inf
        self._pulse_index = -1
        self._unexpected_safety: Event | None = None
        self._clean_events: list[Event] = []
        self._pending_call: tuple[str, str] | None = None
        self._reached = -math.inf
        self._hal_state = "NOMINAL"     # latest safety state reported by the HAL
        self._latched = "NOMINAL"       # safety state reached by the latching tests run so far
        self._start_state = "NOMINAL"   # HAL safety state when the running test started

    # Public API ----------------------------------------------------------------

    def start(self, names: list[str]) -> bool:
        """Queues tests; returns False (and logs why) if the selection is invalid."""
        try:
            self.queue = select_tests(names, self.hal.suite, self.cfg["strict_latching"])
        except ValueError as error:
            self.log(f"invalid test selection: {error}")
            return False
        self.results = []
        self.finished = False
        self._phase = "wait_hal"
        self.log(f"suite '{self.hal.suite}', tests: " + ", ".join(spec.name for spec in self.queue))
        return True

    @property
    def busy(self) -> bool:
        return self._phase not in ("idle", "done")

    def step(self, obs: Observation) -> Output:
        dt = 0.0 if self._last_now is None else max(0.0, obs.now - self._last_now)
        self._last_now = obs.now
        self._take_events(obs)
        if obs.position is not None and self.hold is None:
            self.n = len(obs.position)
            self.j = self.cfg["joint"] % self.n
            self.hold = list(obs.position)
            self.goal = self.hold[self.j]
        if self.hold is not None:
            step = self.cfg["ramp_speed"] * dt
            self.hold[self.j] += max(-step, min(step, self.goal - self.hold[self.j]))
            if self.busy:
                self._advance(obs)
        return self._output(obs)

    def verified_anomalies(self) -> set[str]:
        """RobotAnomalies flags verified by the tests passed in this session."""
        return {a for r in self.results if r.outcome == "PASS" for a in r.anomalies}

    def coverage(self, earlier: set[str] = frozenset(), title: str = "this session") -> str:
        """Anomalies verified in this session and in `earlier` ones, with the tests of this session."""
        passed = [r.name for r in self.results if r.outcome == "PASS"]
        session = self.verified_anomalies()
        credited = {r.name: r.anomalies for r in self.results if r.outcome == "PASS"}
        reasons = {a: "by " + ", ".join(t for t in passed if a in credited[t]) for a in session}
        reasons.update({a: "earlier session" for a in earlier - session})
        return coverage_report(session | earlier, title, reasons)

    def summary(self) -> str:
        width = max([len(r.name) for r in self.results] + [4])
        lines = [f"===== HAL anomaly tests ({self.cfg['target']}, suite '{self.hal.suite}') ====="]
        lines += [f"  {r.outcome:4}  {r.name:<{width}}  {r.detail}" for r in self.results]
        counts = {k: sum(r.outcome == k for r in self.results) for k in ("PASS", "FAIL", "SKIP")}
        lines.append(f"----- {counts['PASS']} passed, {counts['FAIL']} failed, {counts['SKIP']} skipped -----")
        if counts["FAIL"] and self._events_received == 0:
            lines.append("no event received from the HAL in the whole session: check the logging "
                         "discovery server and hal_components")
        if counts["FAIL"] == 0 and counts["PASS"] > 0:
            lines.append("ALL TESTS PASSED" if counts["SKIP"] == 0 else "ALL RUN TESTS PASSED (some skipped)")
        return "\n".join(lines)

    # Events --------------------------------------------------------------------

    def _background(self) -> set[int]:
        """Events that do not disturb the next test: always-on ones, and once the
        HAL is latched, every event the next test does not look for."""
        background = {self.ids[name] for name in STATIC_EVENTS.get(self.hal.suite, ())}
        if self._latched != "NOMINAL" and self._spec is not None:
            wanted = {self.ids[name] for name in self._spec.expected}
            if self._latched == "EMERGENCY_STOP":
                wanted.discard(self.ids["EMERGENCY_STOP_PRESSED"])  # reported for as long as it is latched
            background |= set(self.ids.values()) - wanted
        return background

    def _take_events(self, obs: Observation) -> None:
        safety = self.ids["SAFETY_STATE_NOT_NOMINAL"]
        for event in obs.events:
            if event.component not in self.cfg["hal_components"]:
                continue
            self._events_received += 1
            if event.event_id == safety:
                state = state_in(event.msg)
                if STATE_RANK[state] > STATE_RANK[self._hal_state]:
                    self._hal_state = state
            background = self._background()
            if self._spec is not None and self._phase == "run":
                self._seen.append(event)
            elif self._phase == "clean" and event.event_id not in background:
                self._clean_events.append(event)
            elif event.event_id == safety and self.busy and self._phase != "wait_hal" \
                    and self._unexpected_safety is None \
                    and STATE_RANK[state_in(event.msg)] > STATE_RANK[self._latched]:
                # Between tests: the HAL latched without a latching test running.
                self._unexpected_safety = event
            if event.event_id not in background:
                self._last_event_at = event.time

    def _seen_ids(self) -> set[int]:
        return {e.event_id for e in self._seen}

    def _describe(self, events: list[Event]) -> str:
        return ", ".join(sorted({self.names.get(e.event_id, str(e.event_id)) for e in events})) or "-"

    # State machine -------------------------------------------------------------

    def _set_phase(self, phase: str, now: float) -> None:
        self._phase = phase
        self._phase_start = now

    def _elapsed(self, now: float) -> float:
        return now - self._phase_start

    def _test_joint_range(self) -> tuple[float, float]:
        return self.hal.position_min[self.j], self.hal.position_max[self.j]

    def _center(self) -> float:
        low, high = self._test_joint_range()
        if abs(low) > 1e6 or abs(high) > 1e6:
            return self.hold[self.j]
        return 0.5 * (low + high)

    def _advance(self, obs: Observation) -> None:
        if self._unexpected_safety is not None:
            message = self._unexpected_safety.msg.strip()
            if self._phase == "recover" and self.results and self.results[-1].outcome == "PASS":
                last = self.results[-1]
                last.outcome = "FAIL"
                last.detail += f", but then the safety state changed: {message}"
                self.log(f"FAIL {last.name}: {last.detail}")
            self._unexpected_safety = None
            self._abort(f"the HAL left the nominal safety state ({message}): restart it")
            return
        if self._phase == "wait_hal":
            if obs.feedback is None:
                return
            mode = obs.feedback["control_mode"]
            if self.hal.suite is None:
                self._abort("the HAL config is the production one: run `hal_test_config enable <suite>` "
                            "and restart the HAL")
                return
            if mode != self.hal.mode:
                self._abort(f"the HAL runs control mode {CONTROL_MODES.get(mode)} but the installed config "
                            f"says {CONTROL_MODES.get(self.hal.mode)}: restart the HAL after hal_test_config")
                return
            low, high = self._test_joint_range()
            outside = max(low - self.hold[self.j], self.hold[self.j] - high, 0.0)
            if self.hal.stress_time_scale > 0.0 and outside / self.cfg["ramp_speed"] > 0.5 * self.hal.stress_time_scale:
                # Ramping it back would keep it outside its limits long enough to latch DAMPING.
                self._abort(f"test joint {self.j} at {self.hold[self.j]:.3f} rad, too far outside its test "
                            f"range [{low}, {high}]: move it inside before starting")
                return
            self.log(f"HAL found: control mode {CONTROL_MODES.get(mode)}, test joint {self.j}")
            self.goal = self._center()
            if self.cfg["freeze_base_owner"]:
                self.log("freezing the simulated base: legs hanging, like the suspended robot")
                self._pending_call = (self.cfg["freeze_base_owner"], "freezeBase")
            self._set_phase("settle", obs.now)
        elif self._phase == "settle":
            settle = self.cfg["settle_time"] + (self.cfg["freeze_settle_time"] if self.cfg["freeze_base_owner"] else 0.0)
            if self._elapsed(obs.now) >= settle and self._at_goal():
                self._next_test(obs)
        elif self._phase == "clean":
            self._clean(obs)
        elif self._phase == "run":
            self._run(obs)
        elif self._phase == "recover":
            if self._elapsed(obs.now) >= self.cfg["recover_time"] and self._at_goal():
                self._next_test(obs)

    def _at_goal(self) -> bool:
        return abs(self.hold[self.j] - self.goal) < 1e-6

    def _next_test(self, obs: Observation) -> None:
        self._spec = None
        if not self.queue:
            self._finish()
            return
        spec = self.queue.pop(0)
        reason = self._skip_reason(spec)
        if reason:
            self._record(spec, "SKIP", reason, 0.0)
            self._next_test(obs)
            return
        self._spec = spec
        self._clean_events = []
        self._set_phase("clean", obs.now)

    def _skip_reason(self, spec: Spec) -> str:
        if spec is NOMINAL:
            return ""
        if self._latched != "NOMINAL" and not spec.latching:
            return f"the HAL is latched in {self._latched}: restart it"
        if spec.name == "estop" and self._latched == "EMERGENCY_STOP":
            return "the HAL is already in EMERGENCY_STOP"
        if spec.suite != self.hal.suite:
            return f"needs suite '{spec.suite}' (hal_test_config enable {spec.suite}), active '{self.hal.suite}'"
        target = self.cfg["target"]
        if target not in spec.targets:
            why = {"current_limit": "the simulation has no motor currents",
                   "estop": "no emergency console command in the simulation (invalid_*_command covers the emergency stop)",
                   "fungo": "no fungo button in the simulation"}
            return f"real robot only: {why.get(spec.name, '')}".rstrip(": ")
        if spec.name == "estop" and not self.cfg["estop_command_owner"]:
            return "estop_command_owner not configured"
        if target == "real" and spec.moves_joint:
            low, high = self._test_joint_range()
            if high - low > self.cfg["max_real_range"]:
                return f"test joint range {high - low:.2f} rad wider than max_real_range: wrong test config?"
            if spec.trigger == "torque_pulses" and self.hal.torque_max[self.j] > self.cfg["max_real_torque"]:
                return f"torque_max {self.hal.torque_max[self.j]} Nm above max_real_torque: wrong test config?"
        if spec.name == "mechanical_stress" and self.hal.stress_time_scale <= 0.0:
            return "mechanical stress detection disabled in the config (no exp coefficients)"
        return ""

    def _clean(self, obs: Observation) -> None:
        quiet = obs.now - self._last_event_at >= self.cfg["quiet_time"]
        if quiet and self._at_goal() and self._elapsed(obs.now) >= self.cfg["quiet_time"]:
            self._start_run(obs)
        elif self._elapsed(obs.now) >= self.cfg["clean_timeout"]:
            # The HAL keeps reporting an anomaly while holding: nothing else can be judged.
            events = self._clean_events
            example = f" (e.g. {events[-1].msg.strip()})" if events else ""
            reason = f"baseline not clean: the HAL keeps reporting {self._describe(events)}{example}"
            self._record(self._spec, "FAIL", reason, self._elapsed(obs.now))
            self._spec = None
            self._abort("not run: " + reason)

    def _start_run(self, obs: Observation) -> None:
        self._seen = []
        self._start_state = self._latched
        self._state_reached = False
        self._feedback_ok = False
        self._last_command_call = -math.inf
        self._pulse_index = -1
        self._reached = -math.inf
        self.log(f"running {self._spec.name}: {self._spec.description}")
        if self._spec.trigger == "operator":
            self.log(">>> OPERATOR: PRESS THE FUNGO (emergency) BUTTON NOW")
        self._set_phase("run", obs.now)

    def _timeout(self, spec: Spec) -> float:
        if spec.trigger == "operator":
            return self.cfg["manual_timeout"]
        if spec.slow:
            return self.cfg["slow_test_timeout"] + (self.hal.references_timeout if spec.trigger == "silence" else 0.0)
        return self.cfg["test_timeout"]

    def _run(self, obs: Observation) -> None:
        spec = self._spec
        elapsed = self._elapsed(obs.now)
        safety = self.ids["SAFETY_STATE_NOT_NOMINAL"]
        safety_events = [e for e in self._seen if e.event_id == safety]

        if spec is NOMINAL:
            static = {self.ids[n] for n in STATIC_EVENTS.get(self.hal.suite, ())}
            unexpected = [e for e in self._seen if e.event_id not in static]
            if unexpected:
                self._fail(f"HAL events while holding: {self._describe(unexpected)} ({unexpected[0].msg.strip()})", obs)
            elif elapsed >= self.cfg["nominal_time"]:
                self._pass(f"no HAL event in {self.cfg['nominal_time']:.1f} s", obs)
            return

        if not spec.latching and safety_events:
            self._fail(f"safety state not nominal: {safety_events[0].msg.strip()}", obs)
            return
        # With the HAL already latched at or beyond the final state of the test,
        # only the detection can be proven, not the transition.
        target = max(spec.final_state, self._start_state, key=STATE_RANK.get)
        transition = spec.latching and target != self._start_state
        if spec.latching and not self._state_reached:
            self._state_reached = not transition or any(state_in(e.msg) == target for e in safety_events)
            beyond = [e for e in safety_events if STATE_RANK[state_in(e.msg)] > STATE_RANK[target]]
            if beyond:
                self._fail(f"wrong safety state: {beyond[0].msg.strip()}, expected {target}", obs)
                return
        check = {"EMERGENCY_STOP": "zero", "DAMPING": "damping"}.get(target) if spec.latching else None
        if self._state_reached and check and not self._feedback_ok:
            self._feedback_ok = self._check_feedback(check, obs)

        expected = [n for n in spec.expected
                    if not (n == "EMERGENCY_STOP_PRESSED" and self._start_state == "EMERGENCY_STOP")]
        missing = [name for name in expected if self.ids[name] not in self._seen_ids()]
        state_ok = not spec.latching or self._state_reached
        feedback_ok = check is None or self._feedback_ok
        if not missing and state_ok and feedback_ok:
            if not spec.latching:
                extra = ""
            elif transition:
                extra = f" -> {target}"
            else:
                extra = f" (HAL already in {self._start_state}: detection only)"
            self._pass(f"{', '.join(expected)}{extra} after {elapsed:.2f} s", obs)
        elif elapsed >= self._timeout(spec):
            problems = []
            if missing:
                problems.append(f"not reported: {', '.join(missing)}")
            if not state_ok:
                problems.append(f"no {target} safety state")
            if state_ok and not feedback_ok:
                problems.append(f"commands feedback is not '{check}'")
            if spec.trigger in ("excursion", "hold_outside") and math.isfinite(self._reached):
                problems.append(f"test joint reached {self._reached:.3f} rad, limit {self._test_joint_range()[1]} rad")
            self._fail("; ".join(problems) + f" within {self._timeout(spec):.1f} s", obs)

    def _check_feedback(self, check: str, obs: Observation) -> bool:
        fb = obs.feedback
        if fb is None:
            return False
        if check == "zero":
            return all(abs(v) < 1e-9 for key in ("position", "velocity", "torque") for v in fb[key])
        if check == "damping":
            if obs.velocity is None:
                return False
            gain = self.cfg["damping_gain"]
            return all(abs(t + gain * v) <= 1.0 + 0.2 * abs(gain * v)
                       for t, v in zip(fb["torque"], obs.velocity))
        raise ValueError(f"unknown feedback check {check}")

    # Results -------------------------------------------------------------------

    def _pass(self, detail: str, obs: Observation) -> None:
        self._end("PASS", detail, obs)

    def _fail(self, detail: str, obs: Observation) -> None:
        self._end("FAIL", detail, obs)

    def _end(self, outcome: str, detail: str, obs: Observation) -> None:
        spec = self._spec
        credited = list(COVERS.get(spec.name, ())) if outcome == "PASS" else []
        if self._start_state == "EMERGENCY_STOP" and "emergency_stop_triggered" in credited:
            credited.remove("emergency_stop_triggered")  # an earlier test triggered it
        self._record(spec, outcome, detail, self._elapsed(obs.now), credited)
        self.goal = self._center()
        if spec.latching:
            self._latched = max(self._latched, self._hal_state, key=STATE_RANK.get)
            if self.cfg["strict_latching"]:
                self._spec = None
                for rest in self.queue:
                    self._record(rest, "SKIP", "not run after a latching test (strict_latching)", 0.0)
                self.queue = []
                self._finish()
                return
        self._spec = None
        self._set_phase("recover", obs.now)

    def _record(self, spec: Spec, outcome: str, detail: str, duration: float,
                anomalies: list[str] | None = None) -> None:
        self.results.append(Result(spec.name, outcome, detail, duration, anomalies or []))
        self.log(f"{outcome} {spec.name}: {detail}")

    def _abort(self, reason: str) -> None:
        self.log("aborting: " + reason)
        pending = ([self._spec] if self._spec is not None else []) + self.queue
        for spec in pending:
            self._record(spec, "FAIL" if spec is NOMINAL else "SKIP", reason, 0.0)
        self.queue = []
        self._finish()

    def _finish(self) -> None:
        self._spec = None
        self._phase = "done"
        self.finished = True
        self.log(self.summary())
        if STATE_RANK[self._hal_state] > 0:
            self.log(f"the HAL is in {self._hal_state}: restart it for the next session")

    # Output --------------------------------------------------------------------

    def _output(self, obs: Observation) -> Output:
        if self.hold is None:
            n = len(obs.position) if obs.position else 0
            zeros = [0.0] * n
            # No state yet: a limp reference (zero torque) lets the HAL activate.
            return Output(True, zeros, zeros, zeros, zeros, zeros)
        n = self.n
        out = Output(True, list(self.hold), [0.0] * n, [0.0] * n,
                     [self.cfg["hold_kp"]] * n, [self.cfg["hold_kd"]] * n)
        if self._spec is not None and self._phase == "run":
            self._drive(self._spec, self._elapsed(obs.now), obs, out)
        if self._pending_call is not None:
            out.command_call, self._pending_call = self._pending_call, None
        return out

    def _drive(self, spec: Spec, t: float, obs: Observation, out: Output) -> None:
        """Applies the trigger of the running test to this tick's output."""
        j, k = self.j, (self.j + 1) % self.n
        low, high = self._test_joint_range()
        trigger = spec.trigger
        if trigger in ("excursion", "hold_outside"):
            # Push the reference until the measured position is past the limit
            # (friction and gravity leave a tracking error), then hold there.
            target = high + (0.05 if trigger == "excursion" else 0.1)
            q = obs.position[j] if obs.position is not None else None
            if q is not None and math.isfinite(q):
                self._reached = max(self._reached, q)
            if q is not None and q >= target:
                self.goal = self.hold[j]
            else:
                self.goal = high + self.cfg["max_overshoot"]
        elif trigger == "sine":
            center = self._center()
            amplitude = 0.3 * (high - low)
            omega = 2.5 * self.hal.velocity_max[j] / amplitude
            out.position[j] = center + amplitude * math.sin(omega * t)
            out.velocity[j] = amplitude * omega * math.cos(omega * t)
        elif trigger in ("torque_pulses", "velocity_pulses"):
            # One tick every PULSE_PERIOD, alternating sign. The HAL clamps it
            # to the limit, so the kick is bounded by the test config.
            index = int(t / PULSE_PERIOD)
            if index != self._pulse_index:
                self._pulse_index = index
                sign = 1.0 if index % 2 == 0 else -1.0
                if trigger == "torque_pulses":
                    out.torque[j] = sign * 2.0 * self.hal.torque_max[j]
                else:
                    out.velocity[j] = sign * 2.0 * self.hal.velocity_max[j]
        elif trigger == "position_beyond":
            out.position[j] = high + 0.1
        elif trigger == "invalid_torque":
            out.torque[j], out.torque[k] = math.nan, math.inf
        elif trigger == "invalid_position":
            out.position[j], out.position[k] = math.nan, math.inf
        elif trigger == "invalid_velocity":
            out.velocity[j], out.velocity[k] = math.nan, math.inf
        elif trigger == "silence":
            out.publish_reference = False
        elif trigger == "estop_command":
            if obs.now - self._last_command_call >= 1.0:
                self._last_command_call = obs.now
                out.command_call = (self.cfg["estop_command_owner"], "emergency")
        elif trigger in ("none", "operator"):
            pass
        else:
            raise ValueError(f"unknown trigger {trigger}")
