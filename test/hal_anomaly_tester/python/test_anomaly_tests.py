"""Unit tests of the anomaly test runner against a fake HAL (no DDS).

Run with:  python3 -B -m unittest test_anomaly_tests -v   (from this directory)
"""

import math
import os
import unittest

import simple_yaml
import re

from anomaly_tests import SPECS as SPECS_BY_NAME
from anomaly_tests import (
    ALL_ANOMALIES, CATALOG, COVERS, FALLBACK_EVENT_IDS, NOT_TESTABLE, Event, HalConfig, Observation, Runner,
    coverage_report, parse_event_ids, select_tests,
)

N = 12
DT = 0.01  # tester tick, s
IDS = FALLBACK_EVENT_IDS
HERE = os.path.dirname(os.path.abspath(__file__))
EVENT_HEADER = os.path.join(HERE, "../../../../dls2/modules/log/include/dls2/log/event_config.hpp")
INVARIANTS_HEADER = os.path.join(HERE, "../../../robot_interface/include/dls_hal/robot_invariants.hpp")


def hal_config(suite="base", mode=1, test_range=0.25, torque_max=15.0, stress_a=2.0):
    """Like the merged test configs of pegasus-description."""
    position_min = [-test_range] + [-3.0] * (N - 1)
    position_max = [test_range] + [3.0] * (N - 1)
    return HalConfig(suite=suite, mode=mode, references_timeout=1.0,
                     position_min=position_min, position_max=position_max,
                     velocity_max=[0.8] + [10.0] * (N - 1),
                     torque_max=[torque_max] + [120.0] * (N - 1), stress_time_scale=stress_a)


class FakeHal:
    """Reacts to the tester outputs like the HAL does, and reports through events."""

    def __init__(self, cfg: HalConfig, real=False, broken=(), component="gazebo_sim", dirty=None, lag=0.0):
        self.cfg = cfg
        self.lag = lag              # friction: the test joint stops this short of its reference
        self.dirty = dirty          # event reported on every cycle, e.g. a clamped hold torque
        self.command_calls = []
        self.real = real
        self.broken = set(broken)
        self.component = component
        self.state = "NOMINAL"
        self.estop = False
        self.last_reference = 0.0
        self.outside_since = None
        self.last_logged = {}
        self.pending = []
        self.position = None
        self.fungo_at = None
        self.started = False

    def _log(self, now, name, msg=""):
        if name in self.broken:
            return
        if now - self.last_logged.get(name, -math.inf) < 0.25:  # AppLogger throttling
            return
        self.last_logged[name] = now
        self.pending.append(Event(now, self.component, IDS[name], 2, msg or name))

    def step(self, now, out):
        cfg = self.cfg
        if not self.started:
            self.started = True
            self._log(now, "GENERAL_WARNING", "startup event the tester must wait out")
        if out.publish_reference:
            self.last_reference = now
            ref_position, ref_velocity, ff = out.position, out.velocity, out.torque
        else:
            ref_position, ref_velocity, ff = self.position or out.position, [0.0] * N, [0.0] * N
        if self.state != "NOMINAL" and self.position is not None:
            position = list(self.position)  # recovery overrides the references: the joint stays put
        else:
            position = [p if math.isfinite(p) else 0.0 for p in ref_position]  # perfect tracking...
            position[0] -= math.copysign(min(self.lag, abs(position[0])), position[0])  # ...but for friction
        velocity = [v if math.isfinite(v) else 0.0 for v in ref_velocity]
        self.position = position

        if position[0] < cfg.position_min[0] or position[0] > cfg.position_max[0]:
            self._log(now, "JOINT_POS_OUT_OF_BOUND")
        if abs(velocity[0]) > cfg.velocity_max[0]:
            self._log(now, "JOINT_VEL_OUT_OF_BOUND")

        if cfg.mode == 1:
            commands = ff
            limit = cfg.torque_max
            clamp_event = "JOINT_TORQUE_CLAMPING_ANOMALY"
        elif cfg.mode == 2:
            commands = ref_position
            limit = cfg.position_max
            clamp_event = "JOINT_POS_CLAMPING_ANOMALY"
        else:
            commands = ref_velocity
            limit = cfg.velocity_max
            clamp_event = "JOINT_VEL_CLAMPING_ANOMALY"
        applied = []
        for i, c in enumerate(commands):
            if math.isnan(c):
                self._log(now, "NAN_VALUE")
                self.estop = True
            elif math.isinf(c):
                self._log(now, "INF_VALUE")
                self.estop = True
            elif c > limit[i] or (cfg.mode != 2 and c < -limit[i]):
                self._log(now, clamp_event)
                c = math.copysign(limit[i], c)
            applied.append(c if math.isfinite(c) else 0.0)
        if self.real and cfg.mode == 1 and abs(applied[0]) / (15.0 / 3.6) > 2.0:
            self._log(now, "JOINT_CURRENT_OUT_OF_BOUND")

        if cfg.suite == "base":
            for name in ("MOTOR_OVERHEAT", "MOTOR_UNDER_TEMPERATURE", "BATTERY_CHARGE_LOW"):
                self._log(now, name)
        if self.dirty:
            self._log(now, self.dirty, f"{self.dirty} while holding")
        if out.command_call is not None:
            self.command_calls.append(out.command_call)
        if out.command_call == ("pegasus_hal", "emergency"):
            self.estop = True
        if self.fungo_at is not None and now >= self.fungo_at:
            self._log(now, "FUNGO_PRESSED")
            self.state = "EMERGENCY_STOP"

        lost = now - self.last_reference > cfg.references_timeout
        if lost:
            self._log(now, "MISSING_INPUT")
        measured_torque = 2.0  # gravity load on the test joint
        if position[0] > cfg.position_max[0] and abs(measured_torque) > 0.5 and cfg.stress_time_scale > 0:
            self.outside_since = self.outside_since if self.outside_since is not None else now
            if now - self.outside_since > cfg.stress_time_scale:
                self._log(now, "BREAKING_MECHANICS")
                if "BREAKING_MECHANICS" not in self.broken and self.state == "NOMINAL":
                    self.state = "DAMPING"
        else:
            self.outside_since = None

        if self.estop:
            self._log(now, "EMERGENCY_STOP_PRESSED")
            self.state = "EMERGENCY_STOP"
        elif lost and self.state == "NOMINAL":
            self.state = "DAMPING"
        if self.state != "NOMINAL":
            self._log(now, "SAFETY_STATE_NOT_NOMINAL", f"Safety state is not nominal: {self.state} mode is active.")
            applied = [0.0] * N  # zero velocity: damping torque is zero too

        events, self.pending = self.pending, []
        feedback = {"control_mode": cfg.mode, "position": applied if cfg.mode == 2 else [0.0] * N,
                    "velocity": applied if cfg.mode == 3 else [0.0] * N,
                    "torque": applied if cfg.mode == 1 else [0.0] * N}
        if self.state != "NOMINAL":
            feedback.update(position=[0.0] * N, velocity=[0.0] * N, torque=[0.0] * N)
        return position, velocity, feedback, events


def run(tests, cfg=None, config=None, hal=None, max_time=200.0):
    cfg = cfg or hal_config()
    hal = hal or FakeHal(cfg, real=(config or {}).get("target") == "real")
    logs = []
    runner = Runner({"target": "sim", "settle_time": 0.2, "nominal_time": 0.5, "recover_time": 0.2,
                     **(config or {})}, cfg, log=logs.append)
    assert runner.start(tests), logs
    now, position, velocity, feedback, events = 0.0, [0.1] * N, [0.0] * N, None, []
    while not runner.finished and now < max_time:
        out = runner.step(Observation(now, position, velocity, feedback, events))
        if hal.fungo_at is None and any("PRESS THE FUNGO" in line for line in logs):
            hal.fungo_at = now + 2.0  # the operator reacts
        now += DT
        position, velocity, feedback, events = hal.step(now, out)
    assert runner.finished, "runner did not finish:\n" + "\n".join(logs)
    return {r.name: r for r in runner.results}, runner, logs


REAL = {"target": "real", "estop_command_owner": "pegasus_hal"}


class SelectionTest(unittest.TestCase):
    def test_all_is_every_test_of_the_suite_latching_last_in_safe_order(self):
        names = [s.name for s in select_tests(["all"], "base")]
        self.assertEqual(names, ["nominal", "position_limit", "velocity_limit", "torque_clamping", "current_limit",
                                 "overheat", "under_temperature", "battery_low",
                                 "mechanical_stress", "lost_references", "estop", "invalid_torque_command", "fungo"])

    def test_latching_tests_are_ordered_whatever_the_request(self):
        names = [s.name for s in select_tests(["fungo", "lost_references", "position_limit", "mechanical_stress"], "base")]
        self.assertEqual(names, ["nominal", "position_limit", "mechanical_stress", "lost_references", "fungo"])

    def test_strict_latching(self):
        names = [s.name for s in select_tests(["all"], "base", strict=True)]
        self.assertFalse(any(SPECS_BY_NAME[n].latching for n in names[1:]))
        with self.assertRaises(ValueError):
            select_tests(["estop", "fungo"], "base", strict=True)
        with self.assertRaises(ValueError):
            select_tests(["no_such_test"], "base")

    def test_every_suite_has_a_config(self):
        test_dir = os.path.join(HERE, "../../../../../pegasus-description/interface/config/test")
        if not os.path.isdir(test_dir):
            self.skipTest("pegasus-description not found")
        for suite in {s.suite for s in CATALOG}:
            path = os.path.join(test_dir, f"interface_config_test_{suite}.yaml")
            self.assertEqual(simple_yaml.load_file(path)["test_config"], suite, path)


class BaseSuiteTest(unittest.TestCase):
    def test_full_base_session_in_simulation(self):
        results, runner, logs = run(["all"])
        outcomes = {n: r.outcome for n, r in results.items()}
        expected = {n: "PASS" for n in outcomes}
        expected.update(current_limit="SKIP", estop="SKIP", fungo="SKIP")  # real robot only
        self.assertEqual(outcomes, expected, "\n".join(logs))
        self.assertIn("-> DAMPING", results["mechanical_stress"].detail)
        self.assertIn("HAL already in DAMPING: detection only", results["lost_references"].detail)
        self.assertIn("-> EMERGENCY_STOP", results["invalid_torque_command"].detail)
        self.assertIn("emergency_stop_triggered", results["invalid_torque_command"].anomalies)
        self.assertIn("ALL RUN TESTS PASSED", runner.summary())
        self.assertTrue(any("restart it for the next session" in line for line in logs))

    def test_full_base_session_on_the_real_robot(self):
        results, runner, logs = run(["all"], config=REAL)
        self.assertEqual({r.outcome for r in results.values()}, {"PASS"}, "\n".join(logs))
        self.assertIn("-> EMERGENCY_STOP", results["estop"].detail)
        # The E-STOP is credited to the test that caused it only.
        self.assertIn("emergency_stop_triggered", results["estop"].anomalies)
        self.assertNotIn("emergency_stop_triggered", results["invalid_torque_command"].anomalies)
        self.assertIn("HAL already in EMERGENCY_STOP: detection only", results["fungo"].detail)
        self.assertEqual(len(runner.verified_anomalies()), 13)  # + position_mode and velocity_mode sessions: 19

    def test_current_limit_passes_on_the_real_robot(self):
        results, _, logs = run(["current_limit"], config=REAL)
        self.assertEqual(results["current_limit"].outcome, "PASS", logs)

    def test_each_latching_test_passes_in_its_own_strict_session(self):
        for name, config in [("invalid_torque_command", {}), ("lost_references", {}),
                             ("mechanical_stress", {}), ("estop", REAL)]:
            with self.subTest(name):
                results, runner, logs = run(["all", name], config={**config, "strict_latching": True})
                self.assertEqual(results[name].outcome, "PASS", "\n".join(logs))
                self.assertIn(" -> ", results[name].detail, "strict: the transition is proven")

    def test_fungo_waits_for_the_operator(self):
        cfg = hal_config()
        hal = FakeHal(cfg, real=True)
        hal.fungo_at = 8.0
        results, _, logs = run(["fungo"], cfg=cfg, config=REAL, hal=hal)
        self.assertEqual(results["fungo"].outcome, "PASS", logs)
        self.assertTrue(any("PRESS THE FUNGO" in line for line in logs))

    def test_real_robot_only_tests_skip_in_simulation(self):
        results, _, _ = run(["estop"])
        self.assertEqual(results["estop"].outcome, "SKIP")
        self.assertIn("real robot only", results["estop"].detail)

    def test_undetected_anomaly_fails(self):
        cfg = hal_config()
        results, _, _ = run(["velocity_limit"], cfg=cfg, hal=FakeHal(cfg, broken={"JOINT_VEL_OUT_OF_BOUND"}))
        self.assertEqual(results["velocity_limit"].outcome, "FAIL")
        self.assertIn("not reported: JOINT_VEL_OUT_OF_BOUND", results["velocity_limit"].detail)

    def test_missing_breaking_mechanics_fails(self):
        cfg = hal_config()
        results, _, _ = run(["mechanical_stress"], cfg=cfg, hal=FakeHal(cfg, broken={"BREAKING_MECHANICS"}))
        self.assertEqual(results["mechanical_stress"].outcome, "FAIL")

    def test_no_events_at_all_is_diagnosed(self):
        cfg = hal_config()
        results, runner, _ = run(["velocity_limit"], cfg=cfg, hal=FakeHal(cfg, component="someone_else"))
        self.assertEqual(results["velocity_limit"].outcome, "FAIL")
        self.assertIn("no event received from the HAL in the whole session", runner.summary())

    def test_non_latching_test_fails_if_the_safety_state_changes(self):
        # A HAL whose mechanical stress window is tiny: the position excursion latches DAMPING.
        cfg = hal_config(stress_a=0.01)
        results, _, _ = run(["position_limit", "velocity_limit"], cfg=cfg)
        self.assertEqual(results["position_limit"].outcome, "FAIL")
        self.assertIn("safety state", results["position_limit"].detail)
        self.assertIn("DAMPING", results["position_limit"].detail)
        self.assertEqual(results["velocity_limit"].outcome, "SKIP")
        self.assertIn("restart it", results["velocity_limit"].detail)

    def test_position_tests_push_through_friction(self):
        for name in ("position_limit", "mechanical_stress"):
            with self.subTest(name):
                cfg = hal_config()
                results, _, logs = run([name], cfg=cfg, hal=FakeHal(cfg, lag=0.07))
                self.assertEqual(results[name].outcome, "PASS", "\n".join(logs))

    def test_position_test_reports_how_far_the_joint_got(self):
        cfg = hal_config()
        results, _, _ = run(["position_limit"], cfg=cfg, hal=FakeHal(cfg, lag=0.5))
        self.assertEqual(results["position_limit"].outcome, "FAIL")
        self.assertIn("test joint reached 0.050 rad, limit 0.25 rad", results["position_limit"].detail)

    def test_dirty_baseline_names_the_event_and_stops_the_session(self):
        cfg = hal_config()
        results, _, _ = run(["all", "mechanical_stress"], cfg=cfg, hal=FakeHal(cfg, dirty="JOINT_TORQUE_CLAMPING_ANOMALY"))
        self.assertEqual(results["nominal"].outcome, "FAIL")
        self.assertIn("JOINT_TORQUE_CLAMPING_ANOMALY", results["nominal"].detail)
        self.assertIn("while holding", results["nominal"].detail)
        others = {n: r.outcome for n, r in results.items() if n != "nominal"}
        self.assertEqual(set(others.values()), {"SKIP"}, others)

    def test_simulated_base_is_frozen_once_at_start(self):
        cfg = hal_config()
        hal = FakeHal(cfg)
        results, _, logs = run(["position_limit"], cfg=cfg, config={"freeze_base_owner": "gazebo_sim"}, hal=hal)
        self.assertEqual(hal.command_calls, [("gazebo_sim", "freezeBase")])
        self.assertEqual(results["position_limit"].outcome, "PASS", logs)

    def test_test_joint_far_outside_its_range_aborts(self):
        logs = []
        runner = Runner({"target": "sim"}, hal_config(), log=logs.append)
        runner.start(["all"])
        feedback = {"control_mode": 1, "position": [0.0] * N, "velocity": [0.0] * N, "torque": [0.0] * N}
        runner.step(Observation(0.0, [1.5] + [0.0] * (N - 1), [0.0] * N, feedback))
        self.assertTrue(runner.finished)
        self.assertIn("move it inside", runner.results[0].detail)

    def test_wide_envelope_is_refused_on_the_real_robot(self):
        results, _, _ = run(["position_limit", "torque_clamping"], cfg=hal_config(test_range=2.0, torque_max=120.0),
                            config=REAL)
        self.assertEqual(results["position_limit"].outcome, "SKIP")
        self.assertEqual(results["torque_clamping"].outcome, "SKIP")


class OtherSuitesTest(unittest.TestCase):
    def test_static_thresholds(self):
        results, _, logs = run(["overheat", "under_temperature", "battery_low"])
        self.assertEqual({n: r.outcome for n, r in results.items()},
                         {"nominal": "PASS", "overheat": "PASS", "under_temperature": "PASS", "battery_low": "PASS"}, logs)

    def test_position_mode_suite(self):
        cfg = hal_config(suite="position_mode", mode=2)
        results, _, logs = run(["all", "invalid_position_command"], cfg=cfg)
        self.assertEqual(results["position_clamping"].outcome, "PASS", logs)
        self.assertEqual(results["invalid_position_command"].outcome, "PASS", logs)

    def test_velocity_mode_suite(self):
        cfg = hal_config(suite="velocity_mode", mode=3)
        results, _, logs = run(["all", "invalid_velocity_command"], cfg=cfg)
        self.assertEqual(results["velocity_clamping"].outcome, "PASS", logs)
        self.assertEqual(results["invalid_velocity_command"].outcome, "PASS", logs)

    def test_test_of_another_suite_is_skipped_with_a_hint(self):
        results, _, _ = run(["position_clamping"])
        self.assertEqual(results["position_clamping"].outcome, "SKIP")
        self.assertIn("hal_test_config enable position_mode", results["position_clamping"].detail)

    def test_production_config_aborts(self):
        results, _, _ = run(["all"], cfg=hal_config(suite=None))
        self.assertEqual(results["nominal"].outcome, "FAIL")
        self.assertIn("hal_test_config enable", results["nominal"].detail)

    def test_config_not_loaded_by_the_hal_aborts(self):
        cfg = hal_config()
        results, _, _ = run(["all"], cfg=cfg, hal=FakeHal(hal_config(mode=2)))
        self.assertIn("restart the HAL", results["nominal"].detail)


class CoverageTest(unittest.TestCase):
    def test_anomaly_list_matches_robot_anomalies(self):
        if not os.path.isfile(INVARIANTS_HEADER):
            self.skipTest("dls_hal sources not found")
        with open(INVARIANTS_HEADER, encoding="utf-8") as file:
            body = re.search(r"class RobotAnomalies\{(.*?)\n\};", file.read(), re.S).group(1)
        flags = re.findall(r"^\s*(?:std::vector<bool>|RelaxedAtomicBool|bool)\s+(\w+)", body, re.M)
        self.assertEqual(tuple(flags), ALL_ANOMALIES)

    def test_every_anomaly_is_tested_or_explained(self):
        covered = {a for anomalies in COVERS.values() for a in anomalies}
        self.assertEqual(covered | set(NOT_TESTABLE), set(ALL_ANOMALIES))
        self.assertEqual(covered & set(NOT_TESTABLE), set())
        self.assertEqual(set(COVERS), {s.name for s in CATALOG})

    def test_session_coverage_lists_verified_and_missing_anomalies(self):
        results, runner, logs = run(["all"])
        report = runner.coverage()
        self.assertIn("Anomalies verified (this session): 11/26", report)  # one base session in sim
        self.assertRegex(report, r"\[x\] joint_pos_limit_violation +by position_limit, mechanical_stress")
        self.assertRegex(report, r"\[x\] emergency_stop_triggered +by invalid_torque_command")
        # Only the verified anomalies are listed.
        for name in ("joint_current_limit_violation", "fungo_pressed", "nan_position_detected"):
            self.assertNotIn(name, report)
        self.assertEqual(report.count("[x]"), 11)
        self.assertEqual(len(report.splitlines()), 1 + 11)
        self.assertFalse(any("Anomalies verified" in line for line in logs), "printed once, by the app")
        merged = runner.coverage({"joint_pos_clamping_anomaly"}, "all sim sessions")
        self.assertIn("(all sim sessions): 12/26", merged)
        self.assertRegex(merged, r"\[x\] joint_pos_clamping_anomaly +earlier session")
        self.assertRegex(merged, r"\[x\] breaking_mechanics +by mechanical_stress")

    def test_cumulative_report(self):
        report = coverage_report({"battery_charge_low", "fungo_pressed"}, "all sessions so far, real")
        self.assertIn("(all sessions so far, real): 2/26", report)


class SupportTest(unittest.TestCase):
    def test_fallback_event_ids_match_the_dls2_header(self):
        if not os.path.isfile(EVENT_HEADER):
            self.skipTest("dls2 sources not found")
        with open(EVENT_HEADER, encoding="utf-8") as file:
            parsed = parse_event_ids(file.read())
        self.assertEqual(parsed, FALLBACK_EVENT_IDS)

    def test_hal_config_from_merged_yaml(self):
        text = """
general: {n_joints: 2, n_contacts: 4}
test_config: base
robot_interface:
  references_timeout: 1.0   # s
  safety:
    limits:
      position_min: [-0.25, -2.0]
      position_max: [0.25, 2.0]
      velocity_max: [0.8, 10.0]
      torque_max: [15.0, 120.0]
    mechanical_stress_exp_coefficients: {a: 2.0, b: 0.0}
  low_level_control:
    mode: 1
"""
        simple_yaml._pyyaml, saved = None, simple_yaml._pyyaml
        try:
            cfg = HalConfig.from_dict(simple_yaml.loads(text))
        finally:
            simple_yaml._pyyaml = saved
        self.assertEqual((cfg.suite, cfg.mode, cfg.position_max, cfg.stress_time_scale), ("base", 1, [0.25, 2.0], 2.0))


if __name__ == "__main__":
    unittest.main()
