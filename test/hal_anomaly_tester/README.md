# HAL anomaly tester

End-to-end check that a running HAL detects the anomalies of
[robot_invariants.hpp](../../robot_interface/include/dls_hal/robot_invariants.hpp)
and reacts with the right safety state, in simulation (`gazebo_sim`) and on the
real robot (`pegasus_hal`). **The HAL is used as it is:** everything happens from outside it.

## How it works

```
             rt/control_signal (references)            ┌────────────────────────────┐
hal_anomaly ─────────────────────────────────────────▶ │ HAL                        │
  _tester    rt/command_call ("emergency", real only)  │ running a TEST config      │
     ▲   ─────────────────────────────────────────────▶ │ (hal_test_config)          │
     │                                                  └──────────────┬─────────────┘
     └──── rt/log_events (HAL events) + rt/hal_commands_feedback ──────┘
```

- **Test config.** `hal_test_config enable <suite>` installs a test version of `interface_config.yaml`: the production file plus a few overrides. The HAL then finds anomalies in values that would be normal in production:
  - a narrow envelope on one *test joint*;
  - impossible temperature or battery thresholds;
  - a different control mode.
- **The tester replaces the control layer.** It holds the posture and provokes each anomaly with its references: motions, torque pulses, NaN/Inf references, or stopping the references. On the real robot it can also call the HAL `emergency` command.
- **Pass/fail.** A test passes when the HAL publishes the expected events on `rt/log_events`. Latching tests must also reach the expected safety state, unless the HAL is already latched in it (then only detection is proven), and the HAL must send the matching commands: all zero, or damping.

## Suites (test configs)

Override files live in [pegasus-description/interface/config/test](../../../../pegasus-description/interface/config/test). In an override, `{index: value}` patches one entry of a list.

| Suite | Overrides | Tests |
|---|---|---|
| `base` | joint 0 (LF_HAA): position ±0.25 rad, velocity 0.8 rad/s, torque 15 Nm, current 2 A; mechanical stress 0.5 Nm for 2 s; `references_timeout` 1 s. Joint 1 always overheating, joint 2 always under temperature, battery always low | `position_limit`, `velocity_limit`, `torque_clamping`, `current_limit`, `overheat`, `under_temperature`, `battery_low`, `mechanical_stress`, `lost_references`, `estop`, `invalid_torque_command`, `fungo` |
| `position_mode` | POSITION_CONTROL, joint 0 position ±0.25 rad | `position_clamping`, `invalid_position_command` |
| `velocity_mode` | VELOCITY_CONTROL, joint 0 velocity 0.8 rad/s | `velocity_clamping`, `invalid_velocity_command` |

## Tests

| Test | Expected HAL events | Trigger | Latching | Where |
|---|---|---|---|---|
| `nominal` (always first) | none | hold the posture | | both |
| `position_limit` | JOINT_POS_OUT_OF_BOUND | push the test joint until it is measured 0.05 rad past its limit (reference up to `max_overshoot` beyond, for friction), then back | | both |
| `velocity_limit` | JOINT_VEL_OUT_OF_BOUND | oscillate the test joint at 2.5× its velocity limit | | both |
| `torque_clamping` | JOINT_TORQUE_CLAMPING_ANOMALY | single-tick torque pulses at 2× `torque_max` | | both |
| `current_limit` | JOINT_CURRENT_OUT_OF_BOUND | same pulses | | real |
| `overheat`, `under_temperature`, `battery_low` | MOTOR_OVERHEAT, MOTOR_UNDER_TEMPERATURE, BATTERY_CHARGE_LOW | impossible thresholds of the `base` suite, active the whole session | | both |
| `position_clamping` / `velocity_clamping` | JOINT_POS / JOINT_VEL_CLAMPING_ANOMALY | reference past the limit | | both |
| `estop` | EMERGENCY_STOP_PRESSED → EMERGENCY_STOP, zero commands | `pegasus_hal::emergency` console command | ✓ | real |
| `fungo` | FUNGO_PRESSED → EMERGENCY_STOP, zero commands | the operator presses the button | ✓ | real |
| `invalid_torque/position/velocity_command` | NAN_VALUE, INF_VALUE, EMERGENCY_STOP_PRESSED → EMERGENCY_STOP | NaN and Inf references | ✓ | both |
| `lost_references` | MISSING_INPUT → DAMPING, damping commands | stop publishing references | ✓ | both |
| `mechanical_stress` | JOINT_POS_OUT_OF_BOUND, BREAKING_MECHANICS → DAMPING | push the test joint 0.1 rad past its limit (measured) and hold it there | ✓ | both |

## Anomalies not detected by these tests

`RobotAnomalies` has 26 anomaly flags. The tester can verify 19 of them on the real robot and 17 in simulation. The end-of-session list shows only the verified ones, counted out of 26.

**Never triggered by the tester (7).** Only the gtests in [robot_invariants](../robot_invariants) check these.

| Anomaly | Why |
|---|---|
| `nan_position_detected` | The measured state comes from the encoders or Gazebo, and a NaN/Inf *command* is zeroed by the E-STOP before it is sent. The events confirm this with `source=command`. |
| `nan_velocity_detected` | Same as above |
| `nan_torque_detected` | Same as above |
| `inf_position_detected` | Same as above |
| `inf_velocity_detected` | Same as above |
| `inf_torque_detected` | Same as above |
| `joint_torque_limit_violation` | On the robot the measured torque follows the clamped command, so it can at most equal `torque_max`, and the check is strictly "greater than". In Gazebo the measured torque comes from the joint's transmitted wrench, which includes contact forces, so it can exceed the limit when the robot stands with the base not frozen. There is no test for it yet. |

**Also not triggered in simulation (2).** These are covered on the real robot only.

| Anomaly | Why |
|---|---|
| `joint_current_limit_violation` | Gazebo reports no motor currents. |
| `fungo_pressed` | There is no fungo button, and the sim never sets the fungo status word. |

In simulation `emergency_stop_triggered` is still verified: the `estop` console command exists only on `pegasus_hal`, but the `invalid_*_command` tests also trigger the E-STOP.

## Sessions

A session is one test config plus one dls start. Full coverage takes **3 sessions**, run with `HAL_ANOMALY_TESTS=all`:

| # | `hal_test_config enable` | Where |
|---|---|---|
| 1 | `base` | both |
| 2 | `position_mode` | both¹ |
| 3 | `velocity_mode` | both¹ |

¹ Only if the LLI supports that control mode. Gazebo applies torques only, so the robot goes limp in those sessions.

**Latching tests share a session.** A latching test leaves the HAL in DAMPING or EMERGENCY_STOP until it restarts, but the HAL keeps checking every cycle while latched. So the latching tests run last, in an order where each one still works:

| Order | Test | HAL state | Proves |
|---|---|---|---|
| 1 | `mechanical_stress` | NOMINAL → DAMPING | detection and transition. Must be first: in DAMPING the HAL ignores position references, so the joint can no longer be pushed. |
| 2 | `lost_references` | DAMPING | detection |
| 3 | `estop` (robot) | DAMPING → EMERGENCY_STOP | detection and transition |
| 4 | `invalid_torque_command` | → EMERGENCY_STOP in sim, already there on the robot | detection, plus the transition in sim |
| 5 | `fungo` (robot) | EMERGENCY_STOP | detection |

Once latched, the tester waits before each test only until that test's own events are absent, so whatever it then sees was caused by the test. `emergency_stop_triggered` is credited only to the test that actually triggered the E-STOP.

**Strict mode.** Set `"strict_latching": true` in the JSON config to prove every transition end to end. Each session then runs at most one latching test, and `all` means the non-latching tests only. Full coverage then takes one `base` session per latching test.

## Run in simulation

Build and install as usual. That installs `hal_anomaly_tester`, `hal_test_config` and the test configs. Then, per session:

```sh
sudo hal_test_config enable base
export HAL_ANOMALY_TESTER_CONFIG=/usr/share/hal_anomaly_tester/config/hal_anomaly_tester_sim.json
export HAL_ANOMALY_TESTS=all
dls --startup=../startup_hal_test_sim.yaml   # from build/ (the file is in the repo root); stop it when the summary is printed
```

The sim config sets `freeze_base_owner: gazebo_sim`, so the tester calls `gazebo_sim::freezeBase` at start. The base is held 1 m up and the legs hang, as on the suspended robot. A standing robot loads LF_HAA beyond the 15 Nm test envelope and the baseline is never clean. Don't freeze the base yourself: the command toggles.

When you're done, run `sudo hal_test_config disable`.

## Run on the real robot

- Suspend the robot (legs free) and keep the operator at the emergency button.
- The default config is `target: real`. It refuses motions and pulses if the test joint envelope is wider than 1 rad or `torque_max` is above 20 Nm, which catches a wrong config.

```sh
sudo hal_test_config enable base
export HAL_ANOMALY_TESTS=all
dls --startup=../startup_hal_test_real.yaml  # from build/ (the file is in the repo root)
```

Then activate `pegasus_hal` from the console. It only activates once it receives `rt/control_signal`, which the tester is already publishing. Make sure joint 0 starts inside ±0.25 rad: the tester refuses to start otherwise.

The tester triggers the E-STOP itself (`pegasus_hal::emergency`). For the last test, `fungo`, it prints `>>> OPERATOR: PRESS THE FUNGO (emergency) BUTTON NOW` and waits up to 60 s.

**Always run `sudo hal_test_config disable` afterwards.** `hal_test_config status` shows which config is installed. Every generated file starts with `# GENERATED TEST CONFIG`.

## Results

At the end of a session the tester prints the test summary, then the anomalies verified so far. This example is a `base` session in simulation (recorded against the fake HAL of the unit tests):

```
===== HAL anomaly tests (sim, suite 'base') =====
  PASS  nominal                 no HAL event in 3.0 s
  PASS  position_limit          JOINT_POS_OUT_OF_BOUND after 0.52 s
  PASS  velocity_limit          JOINT_VEL_OUT_OF_BOUND after 0.01 s
  PASS  torque_clamping         JOINT_TORQUE_CLAMPING_ANOMALY after 0.01 s
  SKIP  current_limit           real robot only: the simulation has no motor currents
  PASS  overheat                MOTOR_OVERHEAT after 0.13 s
  PASS  under_temperature       MOTOR_UNDER_TEMPERATURE after 0.08 s
  PASS  battery_low             BATTERY_CHARGE_LOW after 0.08 s
  PASS  mechanical_stress       JOINT_POS_OUT_OF_BOUND, BREAKING_MECHANICS -> DAMPING after 2.52 s
  PASS  lost_references         MISSING_INPUT (HAL already in DAMPING: detection only) after 1.00 s
  SKIP  estop                   real robot only: no emergency console command in the simulation (...)
  PASS  invalid_torque_command  NAN_VALUE, INF_VALUE, EMERGENCY_STOP_PRESSED -> EMERGENCY_STOP after 0.08 s
  SKIP  fungo                   real robot only: no fungo button in the simulation
----- 10 passed, 0 failed, 3 skipped -----
ALL RUN TESTS PASSED (some skipped)
the HAL is in EMERGENCY_STOP: restart it for the next session
===== Anomalies verified (all sim sessions): 11/26 =====
  [x] joint_pos_limit_violation         by position_limit, mechanical_stress
  [x] joint_vel_limit_violation         by velocity_limit
  [x] joint_torque_clamping_anomaly     by torque_clamping
  [x] nan_torque_command_detected       by invalid_torque_command
  [x] inf_torque_command_detected       by invalid_torque_command
  [x] motor_overheat_detected           by overheat
  [x] motor_under_temperature_detected  by under_temperature
  [x] battery_charge_low                by battery_low
  [x] emergency_stop_triggered          by invalid_torque_command
  [x] lost_references                   by lost_references
  [x] breaking_mechanics                by mechanical_stress
  this session: 11/26; reset the total: delete /tmp/hal_anomaly_tester_coverage.json
```

- **Anomaly list.** It covers every session on the same target (`sim` or `real`): `earlier session` marks anomalies verified before this one. It lists only the verified anomalies; the ones it can never verify are in [Anomalies not detected by these tests](#anomalies-not-detected-by-these-tests). The total is kept in `coverage_file`; delete it to start over.
- **Report.** Each session is also saved to `report_file` (default `/tmp/hal_anomaly_tester_report.json`), with the results and an `anomalies_verified` list.

A FAIL says what was missing. For example:
- `not reported: JOINT_VEL_OUT_OF_BOUND within 3.0 s`
- `test joint reached 0.231 rad, limit 0.25 rad`: the joint could not be pushed past its limit.
- `safety state not nominal: ...`: a non-latching test latched the HAL. The rest of the session is skipped.
- `baseline not clean: the HAL keeps reporting ...`: the HAL already reports an anomaly while just holding the posture. The rest of the session is skipped.
- `the HAL config is the production one`: run `hal_test_config enable`.
- `no event received from the HAL in the whole session` (in the summary): check the logging discovery server and `hal_components`.

## Choosing tests and config

- **Tests:** set `HAL_ANOMALY_TESTS` (comma separated) or `tests` in the JSON config. `all` means every test of the active suite, latching ones last (non-latching only with `strict_latching`). At runtime, use the console commands `list_tests` and `run_tests <names>`.
- **Config:** JSON, selected with `HAL_ANOMALY_TESTER_CONFIG`; defaults are in [anomaly_tests.py](python/anomaly_tests.py). The main keys:
  - `target`: `sim` or `real`.
  - `joint`: the test joint (must match the test configs).
  - `hold_kp`, `hold_kd`: holding gains.
  - `estop_command_owner`: `pegasus_hal` on the robot.
  - `freeze_base_owner`: `gazebo_sim` in simulation, to freeze the base at start.
  - `strict_latching`: one latching test per session (default `false`).
  - `report_file`, `coverage_file`: session report and the total of verified anomalies.
  - `hal_components`: event sources accepted as the HAL.
  - `interface_config`: the installed config the HAL loads.
  - Timeouts.

## Unit tests (no HAL needed)

```sh
cmake .. -DDLS_HAL_BUILD_TESTS=ON && make && ctest --test-dir dls2-barebone/dls_hal
```

This runs the `RobotSafety` gtests and the tester logic against a fake HAL ([python/test_anomaly_tests.py](python/test_anomaly_tests.py)).

## Adding a test

1. Add a `Spec` to `CATALOG` in [anomaly_tests.py](python/anomaly_tests.py), giving its suite, trigger, expected events and whether it latches. A latching test goes in the catalog at its place in the latching order.
2. Add to `COVERS` the `RobotAnomalies` flags it verifies. A unit test checks that every flag is either covered or listed in `NOT_TESTABLE`.
3. If it needs a new trigger, add it to `Runner._drive()`.
4. If it needs new limits, add them to a suite file `interface_config_test_<suite>.yaml` (with `test_config: <suite>`), or create a new suite.
