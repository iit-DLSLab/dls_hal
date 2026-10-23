#!/usr/bin/env python3
"""dls2 periodic app that tests the HAL anomaly detection end to end, from
outside the HAL (no HAL changes needed).

It replaces the control layer: it publishes the reference on
rt/control_signal and, on the real robot, can call the HAL "emergency" console
command. It reads the HAL reactions on rt/log_events (HAL events, domain
logging) and rt/hal_commands_feedback. The anomalies come from its references
and from the limits of the active test config (hal_test_config). The test
logic lives in anomaly_tests.py. See ../README.md.
"""

from __future__ import annotations

import json
import os
import tempfile
import threading
import time

from dls2_py.env_bootstrap import setup_environment

import simple_yaml
from anomaly_tests import (
    ALL_ANOMALIES, CATALOG, FALLBACK_EVENT_IDS, Event, HalConfig, Observation, Runner, parse_event_ids,
)

APP_ID = "hal_anomaly_tester"
SIGNALS_DOMAIN = 3
COMMAND_DOMAIN = 2
LOGGING_DOMAIN = 7
CONFIG_PATH = f"/usr/share/{APP_ID}/config/{APP_ID}.json"
DEFAULTS = {
    "interface_config": "/usr/include/pegasus-description/interface_config.yaml",
    "event_config_header": "/usr/include/dls2/log/event_config.hpp",
}

PeriodicAppPlugin = setup_environment(
    package_name=APP_ID,
    module_file=__file__,
    required_message_modules=("BlindState", "ControlSignal", "EventLog", "CommandCall"),
)

import fastdds  # noqa: E402  (available once the dls2 environment is set up)
import BlindState  # noqa: E402
import CommandCall  # noqa: E402
import ControlSignal  # noqa: E402
import EventLog  # noqa: E402


def log(message: str) -> None:
    print(f"[{APP_ID}] {message}", flush=True)


def load_config() -> dict:
    """Tester config ($HAL_ANOMALY_TESTER_CONFIG or the installed one), then
    $HAL_ANOMALY_TESTS (comma separated) overriding the test list."""
    path = os.environ.get("HAL_ANOMALY_TESTER_CONFIG", CONFIG_PATH)
    config = dict(DEFAULTS)
    if os.path.isfile(path):
        with open(path, encoding="utf-8") as file:
            config.update(json.load(file))
        log(f"config: {path}")
    else:
        log(f"config {path} not found, using defaults")
    if os.environ.get("HAL_ANOMALY_TESTS"):
        config["tests"] = os.environ["HAL_ANOMALY_TESTS"].split(",")
    return config


def load_event_ids(path: str) -> dict[str, int]:
    try:
        with open(path, encoding="utf-8") as file:
            return parse_event_ids(file.read())
    except (OSError, ValueError) as error:
        log(f"cannot read EventIDs from {path} ({error}): using the built-in table")
        return FALLBACK_EVENT_IDS


def discovery_server(domain_name: str) -> tuple[str, int]:
    """Discovery server of a dls2 domain, from the same servers.yaml as the C++ participants."""
    path = os.environ.get("DLS_SERVERS_PATH", "/opt/dls2/include/dls2/util/messaging/servers.yaml")
    servers = simple_yaml.load_file(path)
    return str(servers[domain_name]["ip"]), int(servers[domain_name]["port"])


def create_participant(domain: int, domain_name: str):
    """Discovery-server client participant on a dls2 domain (dls2_py only knows domain 3)."""
    ip, port = discovery_server(domain_name)
    profile = f"{APP_ID}_domain_{domain}"
    xml = f"""<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
  <participant profile_name="{profile}">
    <domainId>{domain}</domainId>
    <rtps><builtin><discovery_config>
      <discoveryProtocol>CLIENT</discoveryProtocol>
      <discoveryServersList><locator><udpv4><address>{ip}</address><port>{port}</port></udpv4></locator></discoveryServersList>
    </discovery_config></builtin></rtps>
  </participant>
</profiles>
"""
    fd, path = tempfile.mkstemp(prefix=f"{profile}_", suffix=".xml")
    with os.fdopen(fd, "w") as file:
        file.write(xml)
    factory = fastdds.DomainParticipantFactory.get_instance()
    factory.load_XML_profiles_file(path)
    participant = factory.create_participant_with_profile(profile)
    os.remove(path)
    if participant is None:
        raise RuntimeError(f"cannot create a participant on domain {domain_name} ({ip}:{port})")
    return participant


class EventCollector(fastdds.DataReaderListener):
    """Keeps every received EventLog (the dls2_py reader keeps only the last one)."""

    def __init__(self) -> None:
        super().__init__()
        self.lock = threading.Lock()
        self.events: list[Event] = []

    def on_data_available(self, reader) -> None:
        info = fastdds.SampleInfo()
        while True:
            sample = EventLog.EventLog()
            if reader.take_next_sample(sample, info) != fastdds.RETCODE_OK:
                return
            if info.valid_data:
                event = Event(time.monotonic(), sample.component_name(), int(sample.event_id()),
                              int(sample.severity()), sample.msg())
                with self.lock:
                    self.events.append(event)

    def drain(self) -> list[Event]:
        with self.lock:
            events, self.events = self.events, []
        return events


class EventSubscription:
    """Reliable, keep-all reader of rt/log_events, like the dls2 EventListener."""

    def __init__(self) -> None:
        self.participant = create_participant(LOGGING_DOMAIN, "logging")
        data_type = EventLog.EventLogPubSubType()
        data_type.set_name(data_type.get_name())
        self.participant.register_type(fastdds.TypeSupport(data_type))
        topic_qos = fastdds.TopicQos()
        self.participant.get_default_topic_qos(topic_qos)
        self.topic = self.participant.create_topic("rt/log_events", data_type.get_name(), topic_qos)
        subscriber_qos = fastdds.SubscriberQos()
        self.participant.get_default_subscriber_qos(subscriber_qos)
        self.subscriber = self.participant.create_subscriber(subscriber_qos)
        reader_qos = fastdds.DataReaderQos()
        self.subscriber.get_default_datareader_qos(reader_qos)
        reader_qos.reliability().kind = fastdds.RELIABLE_RELIABILITY_QOS
        reader_qos.history().kind = fastdds.KEEP_ALL_HISTORY_QOS
        self.collector = EventCollector()
        self.reader = self.subscriber.create_datareader(self.topic, reader_qos, self.collector)


class CommandCaller:
    """Calls dls2 console commands (rt/command_call, domain command)."""

    def __init__(self) -> None:
        self.participant = create_participant(COMMAND_DOMAIN, "command")
        data_type = CommandCall.CommandCallPubSubType()
        data_type.set_name(data_type.get_name())
        self.participant.register_type(fastdds.TypeSupport(data_type))
        topic_qos = fastdds.TopicQos()
        self.participant.get_default_topic_qos(topic_qos)
        self.topic = self.participant.create_topic("rt/command_call", data_type.get_name(), topic_qos)
        publisher_qos = fastdds.PublisherQos()
        self.participant.get_default_publisher_qos(publisher_qos)
        self.publisher = self.participant.create_publisher(publisher_qos)
        writer_qos = fastdds.DataWriterQos()
        self.publisher.get_default_datawriter_qos(writer_qos)
        writer_qos.reliability().kind = fastdds.RELIABLE_RELIABILITY_QOS
        self.writer = self.publisher.create_datawriter(self.topic, writer_qos)

    def call(self, owner: str, command: str) -> None:
        msg = CommandCall.CommandCall()
        msg.owner(owner)
        msg.command_name(command)
        self.writer.write(msg)
        log(f"called console command {owner}::{command}")


def _vector(values):
    vector = ControlSignal.double_vector()
    for value in values:
        vector.push_back(float(value))
    return vector


class HalAnomalyTester(PeriodicAppPlugin):
    def __init__(self) -> None:
        super().__init__(APP_ID, SIGNALS_DOMAIN)
        self.config = load_config()
        hal_config_path = self.config["interface_config"]
        hal = HalConfig.from_dict(simple_yaml.load_file(hal_config_path))
        log(f"HAL config {hal_config_path}: suite {hal.suite or 'NONE (production config)'}")
        self.runner = Runner(self.config, hal, load_event_ids(self.config["event_config_header"]), log=log)
        self.report_written = False
        self.sequence_id = 0
        self.pending_tests: list[str] | None = list(self.config.get("tests", ["all"]))
        self.lock = threading.Lock()

        self.events = EventSubscription()
        needs_commands = self.runner.cfg["estop_command_owner"] or self.runner.cfg["freeze_base_owner"]
        self.commands = CommandCaller() if needs_commands else None

        self.state_in = self.build_input(
            "rt/blind_state", BlindState.BlindStatePubSubType(), BlindState.BlindState(),
            required_on_activation=False)
        self.feedback_in = self.build_input(
            "rt/hal_commands_feedback", ControlSignal.ControlSignalPubSubType(),
            ControlSignal.ControlSignal(), required_on_activation=False)
        self.reference_out = self.build_output(
            "rt/control_signal", ControlSignal.ControlSignalPubSubType(), ControlSignal.ControlSignal())

        self.add_command("run_tests", "run HAL anomaly tests, e.g. run_tests velocity_limit,estop",
                         self._run_tests_command, enabled=True)
        self.add_command("list_tests", "list the HAL anomaly tests", self._list_tests_command, enabled=True)

    def check_activation(self) -> bool:
        return True

    def deactivation(self, timestamp_ns: int) -> bool:
        del timestamp_ns
        return True

    # Console commands (called from another thread) -----------------------------

    def _run_tests_command(self, value: str) -> bool:
        if self.runner.busy:
            log("tests already running")
            return False
        with self.lock:
            self.pending_tests = value.replace(" ", ",").split(",")
        return True

    def _list_tests_command(self) -> bool:
        for spec in CATALOG:
            kind = "latching" if spec.latching else "repeatable"
            log(f"{spec.name:<26} suite {spec.suite:<14} {kind:<10} {spec.description}")
        return True

    # Periodic loop ---------------------------------------------------------------

    def run(self, timestamp_ns: int) -> None:
        with self.lock:
            tests, self.pending_tests = self.pending_tests, None
        if tests is not None:
            self.report_written = False
            self.runner.start(tests)

        self.read()
        out = self.runner.step(self._observe())

        if out.publish_reference:
            msg = self.reference_out.data
            msg.timestamp(timestamp_ns)
            msg.sequence_id(self.sequence_id % 1000)
            self.sequence_id += 1
            msg.control_mode(1)  # informative: the HAL uses the mode of its config
            msg.joints_position(_vector(out.position))
            msg.joints_velocity(_vector(out.velocity))
            msg.joints_torques(_vector(out.torque))
            msg.kp(_vector(out.kp))
            msg.kd(_vector(out.kd))
            self.reference_out.write()
        if out.command_call is not None and self.commands is not None:
            self.commands.call(*out.command_call)

        if self.runner.finished and not self.report_written:
            self._write_report()

    def _observe(self) -> Observation:
        state = self.state_in.getData()
        position = list(state.joints_position())
        velocity = list(state.joints_velocity())
        fb = self.feedback_in.getData()
        feedback = None
        if len(fb.joints_torques()) > 0:
            feedback = {"control_mode": int(fb.control_mode()),
                        "position": list(fb.joints_position()),
                        "velocity": list(fb.joints_velocity()),
                        "torque": list(fb.joints_torques())}
        return Observation(time.monotonic(), position or None, velocity or None, feedback,
                           self.events.collector.drain())

    def _write_report(self) -> None:
        self.report_written = True
        path = self.runner.cfg["report_file"]
        report = {
            "target": self.runner.cfg["target"],
            "suite": self.runner.hal.suite,
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "results": [vars(result) for result in self.runner.results],
            "anomalies_verified": sorted(self.runner.verified_anomalies()),
        }
        try:
            with open(path, "w", encoding="utf-8") as file:
                json.dump(report, file, indent=2)
            log(f"report written to {path}")
        except OSError as error:
            log(f"cannot write the report {path}: {error}")
        self._update_coverage()

    def _update_coverage(self) -> None:
        """Adds this session to the anomalies verified over all sessions, per target."""
        path, target = self.runner.cfg["coverage_file"], self.runner.cfg["target"]
        coverage: dict = {}
        try:
            with open(path, encoding="utf-8") as file:
                coverage = json.load(file)
        except (OSError, ValueError):
            pass
        earlier = set(coverage.get(target, []))
        session = self.runner.verified_anomalies()
        coverage[target] = sorted(earlier | session)
        try:
            with open(path, "w", encoding="utf-8") as file:
                json.dump(coverage, file, indent=2)
        except OSError as error:
            log(f"cannot write the coverage {path}: {error}")
        log(self.runner.coverage(earlier, f"all {target} sessions")
            + f"\n  this session: {len(session)}/{len(ALL_ANOMALIES)}; reset the total: delete {path}")


def main() -> None:
    HalAnomalyTester().execute()


if __name__ == "__main__":
    main()
