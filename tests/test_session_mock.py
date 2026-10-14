"""Runs real scenarios on the mock simulation (no CARLA, no screen)."""

import random

import pygame
import pytest

from autosim import actions
from autosim.audio import Audio
from autosim.datalog import DataLogger
from autosim.distributions import deterministic
from autosim.interface import OperatorInterface
from autosim.link import CommandLink, VideoLink
from autosim.messages import resolve_message
from autosim.scenario import load_scenarios
from autosim.session import ScenarioSession
from autosim.simulation import DrivingMode
from autosim.simulation_mock import MockSimulation
from conftest import EXAMPLE_BATTERY

FRAME = 1.0 / 30


@pytest.fixture(scope="module")
def screen():
    pygame.init()
    return pygame.display.set_mode((1280, 720))


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


@pytest.fixture
def run(monkeypatch, screen, tmp_path):
    """Returns a function that starts a scenario and steps it with fake time."""
    clock = Clock()
    import autosim.session
    import autosim.simulation_mock
    monkeypatch.setattr(autosim.session.time, "monotonic", clock)
    monkeypatch.setattr(autosim.simulation_mock.time, "monotonic", clock)
    scenarios = {s.scenario_id: s for s in load_scenarios(EXAMPLE_BATTERY)}

    def start(scenario_id, input_delay=0.0, record_folder=None):
        scenario = scenarios[scenario_id]
        interface = OperatorInterface(screen)
        simulation = MockSimulation()
        simulation.connect()
        simulation.load_scenario(scenario.scene, scenario.time_of_day, scenario.weather, scenario.lights,
                                 interface.camera_requests(0.25), scenario.target_speed_mph, scenario.start_stopped)
        logger = DataLogger(str(record_folder or tmp_path), "T", enabled=record_folder is not None)
        session = ScenarioSession(scenario, 1, simulation, logger, CommandLink(deterministic(input_delay)),
                                  VideoLink(), Audio(False), random.Random(0))
        interface.new_scenario(session)
        session.start()

        def step(seconds, draw=False):
            for _ in range(int(round(seconds / FRAME))):
                clock.now += FRAME
                session.update(FRAME, {})
            if draw:
                interface.draw(session, session.video_link)
        session.step = step
        session.interface = interface
        session.logger_ = logger
        return session
    return start


def test_question_appears_and_lane_change_continues(run):
    session = run("blocked_lane")
    session.step(10)
    assert session.vehicle_state.mode == DrivingMode.HOLDING
    assert session.decision is not None and "left lane" in session.decision.prompt
    session.answer("Yes")
    session.step(15)
    assert session.vehicle_state.mode == DrivingMode.AUTONOMOUS
    assert session.vehicle_state.route_traveled_m > 70     # drove past the van
    assert session.vehicle_state.collision_count == 0


def test_emergency_stop_obeys_input_delay(run):
    session = run("software_fault", input_delay=1.0)
    session.step(3)
    actions.find_action("emergency_stop").handler(session)
    session.step(0.5)
    assert session.vehicle_state.mode == DrivingMode.AUTONOMOUS    # not yet delivered
    session.step(3)
    assert session.vehicle_state.mode == DrivingMode.EMERGENCY_STOP
    assert session.vehicle_state.speed_mph < 0.5


def test_manual_control_refused_when_fast(run):
    session = run("low_battery")
    session.step(4)
    assert session.vehicle_state.speed_mph > 5
    actions.find_action("manual_control").handler(session)
    assert not session.manual_control_active
    actions.find_action("emergency_stop").handler(session)
    session.step(4)
    actions.find_action("manual_control").handler(session)
    assert session.manual_control_active


def test_pull_over_ends_pulled_over_with_hazards(run):
    session = run("low_tire")
    session.step(3)
    actions.find_action("pull_over").handler(session)
    session.step(15)
    assert session.vehicle_state.mode == DrivingMode.PULLED_OVER
    assert session.vehicle_state.hazard_lights_on


def test_end_with_unanswered_question_is_allowed(run):
    session = run("weather_sensors")
    session.step(8)
    assert session.decision is not None
    actions.find_action("end_supervision").handler(session)
    assert session.end_requested
    summary = session.end()
    assert summary["decision_answer"] == "(unanswered)"
    assert summary["correct_responses"] == "Pull over"
    assert "is_correct" not in " ".join(summary)


def test_reroute_detour_avoids_police(run):
    session = run("police_blockage")
    session.step(10)
    session.answer("Take detour")
    session.step(20)
    assert session.vehicle_state.collision_count == 0
    assert session.vehicle_state.mode == DrivingMode.AUTONOMOUS


def test_pedestrian_gap_records_measurement(run):
    session = run("pedestrian_gap")
    session.step(20)
    session.answer("Clear to go")
    summary = session.end()
    assert "gap_available_s" in summary and "lane_occupied_at_go" in summary


def test_route_completion_text(run):
    session = run("low_battery")
    session.step(80)
    assert session.route_complete
    assert session.vehicle_state.mode == DrivingMode.ROUTE_COMPLETE


def test_passenger_message_from_text_file(run):
    session = run("passenger_discomfort")
    notification = session.notifications[0]
    label, accept = notification.actions[0]
    accept()
    assert "carsick" in session.transcript[-1][1]


def test_message_resolution(tmp_path):
    (tmp_path / "audio").mkdir()
    (tmp_path / "audio" / "hello.txt").write_text("Hello there.")
    assert resolve_message("hello.txt", str(tmp_path)) == ("Hello there.", "")
    assert resolve_message("Just text", str(tmp_path)) == ("Just text", "")


def test_interface_draws_every_dialog(run):
    session = run("police_blockage")
    session.step(10, draw=True)
    for name in actions.DIALOGS:
        session.interface.open_dialog = name
        session.interface.draw(session, session.video_link)
    session.interface.open_dialog = None
    session.interface.show_notification_list = True
    session.interface.draw(session, session.video_link)


def test_data_files_written(run, tmp_path):
    import csv
    import os
    session = run("blocked_lane", record_folder=tmp_path)
    session.step(10)
    session.answer("Yes")
    session.logger_.scenario_summary(session.end())
    session.logger_.close()
    folder = [os.path.join(str(tmp_path), f) for f in os.listdir(str(tmp_path))][0]
    events = list(csv.DictReader(open(os.path.join(folder, "events.csv"))))
    assert any(e["event"] == "decision_shown" for e in events)
    summary = list(csv.DictReader(open(os.path.join(folder, "scenarios.csv"))))
    assert summary[0]["decision_answer"] == "Yes"
