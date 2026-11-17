"""Sudden operator commands: they reach the vehicle in the order pressed (also
with an input delay), Pull Over ends manual driving first, and answers given
while the operator drives manually do not take the wheel away."""

import random

import pytest

from autosim import actions, session as session_module
from autosim.audio import Audio
from autosim.datalog import DataLogger
from autosim.distributions import deterministic
from autosim.link import CommandLink, VideoLink
from autosim.scenario import load_scenarios
from autosim.session import ScenarioSession
from autosim.simulation import DrivingMode, OfflineSimulation
from conftest import EXAMPLE_BATTERY


class RecordingCar(OfflineSimulation):
    """Records every vehicle command, in the order it arrives."""

    def __init__(self):
        super().__init__()
        self.state.mode = DrivingMode.AUTONOMOUS
        self.received = []

    @property
    def is_offline(self):
        return False

    def resume_autonomy(self, obstacles_checked=True):
        self.received.append("resume")

    def pull_over(self):
        self.received.append("pull_over")

    def emergency_stop(self):
        self.received.append("estop")

    def set_manual_control(self, enabled):
        self.received.append("manual_on" if enabled else "manual_off")

    def change_lane(self, direction):
        self.received.append("lane_" + direction)
        return True

    def reroute(self, alternate_index):
        self.received.append("reroute_{}".format(alternate_index))
        return True


@pytest.fixture
def clock(monkeypatch):
    now = [1000.0]
    monkeypatch.setattr(session_module.time, "monotonic", lambda: now[0])
    return now


def make(clock, input_delay=0.0):
    car = RecordingCar()
    events = []

    class Logger(DataLogger):
        def event(self, name, *args, **kwargs):
            events.append(name)
    session = ScenarioSession(load_scenarios(EXAMPLE_BATTERY)[0], 1, car, Logger("", "T", enabled=False),
                              CommandLink(deterministic(input_delay)), VideoLink(), Audio(False), random.Random(1))
    session.start()
    return session, car, events


def frames(session, clock, seconds):
    for _ in range(int(round(seconds * 30))):
        clock[0] += 1 / 30.0
        session.update(1 / 30.0)


def press(session, action_id):
    actions.find_action(action_id).handler(session)


def test_quick_presses_arrive_in_order_even_with_input_delay(clock):
    session, car, _ = make(clock, input_delay=0.4)
    press(session, "resume_av")
    press(session, "pull_over")              # immediately after
    frames(session, clock, 0.2)
    assert car.received == [], "nothing arrives before the input delay"
    frames(session, clock, 0.4)
    assert car.received == ["resume", "pull_over"]


def test_pull_over_ends_manual_driving_first(clock):
    session, car, _ = make(clock)
    session.vehicle_state.speed_mps = 0.0
    press(session, "manual_control")
    frames(session, clock, 0.1)
    press(session, "pull_over")
    frames(session, clock, 0.1)
    assert car.received == ["manual_on", "manual_off", "pull_over"]
    assert not session.manual_control_active


def test_answer_while_driving_manually_does_not_take_the_wheel(clock):
    session, car, events = make(clock)
    session.manual_control_active = True
    for outcome in ("continue", "change_lane_left", "pull_over", "reroute:1"):
        session.perform_outcome(outcome)
    frames(session, clock, 0.2)
    assert car.received == [], car.received
    assert events.count("outcome_not_applied") == 4
    session.manual_control_active = False
    session.perform_outcome("continue")
    frames(session, clock, 0.1)
    assert car.received == ["resume"]


def test_estop_after_pull_over_is_not_lost(clock):
    session, car, _ = make(clock, input_delay=0.3)
    press(session, "pull_over")
    press(session, "emergency_stop")
    frames(session, clock, 0.5)
    assert car.received == ["pull_over", "estop"]


def test_reroute_menu_always_offers_detours_and_turning_around(clock):
    session, car, _ = make(clock)
    session.scenario.scene.alternate_routes = []
    assert actions.find_action("reroute_menu").handler(session) == "reroute"
    options = actions.DYNAMIC_DIALOG_OPTIONS["reroute"](session)
    labels = [option.label for option in options]
    assert labels == ["Detour via next left turn", "Detour via next right turn", "Turn around"], labels
    options[1].handler(session)
    options[2].handler(session)
    frames(session, clock, 0.2)
    assert car.received == ["reroute_1", "resume", "reroute_-1", "resume"], car.received
