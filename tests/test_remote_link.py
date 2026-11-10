"""End-to-end checks of the simulated remote-driving link: input delay (button
press -> car), video delay (car -> screen, for camera frames AND the speed /
mode / map read-outs) and the video frame rate.  No CARLA needed: a fake
simulation and a controlled clock stand in."""

import random

import pytest

from autosim import actions, session as session_module, settings
from autosim.audio import Audio
from autosim.datalog import DataLogger
from autosim.distributions import deterministic
from autosim.experiment import build_argument_parser, input_delay_distribution, video_delay_distribution
from autosim.link import CommandLink, VideoLink
from autosim.scenario import load_scenarios
from autosim.session import ScenarioSession
from autosim.simulation import DrivingMode, OfflineSimulation
from conftest import EXAMPLE_BATTERY


class FakeCar(OfflineSimulation):
    """Records when commands arrive; its speed is set by the test."""

    def __init__(self, clock):
        super().__init__()
        self.clock = clock
        self.state.mode = DrivingMode.AUTONOMOUS
        self.pull_over_at = None

    @property
    def is_offline(self):
        return False

    def pull_over(self):
        self.pull_over_at = self.clock[0]
        self.state.mode = DrivingMode.PULLING_OVER


@pytest.fixture
def clock(monkeypatch):
    now = [1000.0]
    monkeypatch.setattr(session_module.time, "monotonic", lambda: now[0])
    return now


def make_session(clock, input_delay=0.0, video_delay=0.0, fps=20.0):
    car = FakeCar(clock)
    scenario = load_scenarios(EXAMPLE_BATTERY)[0]
    session = ScenarioSession(scenario, 1, car, DataLogger("", "T", enabled=False),
                              CommandLink(deterministic(input_delay)), VideoLink(deterministic(video_delay), fps),
                              Audio(False), random.Random(1))
    session.start()
    return session, car


def run_frames(session, clock, seconds, step=1 / 30.0):
    for _ in range(int(round(seconds / step))):
        clock[0] += step
        session.update(step)


def press(session, action_id):
    actions.find_action(action_id).handler(session)


def test_quick_settings_are_the_defaults():
    options = build_argument_parser().parse_args([])
    assert options.input_delay == settings.INPUT_DELAY_SECONDS
    assert options.video_delay == settings.VIDEO_DELAY_SECONDS
    assert options.video_fps == settings.VIDEO_FPS
    options = build_argument_parser().parse_args(["--input-delay", "0.4", "--video-delay", "0.25", "--video-fps", "8"])
    assert input_delay_distribution(options).sample(random.Random()) == 0.4
    assert video_delay_distribution(options).sample(random.Random()) == 0.25
    assert options.video_fps == 8


def test_button_reaches_the_car_after_the_input_delay(clock):
    session, car = make_session(clock, input_delay=0.5)
    pressed_at = clock[0]
    press(session, "pull_over")
    run_frames(session, clock, 0.45)
    assert car.pull_over_at is None, "must not arrive before the input delay"
    run_frames(session, clock, 0.1)
    assert car.pull_over_at is not None
    assert 0.5 <= car.pull_over_at - pressed_at < 0.5 + 1 / 30.0 + 1e-9


def test_screen_shows_the_result_after_input_plus_video_delay(clock):
    session, car = make_session(clock, input_delay=0.4, video_delay=0.3)
    car.state.speed_mps = 9.0
    run_frames(session, clock, 1.0)
    assert session.shown_state.speed_mps == 9.0 and session.shown_state.mode == DrivingMode.AUTONOMOUS
    pressed_at = clock[0]
    press(session, "pull_over")
    seen_at = None
    while seen_at is None and clock[0] - pressed_at < 3.0:
        run_frames(session, clock, 1 / 30.0)
        if session.shown_state.mode == DrivingMode.PULLING_OVER:
            seen_at = clock[0]
    # The car reacted after 0.4 s; the operator sees it 0.3 s later.
    assert car.pull_over_at - pressed_at == pytest.approx(0.4, abs=1 / 30.0 + 1e-9)
    assert seen_at - pressed_at == pytest.approx(0.7, abs=2 / 20.0)   # within a video frame or two


def test_speed_readout_is_delayed_like_the_video(clock):
    session, car = make_session(clock, video_delay=0.5)
    car.state.speed_mps = 5.0
    run_frames(session, clock, 1.0)
    changed_at = clock[0]
    car.state.speed_mps = 0.0            # the car stops
    run_frames(session, clock, 0.4)
    assert session.shown_state.speed_mps == 5.0, "the operator still sees the old speed"
    run_frames(session, clock, 0.2)
    assert session.shown_state.speed_mps == 0.0
    assert session.vehicle_state.speed_mps == 0.0   # logic and data use the real state at once
    assert clock[0] - changed_at < 0.65


def test_telemetry_can_skip_the_delay(clock, monkeypatch):
    monkeypatch.setattr(settings, "TELEMETRY_FOLLOWS_VIDEO", False)
    session, car = make_session(clock, video_delay=0.5)
    car.state.speed_mps = 7.0
    run_frames(session, clock, 1 / 30.0)
    assert session.shown_state.speed_mps == 7.0


@pytest.mark.parametrize("fps", [5.0, 10.0, 20.0])
def test_operator_sees_the_set_frame_rate(clock, fps):
    session, car = make_session(clock, fps=fps)
    camera_frames = iter(range(10 ** 6))
    shown = []
    step = 1 / 30.0                      # camera and screen both at 30 Hz
    for _ in range(90):                  # three seconds
        clock[0] += step
        frame = next(camera_frames)
        session.video_link.push("front", clock[0], frame)
        session.update(step)
        visible = session.video_link.frame("front", clock[0])
        if visible is not None and (not shown or shown[-1] != visible):
            shown.append(visible)
    rate = len(shown) / 3.0
    assert rate == pytest.approx(min(fps, 30.0), rel=0.15), (fps, rate)
