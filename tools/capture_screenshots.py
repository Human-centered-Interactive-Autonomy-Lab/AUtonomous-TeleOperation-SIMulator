"""
capture_screenshots.py -- run scenarios on the mock simulation without a
screen and save PNG screenshots (for documentation and quick checks).

    python tools/capture_screenshots.py --out docs/images
    python tools/capture_screenshots.py --scenario blocked_lane --seconds 12 --every 2

Each scenario is run for a number of simulated seconds.  Scripted "operator"
answers can be given with --answer LABEL@SECONDS (e.g. --answer Yes@9).
"""

import argparse
import os
import random
import sys

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pygame  # noqa: E402

from autosim import settings  # noqa: E402
from autosim.audio import Audio  # noqa: E402
from autosim.datalog import DataLogger  # noqa: E402
from autosim.interface import OperatorInterface  # noqa: E402
from autosim.link import CommandLink, VideoLink  # noqa: E402
from autosim.scenario import load_scenarios  # noqa: E402
from autosim.session import ScenarioSession  # noqa: E402
from autosim.simulation_mock import MockSimulation  # noqa: E402


class FakeClock:
    """Makes time.monotonic() advance by exactly one frame per step, so the
    capture is fast and repeatable."""

    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now


def run(scenario_id: str, seconds: float, every: float, answers, out_folder: str, size, actions_at) -> list:
    import autosim.link
    import autosim.session
    import autosim.simulation_mock
    import time as real_time

    clock = FakeClock()
    for module in (autosim.session, autosim.simulation_mock):
        module.time = type("T", (), {"monotonic": staticmethod(clock)})
    scenario_file = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                 "scenarios", "example_battery.csv")
    scenario = [s for s in load_scenarios(scenario_file) if s.scenario_id == scenario_id][0]

    screen = pygame.display.set_mode(size)
    interface = OperatorInterface(screen)
    simulation = MockSimulation()
    simulation.connect()
    simulation.load_scenario(scenario.scene, scenario.time_of_day, scenario.weather, scenario.lights,
                             interface.camera_requests(1.0), scenario.target_speed_mph, scenario.start_stopped)
    rng = random.Random(1)
    video = VideoLink(fps=settings.GUI_FRAMES_PER_SECOND)
    session = ScenarioSession(scenario, 1, simulation, DataLogger("", "", enabled=False),
                              CommandLink(), video, Audio(False), rng)
    interface.new_scenario(session)
    session.start()

    saved = []
    dt = 1.0 / settings.GUI_FRAMES_PER_SECOND
    steps = int(seconds / dt)
    next_shot = every
    for step in range(steps):
        clock.now += dt
        elapsed = (step + 1) * dt
        for label, at in list(answers):
            if elapsed >= at and session.decision is not None:
                session.answer(label)
                answers.remove((label, at))
        for action_id, at in list(actions_at):
            if elapsed >= at:
                from autosim import actions
                result = actions.find_action(action_id).handler(session)
                if result in actions.DIALOGS:
                    interface.open_dialog = result
                actions_at.remove((action_id, at))
        pygame.event.pump()
        session.update(dt, {})
        interface.draw(session, video)
        if elapsed + 1e-9 >= next_shot:
            path = os.path.join(out_folder, "{}_{:04.1f}s.png".format(scenario_id, elapsed))
            pygame.image.save(screen, path)
            saved.append(path)
            next_shot += every
    print("{}: mode={} speed={:.1f} mph traveled={:.0f} m alerts={}".format(
        scenario_id, session.vehicle_state.mode, session.vehicle_state.speed_mph,
        session.vehicle_state.route_traveled_m, [a.text for a in session.alerts]))
    return saved


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario", default="blocked_lane")
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--every", type=float, default=4.0, help="screenshot interval (simulated seconds)")
    parser.add_argument("--answer", action="append", default=[], help="LABEL@SECONDS")
    parser.add_argument("--action", action="append", default=[], help="ACTION_ID@SECONDS (button press)")
    parser.add_argument("--out", default="screenshots")
    parser.add_argument("--size", default="1600x900")
    options = parser.parse_args()
    os.makedirs(options.out, exist_ok=True)
    pygame.init()
    answers = [(item.rsplit("@", 1)[0], float(item.rsplit("@", 1)[1])) for item in options.answer]
    actions_at = [(item.rsplit("@", 1)[0], float(item.rsplit("@", 1)[1])) for item in options.action]
    size = tuple(int(v) for v in options.size.split("x"))
    for path in run(options.scenario, options.seconds, options.every, answers, options.out, size, actions_at):
        print("saved", path)


if __name__ == "__main__":
    main()
