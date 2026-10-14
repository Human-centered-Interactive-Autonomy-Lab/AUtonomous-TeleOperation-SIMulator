"""
experiment.py -- the entry point: command-line options, scenario order, main loop.

Run ``python -m autosim --help`` for every option.  Typical use:

    python -m autosim --scenarios scenarios/example_battery.csv \\
        --order random --count 12 --wait-time 20 --wait-distribution exponential \\
        --video-delay 0.25 --participant P007

    python -m autosim --mock          # no CARLA needed (development / demos)

Flow of a session
-----------------
  1. Parse options, read and validate the scenario file.
  2. Connect to CARLA (or the mock), open the window, start data recording.
  3. For each scenario in the schedule:
       a. show the waiting screen,
       b. load the scenario's world (the loading time counts toward the wait),
       c. wait out the rest of the inter-scenario time (window stays responsive),
       d. run the scenario until the operator presses "End supervision",
       e. clean up and write the scenario summary.
  4. Save the data and close.
"""

import argparse
import os
import random
import sys
import time
from typing import Any, Dict, List, Optional

from autosim import __version__, settings
from autosim.distributions import Distribution, make_distribution, parse_spec
from autosim.scenario import Scenario, ScenarioFileError, load_scenarios

DISTRIBUTION_CHOICES = ["deterministic", "uniform", "normal", "exponential", "clipped_exponential"]


# =============================================================================
# Command line
# =============================================================================

def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m autosim",
        description="AUTOSIM - remote-assistance experiments on the CARLA simulator.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    files = parser.add_argument_group("scenarios")
    files.add_argument("--scenarios", default=settings.DEFAULT_SCENARIO_FILE,
                       help="scenario CSV file (one row per scenario)")
    files.add_argument("--group", default=None, help="only run rows whose 'group' column equals this")
    files.add_argument("--order", choices=["in_order", "random"], default=settings.DEFAULT_SCENARIO_ORDER,
                       help="run scenarios in file order or shuffled")
    files.add_argument("--with-replacement", action="store_true", default=settings.DEFAULT_WITH_REPLACEMENT,
                       help="with --order random: draw every scenario independently (repeats possible)")
    files.add_argument("--count", type=int, default=settings.DEFAULT_SCENARIO_COUNT,
                       help="number of scenarios to run; 0 = every row once; more than the rows loops the file")
    files.add_argument("--seed", type=int, default=None, help="random seed (reproducible order and values)")

    timing = parser.add_argument_group("time between scenarios (seconds)")
    timing.add_argument("--wait-time", type=float, default=settings.DEFAULT_WAIT_SECONDS,
                        help="mean waiting time between scenarios")
    timing.add_argument("--wait-distribution", choices=DISTRIBUTION_CHOICES,
                        default=settings.DEFAULT_WAIT_DISTRIBUTION, help="shape of the waiting-time distribution")
    timing.add_argument("--wait-spread", type=float, default=settings.DEFAULT_WAIT_SPREAD_SECONDS,
                        help="half-width (uniform) or standard deviation (normal)")
    timing.add_argument("--wait-min", type=float, default=settings.DEFAULT_WAIT_MIN_SECONDS,
                        help="shortest allowed wait (all distributions)")
    timing.add_argument("--wait-max", type=float, default=settings.DEFAULT_WAIT_MAX_SECONDS,
                        help="longest allowed wait (all distributions)")

    link = parser.add_argument_group("simulated communication link")
    link.add_argument("--video-delay", type=float, default=settings.DEFAULT_VIDEO_DELAY_SECONDS,
                      help="mean video delay (s)")
    link.add_argument("--video-delay-distribution", choices=DISTRIBUTION_CHOICES,
                      default=settings.DEFAULT_VIDEO_DELAY_DISTRIBUTION)
    link.add_argument("--video-delay-spread", type=float, default=settings.DEFAULT_VIDEO_DELAY_SPREAD_SECONDS,
                      help="half-width (uniform) or standard deviation (normal) of the video delay")
    link.add_argument("--input-delay", type=float, default=settings.DEFAULT_INPUT_DELAY_SECONDS,
                      help="mean delay before operator commands reach the vehicle (s)")
    link.add_argument("--input-delay-distribution", choices=DISTRIBUTION_CHOICES,
                      default=settings.DEFAULT_INPUT_DELAY_DISTRIBUTION)
    link.add_argument("--input-delay-spread", type=float, default=settings.DEFAULT_INPUT_DELAY_SPREAD_SECONDS)
    link.add_argument("--video-fps", type=float, default=settings.DEFAULT_VIDEO_FPS,
                      help="video frames per second shown to the operator")
    link.add_argument("--video-fps-variability", type=float, default=settings.DEFAULT_VIDEO_FPS_VARIABILITY,
                      help="standard deviation of the frame rate (frames per second)")
    link.add_argument("--video-resolution", type=float, default=settings.DEFAULT_VIDEO_RESOLUTION_SCALE,
                      help="camera resolution as a fraction of the on-screen size (1.0 = sharp, 0.5 = half)")

    window = parser.add_argument_group("window")
    window.add_argument("--window-size", default="{}x{}".format(*settings.DEFAULT_WINDOW_SIZE),
                        help="WIDTHxHEIGHT of the window")
    window.add_argument("--fullscreen", action="store_true", help="use the whole screen")

    data = parser.add_argument_group("data")
    data.add_argument("--participant", default=settings.DEFAULT_PARTICIPANT_ID, help="participant ID for file names")
    data.add_argument("--output", default=settings.DEFAULT_OUTPUT_FOLDER, help="folder for session data")
    data.add_argument("--no-record", dest="record", action="store_false", help="do not write any data files")

    simulator = parser.add_argument_group("simulator")
    simulator.add_argument("--mock", action="store_true", help="run without CARLA (simple built-in stand-in)")
    simulator.add_argument("--host", default=settings.CARLA_HOST, help="CARLA server address")
    simulator.add_argument("--port", type=int, default=settings.CARLA_PORT, help="CARLA server port")
    simulator.add_argument("--no-audio", dest="audio", action="store_false", help="disable sounds")
    parser.add_argument("--version", action="version", version="AUTOSIM " + __version__)
    return parser


def parse_window_size(text: str):
    try:
        width, height = (int(part) for part in text.lower().split("x"))
        return width, height
    except ValueError:
        raise SystemExit("--window-size must look like 1600x900, got '{}'".format(text))


def wait_distribution(options: argparse.Namespace) -> Distribution:
    return make_distribution(options.wait_distribution, options.wait_time, options.wait_spread,
                             options.wait_min, options.wait_max)


def video_delay_distribution(options: argparse.Namespace, scenario: Optional[Scenario] = None) -> Distribution:
    if scenario is not None and scenario.video_delay_spec:
        return parse_spec(scenario.video_delay_spec)
    return make_distribution(options.video_delay_distribution, options.video_delay, options.video_delay_spread, 0.0)


def input_delay_distribution(options: argparse.Namespace, scenario: Optional[Scenario] = None) -> Distribution:
    if scenario is not None and scenario.input_delay_spec:
        return parse_spec(scenario.input_delay_spec)
    return make_distribution(options.input_delay_distribution, options.input_delay, options.input_delay_spread, 0.0)


# =============================================================================
# Scenario order
# =============================================================================

def build_schedule(scenarios: List[Scenario], order: str, count: int, with_replacement: bool,
                   rng: random.Random) -> List[Scenario]:
    """The list of scenarios to run, in order.

    * ``count`` 0 means every scenario once.
    * in_order: the file order, repeated if count exceeds the number of rows.
    * random without replacement: shuffled; when every row has been used, a
      new shuffled pass starts (so long batteries stay balanced).
    * random with replacement: each pick independent.
    Always terminates (the old code could loop forever).
    """
    if not scenarios:
        return []
    total = count if count > 0 else len(scenarios)
    if order == "in_order":
        return [scenarios[i % len(scenarios)] for i in range(total)]
    if with_replacement:
        return [rng.choice(scenarios) for _ in range(total)]
    schedule: List[Scenario] = []
    while len(schedule) < total:
        one_pass = list(scenarios)
        rng.shuffle(one_pass)
        schedule.extend(one_pass)
    return schedule[:total]


# =============================================================================
# Main loop
# =============================================================================

class ExperimentRunner:
    def __init__(self, options: argparse.Namespace) -> None:
        self.options = options
        self.rng = random.Random(options.seed)

    def run(self) -> int:
        # Heavy imports here so "--help" works even without pygame / CARLA installed.
        import pygame
        from autosim.audio import Audio
        from autosim.datalog import DataLogger
        from autosim.interface import OperatorInterface
        from autosim.link import CommandLink, VideoLink
        from autosim.session import ScenarioSession

        options = self.options
        scenario_file = _resolve_path(options.scenarios)
        try:
            scenarios = load_scenarios(scenario_file, options.group)
        except ScenarioFileError as error:
            print(error)
            return 2
        schedule = build_schedule(scenarios, options.order, options.count, options.with_replacement, self.rng)
        print("[experiment] {} scenarios loaded; running {}.".format(len(scenarios), len(schedule)))

        simulation = self._make_simulation()
        try:
            simulation.connect()
        except RuntimeError as error:
            print("[experiment] {}".format(error))
            return 3

        missing = simulation.missing_maps([s.scene.map_name for s in scenarios if s.scene.map_name])
        if missing:
            skipped = [s for s in schedule if s.scene.map_name in missing]
            print("[experiment] WARNING: the simulator does not have these maps: {}".format(", ".join(missing)))
            print("[experiment] WARNING: skipping {} scheduled scenario(s) that use them: {}".format(
                len(skipped), ", ".join(sorted(set(s.scenario_id for s in skipped)))))
            schedule = [s for s in schedule if s.scene.map_name not in missing]
            if not schedule:
                print("[experiment] Nothing left to run.")
                simulation.close()
                return 4

        pygame.init()
        flags = pygame.FULLSCREEN if options.fullscreen else pygame.RESIZABLE
        size = (0, 0) if options.fullscreen else parse_window_size(options.window_size)
        screen = pygame.display.set_mode(size, flags)
        pygame.display.set_caption(settings.WINDOW_TITLE)
        interface = OperatorInterface(screen)
        audio = Audio(options.audio)
        logger = DataLogger(options.output, options.participant, options.record)
        logger.save_run_settings(dict(vars(options), scenario_file=scenario_file,
                                      schedule=[s.scenario_id for s in schedule]), scenario_file)
        clock = pygame.time.Clock()
        waits = wait_distribution(options)
        quit_requested = False

        try:
            for index, scenario in enumerate(schedule, start=1):
                # --- wait + load ------------------------------------------------
                wait_seconds = waits.sample(self.rng) if index > 1 else 0.0
                wait_start = time.monotonic()
                interface.draw_waiting_screen("Please wait for the next task...")
                if not interface.handle_events(None):
                    quit_requested = True
                    break
                video_link = VideoLink(video_delay_distribution(options, scenario), options.video_fps,
                                       options.video_fps_variability, self.rng)
                command_link = CommandLink(input_delay_distribution(options, scenario), self.rng)
                print("[experiment] Scenario {}/{}: {}".format(index, len(schedule), scenario.scenario_id))
                simulation.load_scenario(scenario.scene, scenario.time_of_day, scenario.weather, scenario.lights,
                                         interface.camera_requests(options.video_resolution),
                                         scenario.target_speed_mph, scenario.start_stopped)
                load_seconds = time.monotonic() - wait_start
                if load_seconds > wait_seconds and index > 1:
                    print("[experiment] Note: loading took {:.1f} s, longer than the {:.1f} s wait.".format(
                        load_seconds, wait_seconds))
                while time.monotonic() - wait_start < wait_seconds:
                    if not interface.handle_events(None):
                        quit_requested = True
                        break
                    interface.draw_waiting_screen("Please wait for the next task...")
                    clock.tick(settings.GUI_FRAMES_PER_SECOND)
                if quit_requested:
                    break
                actual_wait = time.monotonic() - wait_start

                # --- run --------------------------------------------------------
                session = ScenarioSession(scenario, index, simulation, logger, command_link, video_link, audio, self.rng)
                interface.new_scenario(session)
                session.start()
                session.log("wait_before_scenario", drawn_s=round(wait_seconds, 2), actual_s=round(actual_wait, 2))
                clock.tick()
                while not session.end_requested:
                    dt = clock.tick(settings.GUI_FRAMES_PER_SECOND) / 1000.0
                    if not interface.handle_events(session):
                        quit_requested = True
                        session.log("window_closed")
                        break
                    session.update(dt, interface.manual_keys())
                    interface.draw(session, video_link)
                summary = session.end()
                summary.update(order_index=index, wait_drawn_s=round(wait_seconds, 2),
                               wait_actual_s=round(actual_wait, 2), ended_by="window_closed" if quit_requested else "operator")
                logger.scenario_summary(summary)
                simulation.cleanup_scenario()
                if quit_requested:
                    break
        except KeyboardInterrupt:
            print("[experiment] Interrupted.")
        finally:
            simulation.close()
            logger.close()
            pygame.quit()
        print("[experiment] Session finished.")
        return 0

    def _make_simulation(self):
        if self.options.mock:
            from autosim.simulation_mock import MockSimulation
            return MockSimulation()
        from autosim.simulation_carla import CarlaSimulation
        return CarlaSimulation(self.options.host, self.options.port)


def _resolve_path(path: str) -> str:
    """Accept paths relative to the working directory or to the repository root."""
    if os.path.isfile(path):
        return path
    repository_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    candidate = os.path.join(repository_root, path)
    return candidate if os.path.isfile(candidate) else path


def main(argv: Optional[List[str]] = None) -> int:
    options = build_argument_parser().parse_args(argv)
    return ExperimentRunner(options).run()


if __name__ == "__main__":
    sys.exit(main())
