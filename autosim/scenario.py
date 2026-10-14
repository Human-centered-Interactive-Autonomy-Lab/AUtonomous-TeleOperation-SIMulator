"""
scenario.py -- read the scenario CSV file (one row = one scenario).

Everything about a scenario is decided by its row: which scene (world setup)
to load, weather, the vehicle status values the operator sees, alerts,
which buttons are visible, and the task logic.  Every column is optional
except ``scenario_id``; a blank or missing value falls back to the defaults
in settings.py.  Columns this program does not know are kept in
``Scenario.extra`` so task scripts can use them.

The full column reference is in docs/SCENARIOS.md.  In short:

  scenario_id        Unique name, used in the data files.            (required)
  group              Any label; select with --group on the command line.
  description        Free text for the experimenter (not shown).
  scene              Scene JSON file (relative to the CSV or its scenes/ folder).
  time_of_day        Sun preset: day / sunset / night.
  weather            Weather preset: clear / overcast / rain / storm / ...
  lights             Street lights: auto / off / low / high.
  target_speed_mph   Cruising speed of the automated vehicle.
  start_stopped      yes = the car is standing still when the scenario starts.
  route_end          customer / depot -- what happens when the route completes.
  alerts             Alerts shown at start, separated by "|".  Prefix with
                     "critical:", "warning:" or "info:" to set the severity.
  hidden_buttons     Action buttons to hide, separated by "|" (e.g. honk|talk_externally).
  <status columns>   Any key of settings.DEFAULT_VEHICLE_STATUS
                     (battery_percent, tire_fl_psi, software_status, ...).
                     A number, text, or a distribution such as uniform:30,34.
  task_trigger       When the task question appears: "start", a distance in
                     metres along the route, or a trigger name from the scene.
  task_prompt        The question, e.g. "Is the left lane open?".
  task_options       Answers and what they do, e.g. "Yes=continue | No=change_lane_left".
  task_alert         Alert text added when the task triggers.
  task_script        Python task script (relative to the CSV or its scripts/ folder).
  passenger_message  What the passenger says when the operator accepts their call.
  passenger_reply    What the passenger says after the operator talks to them.
  external_reply     What a person outside says after the operator talks to them.
                     These three take plain text OR a file name from the audio
                     folder (scenarios/audio): an audio file (.wav/.ogg/.mp3) is
                     played, a .txt file is shown as text.
  correct_responses  For the experimenter's analysis only: free text describing
  incorrect_responses  the responses you consider correct / incorrect.  Copied
                     unchanged into the data; the simulator never judges answers.
  video_delay        Per-scenario override of the video delay distribution.
  input_delay        Per-scenario override of the input delay distribution.
"""

import csv
import os
from typing import Any, Dict, List, Optional, Set, Tuple

from autosim import distributions, messages, settings
from autosim.scene import Scene, SceneError, load_scene

# Outcomes a task answer can trigger without writing any Python.
# (The actual behavior is implemented in autosim/tasks/question.py.)
KNOWN_OUTCOMES: Tuple[str, ...] = (
    "continue",           # resume autonomous driving on the current route
    "stay_stopped",       # keep holding position
    "change_lane_left",   # lane change, then continue
    "change_lane_right",
    "pull_over",          # pull over to the road edge and stop
    "manual_control",     # ask the operator to drive manually (inching)
    "reroute",            # follow the scene's first alternate route ("reroute:2" = second)
    "none",               # only record the answer
)

SEVERITIES: Tuple[str, ...] = ("info", "warning", "critical")

# Columns with a defined meaning (status columns are added below).
_BASE_COLUMNS: Set[str] = {
    "scenario_id", "group", "description", "scene", "time_of_day", "weather", "lights",
    "target_speed_mph", "start_stopped", "route_end", "alerts", "hidden_buttons",
    "task_trigger", "task_prompt", "task_options", "task_alert", "task_script",
    "passenger_message", "passenger_reply", "external_reply", "video_delay", "input_delay",
    "correct_responses", "incorrect_responses",
}
KNOWN_COLUMNS: Set[str] = _BASE_COLUMNS | set(settings.DEFAULT_VEHICLE_STATUS)

_TRUE_WORDS = ("1", "y", "yes", "true", "on")


class ScenarioFileError(ValueError):
    """Raised with a list of every problem found in a scenario file."""

    def __init__(self, problems: List[str]) -> None:
        self.problems = problems
        super().__init__("Problems in scenario file:\n  - " + "\n  - ".join(problems))


class Alert:
    """A line in the Alerts panel."""

    def __init__(self, text: str, severity: str = "warning") -> None:
        self.text = text
        self.severity = severity if severity in SEVERITIES else "warning"

    def __repr__(self) -> str:
        return "Alert({!r}, {})".format(self.text, self.severity)

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Alert) and (self.text, self.severity) == (other.text, other.severity)


class TaskSpec:
    """Declarative task logic from the task_* columns."""

    def __init__(self) -> None:
        self.trigger: str = ""                         # "", "start", "<metres>", "<trigger name>"
        self.prompt: str = ""
        self.options: List[Tuple[str, str]] = []       # (button label, outcome)
        self.alert: str = ""
        self.alert_severity: str = "warning"
        self.script: str = ""                          # absolute path to a .py task script

    @property
    def has_question(self) -> bool:
        return bool(self.prompt and self.options)


class Scenario:
    """One row of the scenario file, with defaults filled in."""

    def __init__(self, scenario_id: str, row_number: int) -> None:
        self.scenario_id = scenario_id
        self.row_number = row_number                  # 1-based data row in the CSV
        self.group: str = ""
        self.description: str = ""
        self.scene_file: str = ""                     # "" = default scene
        self.scene: Scene = Scene.default()
        self.time_of_day: str = settings.DEFAULT_TIME_OF_DAY
        self.weather: str = settings.DEFAULT_WEATHER
        self.lights: str = settings.DEFAULT_LIGHTS
        self.target_speed_mph: float = settings.AUTONOMOUS_TARGET_SPEED_MPH
        self.start_stopped: bool = False
        self.route_end: str = "customer"
        self.alerts: List[Alert] = []
        self.hidden_buttons: Set[str] = set()
        # Status values: raw specs (numbers, text or distribution specs).
        # They are sampled when the scenario starts -- see resolve_status().
        self.status_specs: Dict[str, Any] = dict(settings.DEFAULT_VEHICLE_STATUS)
        self.task = TaskSpec()
        self.base_folder: str = ""                    # folder of the CSV (for audio files)
        self.passenger_message: str = ""
        self.passenger_reply: str = ""
        self.external_reply: str = ""
        self.video_delay_spec: str = ""
        self.input_delay_spec: str = ""
        # Experimenter's notes on correct / incorrect responses (never evaluated).
        self.correct_responses: str = ""
        self.incorrect_responses: str = ""
        self.extra: Dict[str, str] = {}

    def resolve_status(self, rng=None) -> Dict[str, Any]:
        """Draw concrete values for every status field (numbers rounded sensibly)."""
        values: Dict[str, Any] = {}
        for key, spec in self.status_specs.items():
            value = distributions.sample_value(spec, rng)
            if isinstance(value, float):
                value = round(value, 1)
            values[key] = value
        return values

    def __repr__(self) -> str:
        return "Scenario({}, row {}, scene={})".format(self.scenario_id, self.row_number, self.scene.source)


# =============================================================================
# Loading
# =============================================================================

def load_scenarios(csv_path: str, group: Optional[str] = None) -> List[Scenario]:
    """Read a scenario CSV file.  Raises ScenarioFileError listing every problem.

    ``group`` (optional) keeps only rows whose ``group`` column matches.
    """
    if not os.path.isfile(csv_path):
        raise ScenarioFileError(["File not found: {}".format(csv_path)])

    with open(csv_path, "r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ScenarioFileError(["{} is empty".format(csv_path)])
        rows = [
            {_normalize_column(k): (v or "").strip() for k, v in row.items() if k is not None}
            for row in reader
        ]

    problems: List[str] = []
    base_folder = os.path.dirname(os.path.abspath(csv_path))
    columns = set(rows[0].keys()) if rows else set()
    unknown = sorted(c for c in columns if c and c not in KNOWN_COLUMNS)
    if unknown:
        print("[scenario] Note: extra columns kept for task scripts: {}".format(", ".join(unknown)))

    scenarios: List[Scenario] = []
    seen_ids: Set[str] = set()
    for index, row in enumerate(rows):
        if not any(row.values()):
            continue  # blank line
        row_number = index + 1
        try:
            scenario = _scenario_from_row(row, row_number, base_folder, problems)
        except Exception as error:  # keep collecting problems from other rows
            problems.append("row {}: {}".format(row_number, error))
            continue
        if scenario is None:
            continue
        if scenario.scenario_id in seen_ids:
            problems.append("row {}: duplicate scenario_id '{}'".format(row_number, scenario.scenario_id))
        seen_ids.add(scenario.scenario_id)
        scenarios.append(scenario)

    if problems:
        raise ScenarioFileError(problems)
    if group:
        scenarios = [s for s in scenarios if s.group == group]
    if not scenarios:
        raise ScenarioFileError(["No scenarios found in {}{}".format(
            csv_path, " for group '{}'".format(group) if group else "")])
    return scenarios


def _scenario_from_row(
    row: Dict[str, str], row_number: int, base_folder: str, problems: List[str]
) -> Optional[Scenario]:
    where = "row {}".format(row_number)

    def cell(name: str) -> str:
        return row.get(name, "")

    scenario_id = cell("scenario_id")
    if not scenario_id:
        problems.append("{}: scenario_id is empty".format(where))
        return None
    scenario = Scenario(scenario_id, row_number)
    where = "row {} ({})".format(row_number, scenario_id)

    scenario.base_folder = base_folder
    scenario.group = cell("group")
    scenario.description = cell("description")

    # --- scene -----------------------------------------------------------
    if cell("scene"):
        path = _find_file(cell("scene"), base_folder, "scenes")
        if path is None:
            problems.append("{}: scene file '{}' not found".format(where, cell("scene")))
        else:
            try:
                scenario.scene = load_scene(path)
                scenario.scene_file = path
            except SceneError as error:
                problems.append("{}: {}".format(where, error))

    # --- environment -----------------------------------------------------
    for column, presets in (("time_of_day", settings.SUN_PRESETS),
                            ("weather", settings.WEATHER_PRESETS),
                            ("lights", dict(settings.LIGHT_PRESETS, auto=None))):
        value = cell(column).lower()
        if value:
            if value not in presets:
                problems.append("{}: {} '{}' is not a preset (choose from {})".format(
                    where, column, value, ", ".join(sorted(presets))))
            else:
                setattr(scenario, column, value)

    # --- driving -----------------------------------------------------------
    if cell("target_speed_mph"):
        try:
            scenario.target_speed_mph = float(cell("target_speed_mph"))
        except ValueError:
            problems.append("{}: target_speed_mph must be a number".format(where))
    scenario.start_stopped = cell("start_stopped").lower() in _TRUE_WORDS
    if cell("route_end"):
        route_end = cell("route_end").lower()
        if route_end not in ("customer", "depot"):
            problems.append("{}: route_end must be 'customer' or 'depot'".format(where))
        else:
            scenario.route_end = route_end

    # --- interface content -----------------------------------------------
    scenario.alerts = [parse_alert(text) for text in _split_list(cell("alerts"))]
    scenario.hidden_buttons = set(item.lower() for item in _split_list(cell("hidden_buttons")))

    for key in settings.DEFAULT_VEHICLE_STATUS:
        value = cell(key)
        if not value:
            continue
        if distributions.is_spec(value):
            try:
                distributions.parse_spec(value)
            except ValueError as error:
                problems.append("{}: column {}: {}".format(where, key, error))
                continue
        scenario.status_specs[key] = value

    # --- task ----------------------------------------------------------------
    task = scenario.task
    task.trigger = cell("task_trigger")
    task.prompt = cell("task_prompt")
    if cell("task_alert"):
        alert = parse_alert(cell("task_alert"))
        task.alert, task.alert_severity = alert.text, alert.severity
    for item in _split_list(cell("task_options")):
        label, _, outcome = item.partition("=")
        label, outcome = label.strip(), (outcome.strip().lower() or "none")
        if outcome.split(":")[0] not in KNOWN_OUTCOMES:
            problems.append("{}: unknown outcome '{}' for option '{}' (choose from {})".format(
                where, outcome, label, ", ".join(KNOWN_OUTCOMES)))
        task.options.append((label, outcome))
    if task.options and not task.prompt:
        problems.append("{}: task_options given but task_prompt is empty".format(where))
    if task.trigger and task.trigger.lower() != "start":
        if not _is_number(task.trigger) and task.trigger not in scenario.scene.triggers:
            problems.append("{}: task_trigger '{}' is not 'start', a distance, or a trigger in the scene".format(
                where, task.trigger))
    if cell("task_script"):
        path = _find_file(cell("task_script"), base_folder, "scripts")
        if path is None:
            problems.append("{}: task script '{}' not found".format(where, cell("task_script")))
        else:
            task.script = path

    # --- communication -----------------------------------------------------
    scenario.passenger_message = cell("passenger_message")
    scenario.passenger_reply = cell("passenger_reply")
    scenario.external_reply = cell("external_reply")
    for column in ("passenger_message", "passenger_reply", "external_reply"):
        value = cell(column)
        if value and messages.looks_like_file(value) and messages.find_message_file(value, base_folder) is None:
            problems.append("{}: {} file '{}' not found in the audio folder".format(where, column, value))
    scenario.correct_responses = cell("correct_responses")
    scenario.incorrect_responses = cell("incorrect_responses")

    # --- link overrides ------------------------------------------------------
    for column, attribute in (("video_delay", "video_delay_spec"), ("input_delay", "input_delay_spec")):
        value = cell(column)
        if value:
            try:
                distributions.parse_spec(value)
                setattr(scenario, attribute, value)
            except ValueError as error:
                problems.append("{}: column {}: {}".format(where, column, error))

    scenario.extra = {k: v for k, v in row.items() if k and k not in KNOWN_COLUMNS}
    return scenario


# =============================================================================
# Helpers
# =============================================================================

def parse_alert(text: str) -> Alert:
    """'critical: Low battery' -> Alert('Low battery', 'critical')."""
    head, separator, tail = text.partition(":")
    if separator and head.strip().lower() in SEVERITIES:
        return Alert(tail.strip(), head.strip().lower())
    return Alert(text.strip(), "warning")


def _split_list(text: str) -> List[str]:
    return [piece.strip() for piece in text.split("|") if piece.strip()] if text else []


def _normalize_column(name: str) -> str:
    return name.strip().lower().replace(" ", "_")


def _is_number(text: str) -> bool:
    try:
        float(text)
        return True
    except ValueError:
        return False


def _find_file(name: str, base_folder: str, subfolder: str) -> Optional[str]:
    """Look for ``name`` as given, next to the CSV, and in the CSV's subfolder."""
    candidates = [name, os.path.join(base_folder, name), os.path.join(base_folder, subfolder, name)]
    for candidate in candidates:
        if os.path.isfile(candidate):
            return os.path.abspath(candidate)
    return None
