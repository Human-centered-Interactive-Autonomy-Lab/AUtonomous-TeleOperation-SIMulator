"""
tasks.py -- scenario-specific behaviour ("task logic").

There are two ways to give a scenario special behaviour:

1. Without programming -- the ``task_*`` columns of the scenario CSV.
   Example: the car drives 55 m, stops, and asks "Is the left lane open?"
   with buttons Yes (continue) and No (change into the left lane):

       task_trigger = 55
       task_prompt  = Is the left lane open?
       task_options = Yes=continue | No=change_lane_left
       task_correct = No

   This is handled by ``QuestionTask`` below.

2. With a small Python "task script" (``task_script`` column) for anything
   else.  A task script is a .py file that defines a subclass of
   ``TaskScript`` and overrides the hooks it needs::

       from autosim.tasks import TaskScript

       class Task(TaskScript):
           def on_start(self, ctx):
               ctx.add_alert("GPS signal lost", "critical")

           def on_tick(self, ctx, dt):
               if ctx.elapsed > 10 and not self.asked:
                   self.asked = True
                   ctx.stop_vehicle("Needs help")
                   ctx.ask("Can I continue?", ["Yes=continue", "No=pull_over"])

           def on_answer(self, ctx, label):
               ctx.log("my_task_answer", label=label)

   ``ctx`` is a ``TaskContext``: the complete, stable API a task may use
   (vehicle commands, questions, alerts, notifications, spawning actors,
   logging).  See docs/TASK_SCRIPTS.md and scenarios/scripts/ for examples.

Both kinds can be combined in one scenario; both receive every hook.
"""

import importlib.util
import inspect
import os
import random
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple, Union

from autosim.scene import Placement
from autosim.simulation import DrivingMode, VehicleState

OptionList = Sequence[Union[str, Tuple[str, str]]]

# A question task also fires when the AV has stopped by itself within this
# distance before its trigger point.
TRIGGER_STOPPED_NEARBY_METERS = 30.0


class TaskContext:
    """Everything a task may read or do.  Created by the scenario session.

    Read-only information
    ---------------------
    scenario   the Scenario (row of the CSV); ``scenario.extra`` holds your own columns
    state      the latest VehicleState (speed, position, mode, route progress...)
    status     dict of the values shown in the Status panel
    elapsed    seconds since the scenario started
    rng        random.Random to use for reproducible randomness
    """

    def __init__(self, session: Any) -> None:
        self._session = session
        self.scenario = session.scenario
        self.rng: random.Random = session.rng

    # ---------------------------------------------------------------- info

    @property
    def state(self) -> VehicleState:
        return self._session.vehicle_state

    @property
    def status(self) -> Dict[str, Any]:
        return self._session.status_values

    @property
    def elapsed(self) -> float:
        return self._session.elapsed

    def distance_to(self, route_m: float) -> float:
        """Metres along the route from the car to ``route_m`` (negative = already passed)."""
        return self._session.simulation.distance_to_route_point(route_m)

    def trigger_route_m(self, trigger_name: str) -> Optional[float]:
        """Route distance of a named trigger in the scene file."""
        return self._session.simulation.trigger_route_m(self.scenario.scene, trigger_name)

    # ------------------------------------------------------------ the AV
    # These act immediately (they are the AV's own decisions, not operator
    # commands, so the simulated input delay does not apply).

    def stop_vehicle(self, reason: str = "") -> None:
        """The AV brakes gently to a stop and waits.  (No automatic alert is
        shown -- add one with ``add_alert`` or ask a question.)"""
        self._session.simulation.soft_stop("Task: " + reason)

    def resume_vehicle(self) -> None:
        self._session.simulation.resume_autonomy()

    def pull_over(self) -> None:
        self._session.simulation.pull_over()

    def change_lane(self, direction: str) -> bool:
        return self._session.simulation.change_lane(direction)

    def set_target_speed(self, mph: float) -> None:
        self._session.simulation.set_target_speed(mph)

    def perform_outcome(self, outcome: str) -> None:
        """Run a named outcome (continue, pull_over, change_lane_left, ...) right now."""
        self._session.perform_outcome(outcome, delayed=False)

    # ------------------------------------------------------- the world

    def spawn(self, blueprint: str, route_m: float, lane: int = 0, lateral_m: float = 0.0,
              yaw: float = 0.0) -> Any:
        """Spawn an actor at a route position.  Returns a handle for ``destroy``."""
        at = Placement({"route_m": route_m, "lane": lane, "lateral_m": lateral_m, "yaw": yaw})
        return self._session.simulation.spawn_actor(blueprint, at)

    def spawn_walker(self, start: Dict[str, float], goal: Dict[str, float], speed_mps: float = 1.4,
                     blueprint: str = "") -> Any:
        """Spawn a pedestrian walking from ``start`` to ``goal``.

        Both are position dictionaries as in scene files, e.g.
        ``{"route_m": 40, "lateral_m": -9}``.
        """
        return self._session.simulation.spawn_walker(Placement(start), Placement(goal), speed_mps, blueprint)

    def destroy(self, handle: Any) -> None:
        self._session.simulation.destroy_actor(handle)

    # ----------------------------------------------------- the interface

    def ask(self, prompt: str, options: OptionList,
            on_answer: Optional[Callable[[str], None]] = None) -> None:
        """Show a question in the Decision panel.

        ``options`` are labels ("Yes") or "Label=outcome" strings / (label,
        outcome) pairs; the outcome runs when the operator clicks it.  The
        answer and its timing are logged automatically.
        """
        parsed = []
        for option in options:
            if isinstance(option, tuple):
                parsed.append((option[0], option[1]))
            else:
                label, _, outcome = option.partition("=")
                parsed.append((label.strip(), outcome.strip() or "none"))
        self._session.ask(prompt, parsed, on_answer)

    def add_alert(self, text: str, severity: str = "warning") -> None:
        self._session.add_alert(text, severity)

    def remove_alert(self, text: str) -> None:
        self._session.remove_alert(text)

    def notify(self, title: str, body: str = "", severity: str = "info",
               actions: Optional[List[Tuple[str, Callable[[], None]]]] = None) -> None:
        """Notification (bell + banner).  ``actions`` = [(button label, function), ...]."""
        self._session.notify(title, body, severity, actions)

    def set_status(self, key: str, value: Any) -> None:
        """Change a value in the Status panel (e.g. ``set_status('battery_percent', 12)``)."""
        self._session.set_status(key, value)

    def say(self, speaker: str, message: str) -> None:
        """Someone speaks.  ``message`` is text or a file in the audio folder
        (audio files are played, .txt files are shown)."""
        self._session.say(speaker, message)

    def feedback(self, text: str) -> None:
        """Short fading message in the Actions panel."""
        self._session.feedback(text)

    # ---------------------------------------------------------- data

    def log(self, event: str, **details: Any) -> None:
        """Write a row to the event log."""
        self._session.log(event, **details)

    def record(self, name: str, value: Any) -> None:
        """Add a column to this scenario's summary row (e.g. ``record('gap_s', 4.2)``).

        Record measurements, not verdicts: deciding which responses are
        correct is left to the experimenter."""
        self._session.summary_extra[name] = value


class TaskScript:
    """Base class for task scripts.  Override any hook; all are optional."""

    def __init__(self) -> None:
        self.asked = False   # handy flag for the common "ask once" pattern

    def on_start(self, ctx: TaskContext) -> None:
        """Scenario has started (the world is loaded and the interface is visible)."""

    def on_tick(self, ctx: TaskContext, dt: float) -> None:
        """Called every frame (about 30 times per second)."""

    def on_action(self, ctx: TaskContext, action_id: str) -> None:
        """The operator pressed an action button (e.g. 'pull_over', 'honk')."""

    def on_answer(self, ctx: TaskContext, label: str) -> None:
        """The operator answered a question in the Decision panel."""

    def on_mode_change(self, ctx: TaskContext, old_mode: str, new_mode: str) -> None:
        """The driving mode changed (see simulation.DrivingMode)."""

    def on_end(self, ctx: TaskContext) -> None:
        """Scenario is ending ('End supervision' pressed)."""


class QuestionTask(TaskScript):
    """The no-programming task from the task_* CSV columns (see module docstring).

    1. Waits for the trigger (scenario start, a route distance, or a named
       trigger point in the scene).
    2. Then adds the task alert, stops the car if a question is configured,
       and asks the question.
    3. The answer's outcome is run by the session (continue, change lane...).
    """

    def __init__(self) -> None:
        super().__init__()
        self.trigger_route_m: Optional[float] = None
        self.fired = False

    def on_start(self, ctx: TaskContext) -> None:
        spec = ctx.scenario.task
        trigger = spec.trigger.strip()
        if not trigger or trigger.lower() == "start":
            self._fire(ctx)
            return
        try:
            self.trigger_route_m = float(trigger)
        except ValueError:
            self.trigger_route_m = ctx.trigger_route_m(trigger)
        if self.trigger_route_m is None:
            ctx.log("task_error", message="unknown trigger '{}'".format(trigger))
            self._fire(ctx)

    def on_tick(self, ctx: TaskContext, dt: float) -> None:
        if self.fired or self.trigger_route_m is None:
            return
        # Start braking early enough to stop at the trigger point.
        speed = ctx.state.speed_mps
        braking_distance = speed * speed / (2 * 4.0) + 1.0
        distance = ctx.distance_to(self.trigger_route_m)
        # Also fire if the AV already stopped by itself shortly before the
        # trigger point (e.g. its own obstacle detection braked first).
        stopped_nearby = (ctx.state.mode == DrivingMode.HOLDING
                          and distance <= TRIGGER_STOPPED_NEARBY_METERS)
        if distance <= braking_distance or stopped_nearby:
            self._fire(ctx)

    def _fire(self, ctx: TaskContext) -> None:
        self.fired = True
        spec = ctx.scenario.task
        ctx.log("task_triggered", trigger=spec.trigger or "start")
        if spec.alert:
            ctx.add_alert(spec.alert, spec.alert_severity)
        if spec.has_question:
            if ctx.state.mode in (DrivingMode.AUTONOMOUS, DrivingMode.STOPPING, DrivingMode.HOLDING):
                ctx.stop_vehicle(spec.prompt)
            ctx.ask(spec.prompt, spec.options)


def load_task_script(path: str) -> TaskScript:
    """Import a task script file and return an instance of its TaskScript subclass."""
    module_name = "autosim_task_" + os.path.splitext(os.path.basename(path))[0]
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError("Cannot load task script {}".format(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # type: ignore[union-attr]
    classes = [
        value for value in vars(module).values()
        if inspect.isclass(value) and issubclass(value, TaskScript)
        and value not in (TaskScript, QuestionTask) and value.__module__ == module.__name__
    ]
    if not classes:
        raise ImportError("{} defines no subclass of autosim.tasks.TaskScript".format(path))
    return classes[0]()


def build_tasks(scenario: Any) -> List[TaskScript]:
    """All task objects for a scenario (question task and/or script)."""
    tasks: List[TaskScript] = []
    spec = scenario.task
    if spec.has_question or spec.alert:
        tasks.append(QuestionTask())
    if spec.script:
        tasks.append(load_task_script(spec.script))
    return tasks
