"""
session.py -- runs ONE scenario: the link between interface, vehicle and tasks.

The ScenarioSession owns everything the operator sees that is not pixels:
alerts, the pending decision, notifications, status values, route progress,
feedback messages.  The interface (interface.py) draws this state and calls
the session's methods when the operator clicks; the session forwards vehicle
commands through the simulated input delay (link.CommandLink) to the
simulation, runs the task logic, and logs every event.

    interface.py --clicks--> ScenarioSession --commands (delayed)--> Simulation
         ^                        |    ^
         +------ draws state -----+    +---- hooks ---- tasks (TaskScript)
"""

import random
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from autosim import settings
from autosim.audio import Audio
from autosim.messages import resolve_message
from autosim.datalog import DataLogger
from autosim.link import CommandLink, VideoLink
from autosim.scenario import Alert, Scenario, parse_alert
from autosim.scene import Placement
from autosim.simulation import DrivingMode, Simulation, VehicleState
from autosim.tasks import TaskContext, TaskScript, build_tasks

METERS_PER_MILE = 1609.344
ASSUMED_TRIP_SPEED_MPH = 25.0   # used to show remaining trip time in the Route panel
TASK_STOP_PREFIX = "Task:"      # stop reasons from task logic start with this (no extra alert)


class Decision:
    """A question from the AV in the Decision panel."""

    def __init__(self, prompt: str, options: List[Tuple[str, str]],
                 on_answer: Optional[Callable[[str], None]], shown_at: float) -> None:
        self.prompt = prompt
        self.options = options            # (label, outcome)
        self.on_answer = on_answer
        self.shown_at = shown_at          # scenario time (s)
        self.answer: Optional[str] = None
        self.answered_at: Optional[float] = None


class Notification:
    """An entry behind the bell icon."""

    def __init__(self, title: str, body: str, severity: str, created_at: float,
                 actions: Optional[List[Tuple[str, Callable[[], None]]]] = None) -> None:
        self.title = title
        self.body = body
        self.severity = severity
        self.created_at = created_at      # scenario time (s)
        self.actions = actions or []
        self.read = False
        self.handled = False


class Feedback:
    """A short fading message in the Actions panel."""

    def __init__(self, text: str, created: float, duration: float) -> None:
        self.text = text
        self.created = created            # wall clock (time.monotonic)
        self.duration = duration


class ScenarioSession:
    """See module docstring.  Created by the experiment runner for each scenario."""

    def __init__(self, scenario: Scenario, scenario_index: int, simulation: Simulation,
                 logger: DataLogger, command_link: CommandLink, video_link: VideoLink,
                 audio: Audio, rng: random.Random) -> None:
        self.scenario = scenario
        self.scenario_index = scenario_index
        self.simulation = simulation
        self.logger = logger
        self.command_link = command_link
        self.video_link = video_link
        self.audio = audio
        self.rng = rng

        self.start_time = time.monotonic()
        self.vehicle_state = VehicleState()
        self.status_values: Dict[str, Any] = {}
        self.alerts: List[Alert] = []
        self.decision: Optional[Decision] = None
        self.decision_queue: List[Decision] = []
        self.decision_history: List[Decision] = []
        self.notifications: List[Notification] = []
        self.feedback_messages: List[Feedback] = []
        self.transcript: List[Tuple[str, str]] = []       # (speaker, text)

        self.manual_control_active = False
        self.hazard_lights_on = False
        self.return_to_depot = scenario.route_end == "depot"
        self.call_partner = ""                             # "" = no active call
        self.call_muted = False
        self.talking_to = ""                               # push-to-talk target
        self.end_requested = False

        self.route_start_progress_m = 0.0
        self.route_total_m = 0.0
        self.route_complete = False
        self.route_complete_walker: Any = None

        self.counts: Dict[str, int] = {}
        self.action_sequence: List[str] = []                # "action@seconds", in order
        self.first_action: Tuple[str, Optional[float]] = ("", None)
        self.manual_seconds = 0.0
        self.summary_extra: Dict[str, Any] = {}
        self._last_mode = ""
        self._last_collisions = 0

        self.tasks: List[TaskScript] = []
        self.context = TaskContext(self)

    # ======================================================================
    # Lifecycle
    # ======================================================================

    @property
    def elapsed(self) -> float:
        return time.monotonic() - self.start_time

    def start(self) -> None:
        """Called once the world is loaded and the interface is about to show."""
        self.start_time = time.monotonic()
        self.status_values = self.scenario.resolve_status(self.rng)
        self.route_total_m = float(self.status_values.get("route_total_miles", 5.0)) * METERS_PER_MILE
        percent = float(self.status_values.get("route_progress_percent", 0.0))
        self.route_start_progress_m = self.route_total_m * min(max(percent, 0.0), 100.0) / 100.0
        for alert in self.scenario.alerts:
            self.add_alert(alert.text, alert.severity, log=False)
        self.log("scenario_start", scene=self.scenario.scene.source,
                 status_shown=self.status_values,
                 alerts=[a.text for a in self.scenario.alerts])
        if self.scenario.passenger_message:
            self.notify("Passenger requests to talk", "Accept to talk with the passenger.", "warning",
                        [("Accept", self._accept_passenger_call)])
        try:
            self.tasks = build_tasks(self.scenario)
        except Exception as error:   # a broken task script must not crash an experiment
            self.log("task_error", message=str(error))
            print("[session] Task script error: {}".format(error))
            self.tasks = []
        self._call_tasks("on_start")

    def update(self, dt: float, manual_keys: Optional[Dict[str, bool]] = None) -> None:
        """One frame: deliver due commands, drive, run tasks, collect video."""
        now = time.monotonic()
        for name in self.command_link.update(now):
            self.log("command_executed", command=name)
        if self.manual_control_active:
            if manual_keys is not None:
                self.command_link.send_continuous("manual", self._manual_input_from_keys(manual_keys), now)
            delayed_input = self.command_link.continuous_value("manual", now)
            if delayed_input is not None:
                self.simulation.apply_manual_input(*delayed_input)
            self.manual_seconds += dt

        self.vehicle_state = self.simulation.tick(dt)
        self._watch_vehicle()
        self._call_tasks("on_tick", dt)
        for camera, (capture_time, frame) in self.simulation.camera_frames().items():
            self.video_link.push(camera, capture_time, frame)

    def end(self) -> Dict[str, Any]:
        """Finish the scenario; returns the summary row for the data file."""
        self._call_tasks("on_end")
        duration = self.elapsed
        self.log("scenario_end", duration_s=round(duration, 2))
        first_label, first_time = self.first_action
        decisions = self.decision_history + ([self.decision] if self.decision else [])
        decisions += self.decision_queue
        main = decisions[0] if decisions else None
        # Raw facts only.  Whether a response was right is for the experimenter
        # to decide (see the correct_responses / incorrect_responses columns).
        row: Dict[str, Any] = {
            "scenario_index": self.scenario_index,
            "scenario_id": self.scenario.scenario_id,
            "group": self.scenario.group,
            "scene": self.scenario.scene.source,
            "duration_s": round(duration, 2),
            "first_action": first_label,
            "time_to_first_action_s": "" if first_time is None else round(first_time, 2),
            "decision_prompt": main.prompt if main else "",
            "decision_shown_at_s": round(main.shown_at, 2) if main else "",
            "decision_answer": (main.answer or "(unanswered)") if main else "",
            "decision_latency_s": ("" if not main or main.answered_at is None
                                   else round(main.answered_at - main.shown_at, 2)),
            "decisions_total": len(decisions),
            "decisions_unanswered": sum(1 for d in decisions if d.answer is None),
            "action_sequence": " ".join(self.action_sequence),
            "commands_total": sum(self.counts.values()),
            "emergency_stops": self.counts.get("emergency_stop", 0),
            "pull_overs": self.counts.get("pull_over", 0),
            "manual_control_s": round(self.manual_seconds, 1),
            "collisions": self.vehicle_state.collision_count,
            "route_complete": self.route_complete,
            "final_mode": self.vehicle_state.mode,
            "video_delay": self.video_link.delay.describe(),
            "input_delay": self.command_link.delay.describe(),
            "correct_responses": self.scenario.correct_responses,
            "incorrect_responses": self.scenario.incorrect_responses,
        }
        for key, value in self.status_values.items():
            row["shown_" + key] = value
        row.update(self.summary_extra)
        return row

    # ======================================================================
    # Operator input (called by the interface)
    # ======================================================================

    def record_action(self, action_id: str, **details: Any) -> None:
        """Log an operator action and tell the tasks about it."""
        self.counts[action_id] = self.counts.get(action_id, 0) + 1
        if self.first_action[1] is None:
            self.first_action = (action_id, self.elapsed)
        self.action_sequence.append("{}@{:.1f}".format(action_id, self.elapsed))
        self.log("operator_action", action=action_id, **details)
        self._call_tasks("on_action", action_id)

    def send_vehicle_command(self, name: str, function: Callable[[], None], immediate: bool = False) -> None:
        """Send a command to the vehicle through the simulated input delay."""
        self.command_link.send(name, function, time.monotonic(), immediate)

    def answer(self, label: str) -> None:
        """The operator clicked an answer in the Decision panel."""
        decision = self.decision
        if decision is None or decision.answer is not None:
            return
        decision.answer = label
        decision.answered_at = self.elapsed
        outcome = dict(decision.options).get(label, "none")
        self.record_action("answer", prompt=decision.prompt, answer=label, outcome=outcome,
                           latency_s=round(decision.answered_at - decision.shown_at, 2))
        self.perform_outcome(outcome, delayed=True)
        if decision.on_answer is not None:
            decision.on_answer(label)
        self._call_tasks("on_answer", label)
        self.decision_history.append(decision)
        self.decision = self.decision_queue.pop(0) if self.decision_queue else None

    def perform_outcome(self, outcome: str, delayed: bool = True) -> None:
        """Run a named outcome (see scenario.KNOWN_OUTCOMES)."""
        simulation = self.simulation
        name, _, argument = outcome.partition(":")

        def run(function: Callable[[], Any]) -> None:
            if delayed:
                self.send_vehicle_command(outcome, function)
            else:
                function()

        if name == "continue":
            run(simulation.resume_autonomy)
        elif name in ("change_lane_left", "change_lane_right"):
            direction = name.rsplit("_", 1)[1]

            def change_lane() -> None:
                if simulation.change_lane(direction):
                    simulation.resume_autonomy()
                else:
                    self.feedback("No lane to the {}.".format(direction))
            run(change_lane)
        elif name == "pull_over":
            run(simulation.pull_over)
        elif name == "manual_control":
            self.feedback("Please take manual control and drive around.")
        elif name == "reroute":
            index = int(argument) - 1 if argument.isdigit() else 0
            run(lambda: simulation.reroute(index) and simulation.resume_autonomy())
        # "stay_stopped" and "none" deliberately do nothing.

    def request_manual_control(self) -> bool:
        """Manual control is only allowed below the configured speed."""
        speed = self.vehicle_state.speed_mph
        if speed >= settings.MANUAL_CONTROL_MAX_ENTRY_SPEED_MPH:
            self.feedback("Slow below {:.0f} mph to take manual control.".format(
                settings.MANUAL_CONTROL_MAX_ENTRY_SPEED_MPH))
            self.log("manual_control_refused", speed_mph=round(speed, 1))
            return False
        self.manual_control_active = True
        self.send_vehicle_command("manual_control", lambda: self.simulation.set_manual_control(True))
        self.feedback("Manual control: W/A/S/D or arrow keys.")
        return True

    def resume_autonomy(self) -> None:
        if self.manual_control_active:
            self.manual_control_active = False
            self.send_vehicle_command("manual_off", lambda: self.simulation.set_manual_control(False))
        self.send_vehicle_command("resume_av", self.simulation.resume_autonomy)
        self.feedback("Resuming AV mode.")

    def start_talking(self, who: str) -> None:
        self.talking_to = who
        self.log("talk_start", to=who)

    def stop_talking(self) -> None:
        who, self.talking_to = self.talking_to, ""
        if not who:
            return
        self.log("talk_end", to=who)
        reply = self.scenario.passenger_reply if who == "passenger" else self.scenario.external_reply
        if who == "passenger" and not reply and self.scenario.passenger_message:
            reply = "Okay, thank you."
        if reply:
            self.say("Passenger" if who == "passenger" else "Person outside", reply)

    def say(self, speaker: str, message: str) -> None:
        """Someone speaks: plays an audio file and/or shows the text.

        ``message`` is plain text or a file in the audio folder (see
        audio.resolve_message): audio files are played, .txt files shown.
        """
        text, audio_path = resolve_message(message, self.scenario.base_folder)
        if audio_path:
            self.audio.play_file(audio_path)
        shown = text or "(audio message)"
        self.transcript.append((speaker, shown))
        self.feedback('{}: "{}"'.format(speaker, shown), seconds=max(4.0, len(shown) / 12.0))
        self.log("speech", speaker=speaker, text=shown, audio=audio_path)

    # ======================================================================
    # Things tasks and actions use
    # ======================================================================

    def ask(self, prompt: str, options: List[Tuple[str, str]],
            on_answer: Optional[Callable[[str], None]] = None) -> None:
        decision = Decision(prompt, options, on_answer, self.elapsed)
        self.log("decision_shown", prompt=prompt, options=[label for label, _ in options])
        if self.decision is None:
            self.decision = decision
        else:
            self.decision_queue.append(decision)
        self.audio.play("chime")

    def add_alert(self, text: str, severity: str = "warning", log: bool = True) -> None:
        alert = parse_alert(text) if ":" in text and severity == "warning" else Alert(text, severity)
        if alert in self.alerts:
            return
        self.alerts.insert(0, alert)   # newest on top
        if log:
            self.log("alert_shown", text=alert.text, severity=alert.severity)

    def remove_alert(self, text: str) -> None:
        self.alerts = [alert for alert in self.alerts if alert.text != text]

    def notify(self, title: str, body: str = "", severity: str = "info",
               actions: Optional[List[Tuple[str, Callable[[], None]]]] = None) -> Notification:
        notification = Notification(title, body, severity, self.elapsed, actions)
        self.notifications.insert(0, notification)
        self.log("notification", title=title, severity=severity)
        self.audio.play("chime")
        return notification

    def set_status(self, key: str, value: Any) -> None:
        self.status_values[key] = value
        self.log("status_changed", key=key, value=value)

    def feedback(self, text: str, seconds: float = settings.ACTION_FEEDBACK_SECONDS) -> None:
        self.feedback_messages.append(Feedback(text, time.monotonic(), seconds))
        self.feedback_messages = self.feedback_messages[-3:]

    def log(self, event: str, **details: Any) -> None:
        self.logger.event(event, self.scenario_index, self.scenario.scenario_id, self.elapsed,
                          details, self.vehicle_state.as_log_fields())

    # ======================================================================
    # Values the interface draws
    # ======================================================================

    def route_progress(self) -> Tuple[float, float]:
        """(miles travelled, total miles) for the Route panel."""
        traveled = self.route_start_progress_m + self.vehicle_state.route_traveled_m
        if self.route_complete:
            traveled = self.route_total_m
        traveled = min(traveled, self.route_total_m)
        return traveled / METERS_PER_MILE, self.route_total_m / METERS_PER_MILE

    def route_minutes(self) -> Tuple[float, float]:
        """(minutes elapsed, total minutes) of the trip at a nominal trip speed."""
        traveled, total = self.route_progress()
        return traveled / ASSUMED_TRIP_SPEED_MPH * 60, total / ASSUMED_TRIP_SPEED_MPH * 60

    def route_complete_text(self) -> str:
        return (settings.ROUTE_COMPLETE_DEPOT_TEXT if self.return_to_depot
                else settings.ROUTE_COMPLETE_NEXT_CUSTOMER_TEXT)

    def unread_notifications(self) -> int:
        return sum(1 for n in self.notifications if not n.read)

    def active_feedback(self) -> List[Tuple[str, float]]:
        """(text, opacity 0..1) of feedback messages still on screen."""
        now = time.monotonic()
        visible = []
        for message in self.feedback_messages:
            age = now - message.created
            if age < message.duration:
                fade_start = message.duration * 0.6
                opacity = 1.0 if age < fade_start else 1.0 - (age - fade_start) / (message.duration - fade_start)
                visible.append((message.text, max(0.0, opacity)))
        return visible

    # ======================================================================
    # Internals
    # ======================================================================

    def _watch_vehicle(self) -> None:
        state = self.vehicle_state
        if state.mode != self._last_mode:
            old = self._last_mode
            self._last_mode = state.mode
            self.log("mode_change", old=old, new=state.mode, reason=state.stop_reason)
            # The AV stopped by itself (obstacle, collision): tell the operator why.
            # Stops requested by task logic come with their own alert/question.
            if (state.mode == DrivingMode.HOLDING and old == DrivingMode.STOPPING and state.stop_reason
                    and not state.stop_reason.startswith(TASK_STOP_PREFIX)):
                self.add_alert(state.stop_reason, "warning")
            if state.mode == DrivingMode.ROUTE_COMPLETE:
                self._on_route_complete()
            if old:
                self._call_tasks("on_mode_change", old, state.mode)
        if state.collision_count > self._last_collisions:
            self._last_collisions = state.collision_count
            self.log("collision", other=state.last_collision_with, total=state.collision_count)
            self.add_alert("Collision detected", "critical")
        if state.hazard_lights_on != self.hazard_lights_on:
            self.hazard_lights_on = state.hazard_lights_on

    def _on_route_complete(self) -> None:
        self.route_complete = True
        self.log("route_complete", return_to_depot=self.return_to_depot)
        if settings.SPAWN_PEDESTRIAN_AT_ROUTE_END and not self.return_to_depot:
            traveled = self.vehicle_state.route_traveled_m
            try:
                # A customer walks up along the sidewalk on the right.
                self.route_complete_walker = self.simulation.spawn_walker(
                    Placement({"route_m": traveled + 8, "lateral_m": 6.5}),
                    Placement({"route_m": traveled + 1, "lateral_m": 5.6}),
                    speed_mps=1.0,
                )
            except Exception as error:   # optional nicety; never fatal
                print("[session] Could not spawn pedestrian: {}".format(error))

    def _accept_passenger_call(self) -> None:
        self.record_action("accept_passenger_call")
        self.say("Passenger", self.scenario.passenger_message)

    def _manual_input_from_keys(self, keys: Dict[str, bool]) -> Tuple[float, float, float, bool]:
        forward, backward = keys.get("forward", False), keys.get("backward", False)
        left, right = keys.get("left", False), keys.get("right", False)
        speed = self.vehicle_state.speed_mph
        throttle, brake, reverse = 0.0, 0.0, False
        if forward and not backward:
            throttle = settings.MANUAL_MAX_THROTTLE if speed < settings.MANUAL_SPEED_LIMIT_MPH else 0.0
        elif backward and not forward:
            throttle, reverse = settings.MANUAL_MAX_THROTTLE, True
        else:
            brake = settings.MANUAL_COAST_BRAKE
        steer = (-settings.MANUAL_MAX_STEER if left else 0.0) + (settings.MANUAL_MAX_STEER if right else 0.0)
        return throttle, steer, brake, reverse

    def _call_tasks(self, hook: str, *arguments: Any) -> None:
        for task in self.tasks:
            try:
                getattr(task, hook)(self.context, *arguments)
            except Exception as error:   # report, but keep the experiment running
                self.log("task_error", hook=hook, message=repr(error))
                print("[session] Task {} failed in {}: {!r}".format(type(task).__name__, hook, error))
