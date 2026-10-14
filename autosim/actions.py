"""
actions.py -- every operator button and dialog, defined in one place.

To ADD a button:
    1. Write a handler ``def _my_action(session): ...`` below.  It receives
       the ScenarioSession (see session.py) and may return the name of a
       dialog to open.
    2. Add an ``Action(...)`` line to ACTIONS with a unique id, the label, the
       panel area it belongs to (its ``group``) and a style.
To REMOVE or REORDER buttons: delete or move lines in ACTIONS.
To HIDE a button in one scenario: list its id in the scenario's
``hidden_buttons`` column.

Groups (where the button is drawn):
    "actions"       list of commands in the left part of the Actions panel
    "drive"         big driving buttons in the right part of the Actions panel
    "end"           bottom-right corner of the Actions panel
    "map"           tool buttons on the Map panel
    "status"        button at the bottom of the Status panel

Styles: "normal", "danger" (red), "primary" (orange), "link" (text-like).
Kinds:  "click" (default), "toggle" (shows on/off), "hold" (push-to-talk:
        handler runs on press, ``release`` handler on release).
"""

from typing import Callable, Dict, List, Optional, Tuple

from autosim import settings

Handler = Callable[..., Optional[str]]


class Action:
    def __init__(self, action_id: str, label: str, group: str, handler: Handler,
                 style: str = "normal", kind: str = "click",
                 release: Optional[Handler] = None,
                 is_on: Optional[Callable[..., bool]] = None,
                 is_enabled: Optional[Callable[..., bool]] = None) -> None:
        self.action_id = action_id
        self.label = label
        self.group = group
        self.handler = handler
        self.style = style
        self.kind = kind
        self.release = release
        self.is_on = is_on
        self.is_enabled = is_enabled


class DialogOption:
    def __init__(self, label: str, handler: Handler, style: str = "normal") -> None:
        self.label = label
        self.handler = handler
        self.style = style


class Dialog:
    """A small window of choices opened by an action (e.g. "Who to contact?")."""

    def __init__(self, title: str, options: List[DialogOption], text: Optional[Callable[..., str]] = None) -> None:
        self.title = title
        self.options = options
        self.text = text            # optional function(session) -> body text


# =============================================================================
# Handlers.  Each receives the ScenarioSession.  Return a dialog name to open
# a dialog, or None.
# =============================================================================

def _emergency_stop(session) -> None:
    session.record_action("emergency_stop")
    session.manual_control_active = False
    session.send_vehicle_command("emergency_stop", session.simulation.emergency_stop,
                                 immediate=settings.EMERGENCY_STOP_IGNORES_INPUT_DELAY)
    session.feedback("EMERGENCY STOP sent.")


def _pull_over(session) -> None:
    session.record_action("pull_over")
    session.manual_control_active = False
    session.send_vehicle_command("pull_over", session.simulation.pull_over)
    session.feedback("Pulling over when safe.")


def _manual_control(session) -> None:
    session.record_action("manual_control")
    session.request_manual_control()


def _resume_av(session) -> None:
    session.record_action("resume_av")
    session.resume_autonomy()


def _end_supervision(session) -> None:
    # Always allowed -- an unanswered question is simply recorded as unanswered.
    pending = session.decision is not None and session.decision.answer is None
    session.record_action("end_supervision", decision_pending=pending)
    session.end_requested = True


def _honk(session) -> None:
    session.record_action("honk")
    session.audio.play("horn")
    session.feedback("Honk.")


def _hazard_lights(session) -> None:
    turn_on = not session.hazard_lights_on
    session.record_action("hazard_lights", on=turn_on)
    session.hazard_lights_on = turn_on
    session.send_vehicle_command("hazard_lights", lambda: session.simulation.set_hazard_lights(turn_on))
    session.feedback("Hazard lights {}.".format("on" if turn_on else "off"))


def _return_to_depot(session) -> None:
    session.return_to_depot = not session.return_to_depot
    session.record_action("return_to_depot", on=session.return_to_depot)
    session.feedback("Return to depot after this job: {}.".format("yes" if session.return_to_depot else "no"))


def _open(dialog_name: str, action_id: str) -> Handler:
    def handler(session) -> str:
        session.record_action(action_id)
        return dialog_name
    return handler


def _talk_start(who: str) -> Handler:
    def handler(session) -> None:
        session.record_action("talk_to_" + who)
        session.start_talking(who)
    return handler


def _talk_stop(session) -> None:
    session.stop_talking()


def _call(partner: str) -> Handler:
    def handler(session) -> str:
        session.call_partner = partner
        session.call_muted = False
        session.record_action("call", partner=partner, latitude=session.vehicle_state.latitude,
                              longitude=session.vehicle_state.longitude)
        return "call"
    return handler


def _hang_up(session) -> None:
    session.record_action("hang_up", partner=session.call_partner)
    session.feedback("Call with {} ended.".format(session.call_partner))
    session.call_partner = ""


def _mute(session) -> str:
    session.call_muted = not session.call_muted
    session.record_action("mute", on=session.call_muted)
    return "call"


def _fleet_request(request: str) -> Handler:
    def handler(session) -> None:
        session.record_action("fleet_request", request=request, latitude=session.vehicle_state.latitude,
                              longitude=session.vehicle_state.longitude)
        session.feedback("Fleet services: {} requested.".format(request.lower()))
    return handler


def _send_location(session) -> None:
    state = session.vehicle_state
    session.record_action("send_location", latitude=state.latitude, longitude=state.longitude)
    session.feedback("Location sent ({:.{d}f}, {:.{d}f}).".format(
        state.latitude, state.longitude, d=settings.GPS_DECIMALS))


def _report_hazard(session) -> None:
    state = session.vehicle_state
    session.record_action("report_hazard", latitude=state.latitude, longitude=state.longitude)
    session.feedback("Hazard reported at the vehicle's location.")


def _reroute_menu(session) -> Optional[str]:
    session.record_action("reroute_menu")
    if not session.scenario.scene.alternate_routes:
        session.feedback("No alternate routes available.")
        return None
    return "reroute"


def _reroute_to(index: int) -> Handler:
    def handler(session) -> None:
        session.record_action("reroute", alternate=index + 1)
        session.perform_outcome("reroute:{}".format(index + 1))
        session.feedback("Rerouting.")
    return handler


def _detailed_status(session) -> str:
    session.record_action("detailed_status")
    return "detailed_status"


def _close(session) -> None:
    return None


# =============================================================================
# The button list (order = drawing order within a group)
# =============================================================================

ACTIONS: List[Action] = [
    # --- command list (Actions panel, left) ---------------------------------
    Action("emergency_services", "Emergency services", "actions", _open("contacts", "emergency_services"), "link"),
    Action("fleet_services", "Fleet services", "actions", _open("fleet", "fleet_services"), "link"),
    Action("talk_to_passenger", "Talk to passenger (hold)", "actions", _talk_start("passenger"), "link",
           kind="hold", release=_talk_stop),
    Action("talk_externally", "Talk externally (hold)", "actions", _talk_start("external"), "link",
           kind="hold", release=_talk_stop),
    Action("honk", "Honk", "actions", _honk, "link"),
    Action("hazard_lights", "Hazard lights", "actions", _hazard_lights, "link", kind="toggle",
           is_on=lambda s: s.hazard_lights_on),
    Action("return_to_depot", "Return to depot after this job", "actions", _return_to_depot, "link",
           kind="toggle", is_on=lambda s: s.return_to_depot),
    # --- driving (Actions panel, right) -------------------------------------
    Action("emergency_stop", "E-STOP", "drive", _emergency_stop, "danger"),
    Action("pull_over", "Pull Over", "drive", _pull_over, "primary"),
    Action("manual_control", "Manual Control", "drive", _manual_control, "primary", kind="toggle",
           is_on=lambda s: s.manual_control_active),
    Action("resume_av", "Resume AV", "drive", _resume_av, "primary"),
    # --- end of scenario ----------------------------------------------------
    Action("end_supervision", "End supervision", "end", _end_supervision, "normal"),
    # --- map tools ----------------------------------------------------------
    Action("reroute_menu", "Reroute", "map", _reroute_menu),
    Action("report_hazard", "Report hazard", "map", _report_hazard),
    Action("send_location", "Send location", "map", _send_location),
    # --- status -------------------------------------------------------------
    Action("detailed_status", "Detailed sensor info", "status", _detailed_status),
]


# =============================================================================
# Dialogs (opened when a handler returns their name)
# =============================================================================

def _reroute_options(session) -> List[DialogOption]:
    routes = session.scenario.scene.alternate_routes
    return [DialogOption(route.label or "Alternate route {}".format(i + 1), _reroute_to(i))
            for i, route in enumerate(routes)]


DIALOGS: Dict[str, Dialog] = {
    "contacts": Dialog("Who to contact?", [
        DialogOption("911", _call("911"), "danger"),
        DialogOption("Police non-emergency", _call("Police non-emergency")),
        DialogOption("Fire department", _call("Fire department")),
        DialogOption("EMS / Ambulance", _call("EMS")),
        DialogOption("Close", _close),
    ]),
    "call": Dialog("Call", [
        DialogOption("Mute / unmute", _mute),
        DialogOption("Hang up", _hang_up, "danger"),
    ], text=lambda s: "On call with {}{}\nVehicle location is shared automatically.".format(
        s.call_partner, " (muted)" if s.call_muted else "")),
    "fleet": Dialog("Fleet services", [
        DialogOption("Request tow truck", _fleet_request("Tow truck")),
        DialogOption("Request battery charge", _fleet_request("Battery charge")),
        DialogOption("Request customer rep", _fleet_request("Customer representative")),
        DialogOption("Request technician", _fleet_request("Technician")),
        DialogOption("Send location", _send_location),
        DialogOption("Close", _close),
    ]),
    # Options filled in at run time from the scene's alternate routes.
    "reroute": Dialog("Choose a route", []),
    # Drawn specially by the interface (full sensor list).
    "detailed_status": Dialog("Detailed status", [DialogOption("Close", _close)]),
}

DYNAMIC_DIALOG_OPTIONS: Dict[str, Callable] = {"reroute": _reroute_options}


def actions_in_group(group: str, hidden: Optional[set] = None) -> List[Action]:
    hidden = hidden or set()
    return [action for action in ACTIONS if action.group == group and action.action_id not in hidden]


def dialog_options(name: str, session) -> List[DialogOption]:
    dialog = DIALOGS[name]
    options = list(dialog.options)
    if name in DYNAMIC_DIALOG_OPTIONS:
        options = DYNAMIC_DIALOG_OPTIONS[name](session) + [DialogOption("Close", _close)]
    return options


def find_action(action_id: str) -> Optional[Action]:
    for action in ACTIONS:
        if action.action_id == action_id:
            return action
    return None


# Labels for the detailed status view: (status key, label, unit).
DETAILED_STATUS_ROWS: List[Tuple[str, str, str]] = [
    ("software_status", "Software", ""),
    ("detection_status", "Detection", ""),
    ("route_planner_status", "Route planner", ""),
    ("control_module_status", "Control module", ""),
    ("camera_front_status", "Camera F", ""),
    ("camera_rear_status", "Camera B", ""),
    ("camera_left_status", "Camera L", ""),
    ("camera_right_status", "Camera R", ""),
    ("lidar_status", "Lidar", ""),
    ("radar_status", "Radar", ""),
    ("gps_status", "GPS", ""),
    ("connection_status", "Connection", ""),
    ("battery_percent", "Battery", "%"),
    ("battery_temp_f", "Battery temp", "°F"),
    ("efficiency_percent", "Efficiency", "%"),
    ("motor_temp_f", "Motor temp", "°F"),
    ("outdoor_temp_f", "Outdoor temp", "°F"),
    ("cabin_temp_f", "Cabin temp", "°F"),
    ("windows", "Windows", ""),
    ("seatbelts", "Seatbelts", ""),
    ("passengers", "Passengers", ""),
]
