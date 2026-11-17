"""
simulation_carla.py -- the Simulation interface implemented with CARLA.

What happens for each scenario
------------------------------
1. Load the scene's map (only if a different map is loaded; it is slow).
2. Apply sun, weather and street lights.
3. Spawn the ego vehicle and plan its route (to a destination, or by
   following the road for a given length).
4. Spawn the scene's actors.  Route-relative positions are converted to map
   coordinates along the planned route.
5. Attach the cameras and a collision sensor.

Who drives the ego vehicle
--------------------------
AUTOSIM calls ``apply_control`` on the ego vehicle EVERY tick, choosing the
control from the current driving mode:

  autonomous      CARLA's BasicAgent follows the route (stops for vehicles
                  and red lights), plus AUTOSIM's own check for props and
                  pedestrians in the lane (BasicAgent ignores those).
  stopping/holding/emergency_stop/pulled_over   brakes.
  pulling_over    BasicAgent with a sideways offset at low speed.
  manual          the operator's (delayed) keyboard input.

CARLA's Traffic Manager autopilot is never used for the ego vehicle, because
it re-sends its own controls every tick and would override braking (the
"car accelerates after E-STOP" problem of earlier prototypes).
"""

import math
import os
import random
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy

from autosim import settings
from autosim.scene import ActorSpec, Placement, RouteSpec, Scene
from autosim.simulation import CameraRequest, DrivingMode, Simulation, VehicleState

try:
    import carla
except ImportError:   # reported with a helpful message in connect()
    carla = None

MPH_TO_KMH = 1.609344
MPH_TO_MPS = 0.44704
ROUTE_STEP_METERS = 2.0                   # spacing of route waypoints
CAMERA_SENSOR_TICK_SECONDS = 1.0 / 30.0   # CARLA renders camera images at most this often
OBSTACLE_CHECK_EVERY_TICKS = 3            # obstacle scan cost control
LANE_CHANGE_SKIP_METERS = 8.0             # where in the new lane the car aims first
OPPOSITE_LANE_RETURN_AFTER_METERS = 35.0  # leave an oncoming lane again after this
SPAWN_HEIGHT = {"vehicle": 0.3, "walker": 1.0, "prop": 0.1}


def _import_agents():
    """Import CARLA's BasicAgent: from CARLA itself if found, else the bundled copy."""
    candidates = []
    if settings.CARLA_AGENTS_PATH:
        candidates.append(settings.CARLA_AGENTS_PATH)
    if os.environ.get("CARLA_ROOT"):
        candidates.append(os.path.join(os.environ["CARLA_ROOT"], "PythonAPI", "carla"))
    candidates.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "third_party", "carla_agents"))
    for folder in candidates:
        if os.path.isdir(os.path.join(folder, "agents")) and folder not in sys.path:
            sys.path.append(folder)
    from agents.navigation.basic_agent import BasicAgent
    from agents.navigation.global_route_planner import GlobalRoutePlanner
    from agents.navigation.local_planner import RoadOption
    return BasicAgent, GlobalRoutePlanner, RoadOption


class _Route:
    """The ego route: waypoints with cumulative distances, for 'route_m' lookups."""

    def __init__(self, waypoints: List[Any], start_distance: float = 0.0) -> None:
        self.waypoints = waypoints
        self.locations = [(w.transform.location.x, w.transform.location.y) for w in waypoints]
        self.cumulative = [start_distance]
        for (x1, y1), (x2, y2) in zip(self.locations, self.locations[1:]):
            self.cumulative.append(self.cumulative[-1] + math.hypot(x2 - x1, y2 - y1))

    @property
    def end_distance(self) -> float:
        return self.cumulative[-1]

    def index_at(self, route_m: float) -> int:
        for index, distance in enumerate(self.cumulative):
            if distance >= route_m:
                return index
        return len(self.cumulative) - 1

    def project(self, x: float, y: float, near_index: Optional[int] = None, window: int = 60) -> Tuple[float, float, int]:
        """(route_m, lateral metres to the right, nearest index) of a map point."""
        indices = range(len(self.locations))
        if near_index is not None:
            indices = range(max(0, near_index - 5), min(len(self.locations), near_index + window))
        best = (float("inf"), 0.0, 0.0, 0)
        for index in indices:
            if index + 1 >= len(self.locations):
                continue
            (x1, y1), (x2, y2) = self.locations[index], self.locations[index + 1]
            dx, dy = x2 - x1, y2 - y1
            length_sq = dx * dx + dy * dy or 1e-9
            t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / length_sq))
            px, py = x1 + dx * t, y1 + dy * t
            distance = math.hypot(x - px, y - py)
            if distance < best[0]:
                length = math.sqrt(length_sq)
                lateral = (-(x - px) * dy + (y - py) * dx) / length    # CARLA: +y is to the right of +x
                best = (distance, self.cumulative[index] + t * length, lateral, index)
        if best[0] == float("inf"):
            return self.cumulative[0], 0.0, 0
        return best[1], best[2], best[3]


class CarlaSimulation(Simulation):
    """See module docstring."""

    def __init__(self, host: str = settings.CARLA_HOST, port: int = settings.CARLA_PORT) -> None:
        self.host, self.port = host, port
        self.client = None
        self.world = None
        self.map = None
        self.route_planner = None
        self.state = VehicleState()
        self.scene: Optional[Scene] = None
        self.ego = None
        self.agent = None
        self.route: Optional[_Route] = None
        self.route_index = 0
        self.mode = DrivingMode.HOLDING
        self.stop_reason = ""
        self.target_speed_mph = settings.AUTONOMOUS_TARGET_SPEED_MPH
        self.manual_input = (0.0, 0.0, settings.MANUAL_COAST_BRAKE, False)
        self.pull_over_end_m = 0.0
        self.route_finishing = False
        self.curb_after_m: Optional[float] = None   # pulling over: steer to the curb only past this (an intersection)
        self.passenger_dropped = False              # the route-end drop-off has happened
        self.passenger_look = ""                    # pedestrian blueprint of the passenger on board ("" = any)
        self.dropoff: Optional[Dict[str, Any]] = None   # the passenger getting out, while it lasts
        self.route_end_action = "dropoff"           # what happens where the route ends: "dropoff" / "pickup"
        self.pickup: Optional[Dict[str, Any]] = None    # the next passenger, waiting or getting in
        self.next_trip_length_m = 0.0               # pickup to drop-off, for the trip being started
        self.lane_return_at_m: Optional[float] = None
        self.lane_offset = 0                        # lanes right of the route's lane the plan follows (- = left)
        self.lane_target = 0                        # lane the car wants; changed one lane at a time, when safe
        self._overtaking: Optional[int] = None      # id of the stopped vehicle / obstacle being passed
        self._keep_lane_until_m: Optional[float] = None  # an operator's lane change: stay in it until here
        self._stuck_behind_since: Optional[float] = None
        self._curb_pending = False                  # pulling over: not yet in the curb lane
        self._hazard_vehicle: Optional[Any] = None   # the vehicle BasicAgent is braking for
        self._next_curb_check = 0.0
        self._resume_when_clear: Optional[bool] = None   # Resume AV asked while a passenger was at the car
        self._blinker: Optional[str] = None          # turn signal now on: "left", "right" or None
        self._lane_changed: Tuple[float, str] = (0.0, "")   # when and to which side the last lane change was
        self.ignored_obstacles: set = set()
        self.spawned: List[Any] = []                # everything we must destroy
        self.scene_actor_ids: set = set()           # props/vehicles placed by the scene or tasks
        self.actor_names: Dict[int, str] = {}       # actor id -> name from the scene file
        self.walker_targets: Dict[int, Tuple[Any, float]] = {}   # walker id -> (target location, speed)
        self.walker_progress: Dict[int, Tuple[Any, float]] = {}  # walker id -> (where it last moved, when)
        self._traffic_manager = None
        self._npc_cars: List[Any] = []              # background cars (Traffic Manager autopilot)
        self._npc_walkers: List[Tuple[Any, Any]] = []   # background pedestrians: (walker, its AI controller)
        self._npc_watch: Dict[int, Dict[str, Any]] = {}  # actor id -> where/when it last moved, nudges tried
        self._wanted_cars = 0
        self._wanted_walkers = 0
        self._next_traffic_check = 0.0
        self.cameras: Dict[str, Any] = {}
        self._frames: Dict[str, Tuple[float, Any]] = {}
        self._frame_lock = threading.Lock()
        self._collisions: List[str] = []
        self._tick_count = 0
        self._yielding = False                      # waiting for a pedestrian to pass
        self._road_cache: Dict[str, List[List[Tuple[float, float]]]] = {}
        self._planner_cache: Dict[str, Any] = {}
        self._stop_signs: List[Tuple[Any, float]] = []     # (stop-sign approach zone, its lane's yaw)
        self._light_locations: List[Any] = []
        self._junctions: List[Dict[str, Any]] = []         # intersections along the current route
        self._junctions_route: Optional[_Route] = None      # the route they were found for
        self._junctions_extended_from: Optional[_Route] = None  # set while a route is being lengthened
        self._junction_asked: Optional[Dict[str, Any]] = None   # blocked intersection the operator was asked about
        self._red_turn: Optional[Dict[str, Any]] = None     # intersection we are turning right on red at
        self._light_stops: List[Tuple[Any, Any, float]] = []    # (traffic light, its stop line, lane yaw)
        self._blocked_since: Optional[float] = None         # held by a vehicle / pedestrian since (AV mode)
        self._BasicAgent = self._GlobalRoutePlanner = self._RoadOption = None

    # =====================================================================
    # Lifecycle
    # =====================================================================

    def connect(self) -> None:
        if carla is None:
            raise RuntimeError("The 'carla' Python package is not installed. Install the client that matches "
                               "your CARLA server, e.g.  pip install carla==0.9.15")
        try:
            self._BasicAgent, self._GlobalRoutePlanner, self._RoadOption = _import_agents()
        except ImportError as error:
            raise RuntimeError("Could not import CARLA's navigation agents ({}). Install networkx and "
                               "shapely:  pip install networkx shapely".format(error))
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(settings.CARLA_TIMEOUT_SECONDS)
        try:
            server_version = self.client.get_server_version()
        except RuntimeError as error:
            raise RuntimeError("Cannot reach the CARLA server at {}:{} ({}). Is CarlaUE4 running?".format(
                self.host, self.port, error))
        client_version = self.client.get_client_version()
        if server_version != client_version:
            print("[carla] Warning: client {} differs from server {}.".format(client_version, server_version))
        self.world = self.client.get_world()
        self._make_asynchronous()
        print("[carla] Connected to CARLA {} at {}:{}.".format(server_version, self.host, self.port))

    def missing_maps(self, map_names: Sequence[str]) -> List[str]:
        available = {_short_map_name(name).lower() for name in self.client.get_available_maps()}
        return sorted({name for name in map_names if name and _short_map_name(name).lower() not in available})

    def load_scenario(self, scene: Scene, time_of_day: str, weather: str, lights: str,
                      cameras: Sequence[CameraRequest], target_speed_mph: float,
                      start_stopped: bool) -> None:
        self.scene = scene
        self._load_map(scene.map_name)
        self._apply_environment(time_of_day, weather, lights)
        self._cache_traffic_controls()
        self.state = VehicleState()
        self._collisions = []
        self.ignored_obstacles = set()
        self.walker_targets = {}
        self.walker_progress = {}
        self._npc_cars, self._npc_walkers, self._npc_watch = [], [], {}
        self.scene_actor_ids = set()
        self.actor_names = {}
        self._yielding = False
        self.lane_return_at_m = None
        self.route_finishing = False
        self.curb_after_m = None
        self.passenger_dropped = False
        self.passenger_look = ""
        self.dropoff = None
        self.route_end_action = "dropoff"
        self.pickup = None
        self.target_speed_mph = target_speed_mph

        self._spawn_ego(scene)
        self.route = _Route(self._plan_route(self.map.get_waypoint(self.ego.get_location()), scene.route))
        self._extend_route_past_junction()
        self.route_index = 0
        self.agent = self._BasicAgent(self.ego, target_speed=target_speed_mph * MPH_TO_KMH,
                                      opt_dict={"sampling_resolution": ROUTE_STEP_METERS},
                                      map_inst=self.map, grp_inst=self._route_planner())
        self._follow_route_lane()
        if time_of_day == "night":
            self.ego.set_light_state(carla.VehicleLightState(
                carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam))

        for spec in scene.actors:
            self._spawn_scene_actor(spec)
        self._spawn_traffic(scene.traffic_vehicles, scene.traffic_walkers)
        self._attach_sensors(cameras)

        start_speed = scene.ego_start_speed_mph
        if start_stopped or start_speed == 0:
            self.mode = DrivingMode.HOLDING
        else:
            self.mode = DrivingMode.AUTONOMOUS
            speed = (start_speed if start_speed is not None else target_speed_mph) * MPH_TO_MPS
            forward = self.ego.get_transform().get_forward_vector()
            self.ego.set_target_velocity(carla.Vector3D(forward.x * speed, forward.y * speed, 0.0))
        self.world.wait_for_tick()
        print("[carla] Scenario ready: route {:.0f} m, {} actors.".format(self.route.end_distance, len(scene.actors)))

    def cleanup_scenario(self) -> None:
        for sensor in list(self.cameras.values()):
            try:
                sensor.stop()
            except RuntimeError:
                pass
        self.cameras = {}
        for actor in self.spawned:
            if actor.type_id.startswith("controller.ai.walker"):
                try:
                    actor.stop()
                except RuntimeError:
                    pass
        if self.client is not None and self.spawned:
            commands = [carla.command.DestroyActor(actor) for actor in reversed(self.spawned)]
            self.client.apply_batch_sync(commands, False)
        self.spawned = []
        self._npc_cars, self._npc_walkers, self._npc_watch = [], [], {}
        self.dropoff = None
        self.pickup = None
        self.ego = None
        self.agent = None
        with self._frame_lock:
            self._frames = {}

    def close(self) -> None:
        if self.client is not None:
            self.cleanup_scenario()

    # =====================================================================
    # Per frame
    # =====================================================================

    def tick(self, dt: float) -> VehicleState:
        if self.ego is None:
            return self.state
        self._tick_count += 1
        self._update_state()
        self._move_walkers()
        self._update_dropoff()
        self._update_pickup()
        self._manage_traffic()
        if self._resume_when_clear is not None and not self._passenger_at_car():
            self.resume_autonomy(self._resume_when_clear)   # Resume AV pressed while the passenger was there
        control = self._control_for_mode()
        self.ego.apply_control(control)
        self._update_turn_signal()
        return self.state

    def camera_frames(self) -> Dict[str, Tuple[float, Any]]:
        with self._frame_lock:
            return dict(self._frames)

    def _control_for_mode(self):
        mode = self.mode
        self.state.waiting_for = ""      # set again below while a traffic rule holds the car
        speed_mph = self.state.speed_mph
        stopped = speed_mph < settings.STOPPED_SPEED_MPH

        if mode == DrivingMode.MANUAL:
            throttle, steer, brake, reverse = self.manual_input
            return carla.VehicleControl(throttle=throttle, steer=steer, brake=brake, reverse=reverse)

        if mode == DrivingMode.EMERGENCY_STOP:
            return carla.VehicleControl(throttle=0.0, brake=settings.EMERGENCY_STOP_BRAKE, hand_brake=True)

        if mode in (DrivingMode.STOPPING, DrivingMode.HOLDING, DrivingMode.PULLED_OVER, DrivingMode.DROPPING_OFF,
                    DrivingMode.PICKING_UP, DrivingMode.ROUTE_COMPLETE):
            if mode == DrivingMode.STOPPING and stopped:
                self.mode = DrivingMode.HOLDING
            brake = settings.SOFT_STOP_BRAKE if mode == DrivingMode.STOPPING else 1.0
            return carla.VehicleControl(throttle=0.0, brake=brake, hand_brake=stopped)

        if mode == DrivingMode.PULLING_OVER:
            if self.curb_after_m is None:
                self._update_lane()
                self._reach_curb_lane()
                if self._curb_pending and self.state.route_traveled_m >= self.pull_over_end_m - 12.0:
                    # Not in the curb lane yet (traffic beside us): pull forward until it is safe.
                    self.pull_over_end_m = self.state.route_traveled_m + 15.0
                    self._extend_route_to(self.pull_over_end_m + 10.0)
                self._check_curb_spot()
            if self.state.route_traveled_m >= self.pull_over_end_m or self.agent.done():
                if stopped:
                    self.set_hazard_lights(True)
                    if self.route_finishing and self.route_end_action == "pickup":
                        self.route_finishing = False
                        self.mode = DrivingMode.PICKING_UP
                        self._start_boarding()
                    elif self.route_finishing:
                        self.route_finishing = False
                        self.mode = DrivingMode.DROPPING_OFF if self._start_dropoff() else DrivingMode.ROUTE_COMPLETE
                    else:
                        self.mode = DrivingMode.PULLED_OVER
                return carla.VehicleControl(throttle=0.0, brake=settings.SOFT_STOP_BRAKE)
            if self.curb_after_m is not None:     # an intersection first: cross it like normal driving
                if self.state.route_traveled_m >= self.curb_after_m:
                    self.curb_after_m = None
                    self._steer_to_curb()
                else:
                    held = self._traffic_rules()
                    if held is not None:
                        return held
            # Moving toward the curb: still stop for an obstacle and wait for pedestrians.
            if self._tick_count % OBSTACLE_CHECK_EVERY_TICKS == 0:
                blocking, self._yielding = self._scan_path()
                if blocking is not None:       # only if we would reach it before our stop point
                    spot = blocking.get_location()
                    box = getattr(blocking, "bounding_box", None)
                    reach_m = self.route.project(spot.x, spot.y, near_index=self.route_index)[0] - \
                        self.ego.bounding_box.extent.x - (box.extent.x if box is not None else 0.5) - 1.0
                    if reach_m > self.pull_over_end_m:
                        blocking = None
                if blocking is not None:
                    self.soft_stop("Obstacle detected ahead: {}".format(
                        self.actor_names.get(blocking.id) or _describe(blocking)))
                    return carla.VehicleControl(throttle=0.0, brake=settings.SOFT_STOP_BRAKE)
            if self._yielding:
                return carla.VehicleControl(throttle=0.0, brake=settings.SOFT_STOP_BRAKE)
            return self.agent.run_step()

        # --- autonomous -------------------------------------------------------
        if self.lane_return_at_m is not None and self.state.route_traveled_m >= self.lane_return_at_m:
            self.lane_return_at_m = None
            self.lane_target = 0     # back to the original lane (when safe)
        self._update_lane()
        if self.lane_offset != 0 and self.lane_target == 0 and self._turn_ahead_needs_route_lane():
            # Still not back in the route's lane before a turn: wait for a gap rather than turn from it.
            front = self.state.route_traveled_m + self.ego.bounding_box.extent.x
            if any(0.0 < junction["entry_m"] - front < 10.0 for junction in self._route_junctions()):
                self.state.waiting_for = "lane change"
                return carla.VehicleControl(throttle=0.0, brake=settings.SOFT_STOP_BRAKE)
        remaining_m = self.route.end_distance - self.state.route_traveled_m
        if (not self.route_finishing and remaining_m <= settings.ROUTE_END_PULL_OVER_START_METERS
                and not any(waypoint.is_junction for waypoint in self.route.waypoints[self.route_index + 1:])):
            self._pull_over_at_route_end()   # not before the last junction: never drift to the curb inside one
            return self.agent.run_step()
        if self.agent.done():        # nothing left to follow: stop where we are
            self.route_finishing = True
            self.pull_over_end_m = self.state.route_traveled_m
            self.mode = DrivingMode.PULLING_OVER
            return carla.VehicleControl(throttle=0.0, brake=settings.SOFT_STOP_BRAKE)
        if self._tick_count % OBSTACLE_CHECK_EVERY_TICKS == 0:
            blocking, walker_in_path = self._scan_path()
            if blocking is not None:
                name = self.actor_names.get(blocking.id) or _describe(blocking)
                self.soft_stop("Obstacle detected ahead: {}".format(name))
                return carla.VehicleControl(throttle=0.0, brake=settings.SOFT_STOP_BRAKE)
            self._yielding = walker_in_path
        if self._yielding:
            self._note_blocked("pedestrian")
            return carla.VehicleControl(throttle=0.0, brake=settings.SOFT_STOP_BRAKE)
        held = self._traffic_rules()
        if held is not None:
            self._blocked_since = None
            return held
        control = self.agent.run_step()
        reason = self._agent_hazard(control)
        queued = reason == "vehicle ahead" and self._in_queue()
        self._note_blocked(reason, queued)
        self._manage_lanes(self._hazard_vehicle if reason == "vehicle ahead" and not queued else None)
        return control

    def _agent_hazard(self, control) -> str:
        """Why CARLA's BasicAgent is braking for a hazard: "red light", "vehicle ahead" or ""."""
        self._hazard_vehicle = None
        if control.throttle > 0.0 or control.brake <= 0.0 or self.state.speed_mph > 3.0:
            return ""
        agent, speed = self.agent, self.state.speed_mps
        try:
            if agent._affected_by_traffic_light(agent._lights_list,
                                                agent._base_tlight_threshold + agent._speed_ratio * speed)[0]:
                return "red light"
            found, vehicle, _ = agent._vehicle_obstacle_detected(self.world.get_actors().filter("*vehicle*"),
                                                                 agent._base_vehicle_threshold + agent._speed_ratio * speed)
            if found:
                self._hazard_vehicle = vehicle
                return "vehicle ahead"
        except (AttributeError, TypeError, RuntimeError):
            pass                       # a different agents version: just no explanation
        return ""

    def _in_queue(self) -> bool:
        """Is the vehicle ahead of us itself waiting (at a red light, or behind another car)?"""
        try:
            found, ahead, _ = self.agent._vehicle_obstacle_detected(
                self.world.get_actors().filter("*vehicle*"), self.agent._base_vehicle_threshold)
        except (AttributeError, TypeError, RuntimeError):
            return False
        if not found or ahead is None:
            return False
        try:
            if ahead.is_at_traffic_light() and ahead.get_traffic_light_state() != carla.TrafficLightState.Green:
                return True
        except (AttributeError, RuntimeError):
            pass
        spot, heading = ahead.get_location(), ahead.get_transform().get_forward_vector()
        for other in self.world.get_actors().filter("*vehicle*"):
            if other.id in (ahead.id, self.ego.id):
                continue
            offset = other.get_location() - spot
            along = offset.x * heading.x + offset.y * heading.y
            if 0.0 < along < 10.0 and abs(offset.x * heading.y - offset.y * heading.x) < 2.0:
                return True                         # something right in front of it too
        return False

    def _note_blocked(self, reason: str, queued: bool = False) -> None:
        """Show why the AV is standing still; if a vehicle or pedestrian keeps it there
        for AV_BLOCKED_ASK_SECONDS, stop and ask the operator (red lights just wait)."""
        if reason:
            self.state.waiting_for = reason
        stopped = self.state.speed_mph < settings.STOPPED_SPEED_MPH
        if reason == "vehicle ahead" and queued:
            self.state.waiting_for = "traffic"     # queued behind a held car: just wait
            reason = "traffic"
        if reason not in ("vehicle ahead", "pedestrian") or not stopped:
            self._blocked_since = None
            return
        now = time.monotonic()
        if self._blocked_since is None:
            self._blocked_since = now
        elif now - self._blocked_since >= settings.AV_BLOCKED_ASK_SECONDS:
            self._blocked_since = None
            print("[carla] Blocked by a {} for {:.0f} s; asking the operator.".format(
                "stopped vehicle" if reason == "vehicle ahead" else "pedestrian", settings.AV_BLOCKED_ASK_SECONDS))
            self.soft_stop("Blocked by a {} ahead for {:.0f} s".format(
                "stopped vehicle" if reason == "vehicle ahead" else "pedestrian", settings.AV_BLOCKED_ASK_SECONDS))

    # --- traffic rules (AV mode) ----------------------------------------------

    def _cache_traffic_controls(self) -> None:
        """Stop-sign approach zones and traffic-light positions on the current map."""
        self._stop_signs = []
        for sign in self.world.get_actors().filter("traffic.stop"):
            try:
                zone = sign.get_transform().transform(sign.trigger_volume.location)
            except (AttributeError, RuntimeError):
                continue
            lane = self.map.get_waypoint(zone)
            if lane is not None:
                self._stop_signs.append((zone, lane.transform.rotation.yaw))
        lights = list(self.world.get_actors().filter("traffic.traffic_light"))
        self._light_locations = [light.get_location() for light in lights]
        self._light_stops = []
        for light in lights:              # which light controls which lane (for other cars' lights)
            try:
                for stop in light.get_stop_waypoints():
                    self._light_stops.append((light, stop.transform.location, stop.transform.rotation.yaw))
            except (AttributeError, RuntimeError):
                pass
        self._junctions_route = None
        self._junction_asked = None
        print("[carla] Traffic rules: {} stop signs, {} traffic lights on this map.".format(
            len(self._stop_signs), len(self._light_locations)))

    def _route_junctions(self) -> List[Dict[str, Any]]:
        """The intersections along the current route (found once per route)."""
        if self._junctions_route is self.route:
            return self._junctions
        route, waypoints = self.route, self.route.waypoints
        junctions, index = [], 0
        while index < len(waypoints):
            if not waypoints[index].is_junction:
                index += 1
                continue
            start = index
            while index < len(waypoints) and waypoints[index].is_junction:
                index += 1
            inside = waypoints[start:index]
            before, after = waypoints[max(0, start - 1)], waypoints[min(index, len(waypoints) - 1)]
            center = carla.Location(sum(w.transform.location.x for w in inside) / len(inside),
                                    sum(w.transform.location.y for w in inside) / len(inside),
                                    inside[0].transform.location.z)
            radius = max(8.0, 0.5 * before.transform.location.distance(after.transform.location))
            junction_id = None
            try:
                junction_id = inside[0].get_junction().id
                box = inside[0].get_junction().bounding_box
                radius = max(box.extent.x, box.extent.y) + 1.0
                if box.location.distance(center) < radius:
                    center = box.location
            except (AttributeError, RuntimeError):
                pass
            entry_m = route.cumulative[start]
            signalized = any(light.distance(center) < radius + 12.0 for light in self._light_locations)
            stop_m = None
            if not signalized:
                for zone, lane_yaw in self._stop_signs:
                    if zone.distance(before.transform.location) > 25.0:
                        continue
                    zone_m, lateral, near = route.project(zone.x, zone.y)
                    heading = (lane_yaw - waypoints[near].transform.rotation.yaw + 180.0) % 360.0 - 180.0
                    if entry_m - 25.0 <= zone_m <= entry_m + 1.0 and abs(lateral) < 4.0 and abs(heading) < 45.0:
                        stop_m = entry_m - settings.STOP_LINE_SETBACK_METERS
                        break
            exit_m = route.cumulative[index] if index < len(waypoints) else route.end_distance
            others_stop = not signalized and stop_m is None and any(
                zone.distance(center) < radius + 25.0 for zone, _ in self._stop_signs)
            junctions.append({"entry_m": entry_m, "exit_m": exit_m, "center": center, "radius": radius,
                              "id": junction_id, "entry_yaw": before.transform.rotation.yaw, "entry_wp": before,
                              "others_stop": others_stop, "light": None, "red_stop_at": None,
                              "turn": _yaw_change(before, after), "signalized": signalized, "stop_m": stop_m,
                              "stopped_at": None, "stop_done": False, "cleared": False, "waiting_since": None,
                              "arrived_at": None, "arrivals": {}})
        extended = self._junctions_extended_from is not None and self._junctions_extended_from is self._junctions_route
        previous = {round(j["entry_m"], 1): j for j in self._junctions} if extended else {}
        self._junctions_extended_from = None
        for junction in junctions:          # same intersection after the route was extended: keep its state
            old = previous.get(round(junction["entry_m"], 1))
            if old is not None:
                for key in ("stopped_at", "stop_done", "cleared", "waiting_since", "arrived_at", "arrivals",
                            "light", "red_stop_at"):
                    junction[key] = old[key]
        self._junctions, self._junctions_route = junctions, route
        return junctions

    def _traffic_rules(self) -> Optional[Any]:
        """Speed limit, stop signs and right of way.  A braking control while a rule holds the car, else None.

        Red lights and vehicles directly ahead are handled by CARLA's BasicAgent."""
        desired_kmh = self.target_speed_mph * MPH_TO_KMH
        if settings.OBEY_SPEED_LIMITS:
            limit = self.ego.get_speed_limit()
            if limit and limit > 0:
                desired_kmh = min(desired_kmh, limit)
        front = self.state.route_traveled_m + self.ego.bounding_box.extent.x
        # The intersection ahead, or the one we are in (watched until we are out of it).
        junction = next((j for j in self._route_junctions()
                         if j["entry_m"] <= front + settings.INTERSECTION_LOOKAHEAD_METERS
                         and front < j["exit_m"] + 1.0), None)
        if self._red_turn is not None and junction is not self._red_turn:
            self._red_turn = None                       # through the right turn on red: obey lights again
            self._agent_ignores_lights(False)
        if junction is None:
            self.agent.set_target_speed(desired_kmh)
            return None
        now = time.monotonic()
        stopped = self.state.speed_mph < settings.STOPPED_SPEED_MPH
        if not junction["signalized"]:
            if junction["stop_m"] is not None or not junction["others_stop"] or abs(junction["turn"]) > 30.0:
                desired_kmh = min(desired_kmh, settings.INTERSECTION_APPROACH_SPEED_MPH * MPH_TO_KMH)
            stop_sign = settings.OBEY_STOP_SIGNS and junction["stop_m"] is not None
            if front > junction["entry_m"]:
                junction["stop_done"] = True            # already in it (e.g. the operator sent it on)
            if settings.YIELD_AT_INTERSECTIONS:
                self._track_arrivals(junction, now)     # from the start, so earlier arrivals keep their turn
            if stop_sign and not junction["stop_done"]:
                to_line = junction["stop_m"] - front
                desired_kmh = min(desired_kmh, max(3.0, math.sqrt(
                    2.0 * settings.STOP_SIGN_DECELERATION_MPS2 * max(0.0, to_line)) * 3.6))
                if to_line <= 1.0:
                    if not stopped:
                        self.state.waiting_for = "stop sign"
                        return carla.VehicleControl(throttle=0.0, brake=settings.SOFT_STOP_BRAKE)
                    if junction["stopped_at"] is None:
                        junction["stopped_at"] = now
                        print("[carla] Stopped at a stop sign.")
                    if now - junction["stopped_at"] < settings.STOP_SIGN_WAIT_SECONDS:
                        self.state.waiting_for = "stop sign"
                        return carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True)
                    junction["stop_done"] = True
            if stop_sign and not junction["stop_done"]:
                self.agent.set_target_speed(desired_kmh)
                return None
        if settings.YIELD_AT_INTERSECTIONS:
            held = self._right_of_way(junction, front, now, stopped)
            if held is not None:
                if isinstance(held, float):
                    desired_kmh = min(desired_kmh, held)       # rolling up to the line
                else:
                    return held
        self.agent.set_target_speed(desired_kmh)
        return None

    def _right_of_way(self, junction: Dict[str, Any], front: float, now: float, stopped: bool):
        """Who may go at this intersection: None = we may (go on); a float = a speed
        limit in km/h (rolling up to the line while giving way); a control = hold / brake.

        Every nearby vehicle's possible paths through the intersection are compared
        with ours; only one that would be where we are going, at about the same
        time, matters (so turning right does not wait for traffic that never comes
        near the right turn).  Whether we give way follows the rules of the road:

        * Anyone already in the intersection, or about to run a stop sign / red light.
        * At a stop sign (and turning right on red): everyone with right of way;
          at an all-way stop, whoever stopped first (same time: the car on the right).
        * Green light: only oncoming traffic when we turn left.
        * No signs at all: traffic from the right; oncoming traffic when we turn left.
        * Never enter without room behind the intersection (do not block it).

        The check goes on inside the intersection: if a car pulls out anyway (as
        CARLA's background cars do, they do not take turns with the AV) and we can
        still stop before its path, we do."""
        to_entry = junction["entry_m"] - front
        speed = self.state.speed_mps
        if to_entry > max(12.0, 3.0 + speed * speed / 6.0):
            return None                 # early enough to stop at the line
        cleared = junction["cleared"]   # the operator checked it and said go: only avoid a collision
        ours = self._our_priority(junction, to_entry, now, stopped)
        if ours == "red":
            return None                 # red light: CARLA's BasicAgent holds the car at the line
        if stopped and to_entry <= 3.5 and junction.get("arrived_at") is None:
            junction["arrived_at"] = junction.get("stopped_at") or junction.get("red_stop_at") or now
        path = self._our_path(junction, front)
        self._track_arrivals(junction, now)
        other, conflict_m = self._must_give_way_to(junction, path, ours, collision_only=cleared)
        reason = "cross traffic"
        if other is None and to_entry > -0.5 and not cleared:      # not in the intersection yet
            if ours in ("stop", "right_on_red"):
                other, conflict_m = self._has_priority_over_us(junction, path), to_entry
            if other is None and to_entry <= 3.5:
                other, reason = self._exit_blocker(junction), "space after intersection"
        if other is None:
            junction["waiting_since"] = None
            if ours == "right_on_red" and to_entry > -0.5 and self._red_turn is not junction:
                self._red_turn = junction                      # all clear after a full stop: turn right on red
                self._agent_ignores_lights(True)
                print("[carla] Turning right on red after a full stop.")
            return None
        if to_entry <= -0.5:
            # Already in the intersection: stop short of the other car's path if we still can;
            # if we cannot, keep going (stopping in its path would be worse).
            if conflict_m - 1.0 < speed * speed / (2.0 * 5.0):
                return None
            self.state.waiting_for = reason
            return carla.VehicleControl(throttle=0.0, brake=0.7 if speed > 2.0 else settings.SOFT_STOP_BRAKE)
        self.state.waiting_for = reason
        if reason == "cross traffic" and stopped and not cleared:
            if junction["waiting_since"] is None:
                junction["waiting_since"] = now
                print("[carla] Giving way to {} at an intersection.".format(_describe(other)))
            elif now - junction["waiting_since"] > settings.YIELD_MAX_WAIT_SECONDS:
                self._junction_asked = junction
                self.soft_stop("Intersection blocked by {}".format(_describe(other)))
        if to_entry > 3.0:                             # not at the line yet: roll up to it, then hold
            return max(3.0, math.sqrt(2.0 * settings.STOP_SIGN_DECELERATION_MPS2 * (to_entry - 2.5)) * 3.6)
        return carla.VehicleControl(throttle=0.0, brake=settings.SOFT_STOP_BRAKE)

    def _our_priority(self, junction: Dict[str, Any], to_entry: float, now: float, stopped: bool) -> str:
        """Our standing at this intersection: "green" (light green or yellow), "red" (wait for
        the light), "right_on_red" (after a full stop at red, turning right: lowest priority),
        "stop" (stop sign: lowest priority), "priority" (through road, the others have stop
        signs) or "uncontrolled" (no signs: yield to the right)."""
        if not junction["signalized"]:
            if junction["stop_m"] is not None and settings.OBEY_STOP_SIGNS:
                return "stop"
            return "priority" if junction["others_stop"] else "uncontrolled"
        if self._red_turn is junction:
            return "right_on_red"
        state = self._our_light()
        if state is None and to_entry > 0.0:             # not at it yet: look the light up for our lane
            state = self._approach_control(junction["entry_wp"], junction)[1]
        if state is not None:
            junction["light"] = state                      # remembered while crossing (no longer "at" the light)
        state = junction.get("light")
        if state != "red":
            junction["red_stop_at"] = None
            return "green" if state is not None else "red"
        if not (settings.RIGHT_ON_RED and junction["turn"] > 30.0 and stopped and to_entry <= 6.0):
            junction["red_stop_at"] = None
            return "red"
        if junction.get("red_stop_at") is None:
            junction["red_stop_at"] = now
        return "right_on_red" if now - junction["red_stop_at"] >= settings.STOP_SIGN_WAIT_SECONDS else "red"

    def _our_light(self) -> Optional[str]:
        """Our traffic light while we are at it: "green" (also yellow), "red", or None."""
        try:
            if not self.ego.is_at_traffic_light():
                return None
            return "red" if self.ego.get_traffic_light_state() == carla.TrafficLightState.Red else "green"
        except (AttributeError, RuntimeError):
            return None

    def _agent_ignores_lights(self, ignore: bool) -> None:
        try:
            self.agent.ignore_traffic_lights(ignore)
        except (AttributeError, RuntimeError):
            pass

    def _our_path(self, junction: Dict[str, Any], front: float) -> List[Tuple[float, float, float]]:
        """Our way through the intersection: points (x, y, metres ahead of our front bumper)."""
        route = self.route
        waypoints = route.waypoints
        lane_width = waypoints[min(self.route_index, len(waypoints) - 1)].lane_width or 3.5
        shift = self.lane_offset * lane_width
        points = []
        for index in range(max(0, route.index_at(min(front, junction["entry_m"] - 2.0)) - 1), len(waypoints)):
            route_m = route.cumulative[index]
            if route_m > junction["exit_m"] + 4.0:
                break
            transform = waypoints[index].transform
            x, y = transform.location.x, transform.location.y
            if shift:
                right = transform.get_right_vector()
                x, y = x + right.x * shift, y + right.y * shift
            points.append((x, y, route_m - front))
        return _densify(points)

    def _vehicle_paths(self, vehicle, junction: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Where this vehicle may go through the intersection.  Each path: "points"
        (x, y, metres from the vehicle's centre), "inside" (already in the intersection),
        "entry_m" (metres to the intersection), "control" ("light", "stop" or None) with
        "light" (its light state, if known), "from" ("left", "right", "oncoming", "same")."""
        location = vehicle.get_location()
        lane = self.map.get_waypoint(location)
        if lane is None:
            return []
        velocity = vehicle.get_velocity()
        speed = math.hypot(velocity.x, velocity.y)
        heading = vehicle.get_transform().get_forward_vector()
        if speed > 0.5:
            heading = carla.Vector3D(velocity.x / speed, velocity.y / speed, 0.0)
        if lane.is_junction:
            try:
                if lane.get_junction().id != junction["id"]:
                    return []
            except (AttributeError, RuntimeError):
                return []
            # In the intersection: follow its current motion (lanes overlap here).
            points, yaw = [(location.x, location.y, 0.0)], math.atan2(heading.y, heading.x)
            try:
                yaw_rate = math.radians(vehicle.get_angular_velocity().z)
            except (AttributeError, RuntimeError):
                yaw_rate = 0.0
            x, y, s, step = location.x, location.y, 0.0, 1.0
            while speed > 0.3 and s < max(8.0, speed * 4.0):
                yaw += yaw_rate * step / speed
                x, y, s = x + math.cos(yaw) * step, y + math.sin(yaw) * step, s + step
                points.append((x, y, s))
            if speed <= 0.3:              # standing: its whole body
                extent = vehicle.bounding_box.extent.x
                points = [(location.x + heading.x * d, location.y + heading.y * d, 0.0) for d in (-extent, 0.0, extent)]
            return [{"points": points, "inside": True, "entry_m": 0.0, "control": None, "light": None,
                     "from": self._approach_side(math.degrees(math.atan2(heading.y, heading.x)), junction)}]
        forward = lane.transform.get_forward_vector()
        if heading.x * forward.x + heading.y * forward.y < 0.5:
            return []                     # not driving along this lane (matched to the lane beside it)
        limit = min(settings.YIELD_WATCH_METERS, 15.0 + speed * (settings.YIELD_GAP_SECONDS + 6.0))
        prefix, s, step = [(location.x, location.y, 0.0)], 0.0, 2.0
        while s < limit:
            ahead = lane.next(step)
            if not ahead:
                return []
            if any(w.is_junction for w in ahead):
                if not any(w.is_junction and _junction_id(w) == junction["id"] for w in ahead):
                    return []             # another intersection first
                break
            lane, s = ahead[0], s + step
            prefix.append((lane.transform.location.x, lane.transform.location.y, s))
        else:
            return []                     # does not reach this intersection (soon)
        control, light = self._approach_control(lane, junction)
        side = self._approach_side(lane.transform.rotation.yaw, junction)
        paths = []
        for first in ahead[:4]:
            points, through, d = list(prefix), first, s
            for _ in range(60):
                d += step
                points.append((through.transform.location.x, through.transform.location.y, d))
                if not through.is_junction:
                    break
                following = through.next(step)
                if not following:
                    break
                through = following[0]
            for _ in range(2):            # a little way into the road beyond
                following = through.next(step)
                if not following:
                    break
                through, d = following[0], d + step
                points.append((through.transform.location.x, through.transform.location.y, d))
            paths.append({"points": _densify(points), "inside": False, "entry_m": s, "control": control,
                          "light": light, "from": side, "turn": _yaw_change(lane, through)})
        # A turn signal tells which way it goes (CARLA's traffic sets them).
        try:
            lights = int(vehicle.get_light_state())
            left = bool(lights & int(carla.VehicleLightState.LeftBlinker))
            right = bool(lights & int(carla.VehicleLightState.RightBlinker))
        except (AttributeError, RuntimeError, TypeError):
            left = right = False
        if left and right:
            return paths                  # hazard lights: could go anywhere
        if right:
            signalled = [p for p in paths if p["turn"] > 30.0]
        elif left:
            signalled = [p for p in paths if p["turn"] < -30.0]
        else:                             # no signal: straight on (it is watched once it is in the intersection)
            signalled = [p for p in paths if abs(p["turn"]) <= 30.0]
        return signalled or paths

    def _approach_side(self, yaw: float, junction: Dict[str, Any]) -> str:
        """Which way a vehicle heading ``yaw`` comes from, as seen from our approach."""
        relative = (yaw - junction["entry_yaw"] + 180.0) % 360.0 - 180.0
        if abs(relative) < 45.0:
            return "same"
        if abs(relative) > 135.0:
            return "oncoming"
        return "right" if relative < 0.0 else "left"      # CARLA: yaw grows clockwise

    def _approach_control(self, entry, junction: Dict[str, Any]) -> Tuple[Optional[str], Optional[str]]:
        """(the traffic control on the lane ``entry`` that leads into the intersection, its light state)."""
        location, yaw = entry.transform.location, entry.transform.rotation.yaw
        for light, stop, stop_yaw in self._light_stops:
            if stop.distance(location) < 15.0 and abs((stop_yaw - yaw + 180.0) % 360.0 - 180.0) < 45.0:
                try:
                    return "light", "red" if light.get_state() == carla.TrafficLightState.Red else "green"
                except (AttributeError, RuntimeError):
                    return "light", None
        if junction["signalized"]:
            return "light", None
        for zone, lane_yaw in self._stop_signs:
            if zone.distance(location) < 25.0 and abs((lane_yaw - yaw + 180.0) % 360.0 - 180.0) < 45.0:
                return "stop", None
        return None, None

    def _must_give_way_to(self, junction: Dict[str, Any], path, ours: str,
                          collision_only: bool = False) -> Tuple[Optional[Any], float]:
        """(a vehicle whose path we would cross too close to it in time and that has
        right of way over us, metres from our front bumper to where we would meet it).
        ``collision_only`` (the operator said go): any moving vehicle we would run into,
        whatever the right of way; standing ones are left to the operator."""
        if not path:
            return None, 0.0
        ego = self.ego.get_transform()
        forward = ego.get_forward_vector()
        center, radius = junction["center"], junction["radius"]
        turning_left = junction["turn"] < -30.0
        our_speed = self.state.speed_mps
        our_top = max(our_speed, settings.INTERSECTION_APPROACH_SPEED_MPH / 2.237)
        our_half = self.ego.bounding_box.extent
        margin = settings.YIELD_GAP_SECONDS
        for vehicle in self.world.get_actors().filter("vehicle.*"):
            if vehicle.id == self.ego.id or vehicle.id in self.scene_actor_ids:
                continue   # placed obstacles are handled by the obstacle check
            location = vehicle.get_location()
            if math.hypot(center.x - location.x, center.y - location.y) > radius + settings.YIELD_WATCH_METERS:
                continue
            if (location.x - ego.location.x) * forward.x + (location.y - ego.location.y) * forward.y < -3.0 and \
                    math.hypot(center.x - location.x, center.y - location.y) > radius:
                continue   # behind us
            velocity = vehicle.get_velocity()
            speed = math.hypot(velocity.x, velocity.y)
            to_center = math.hypot(center.x - location.x, center.y - location.y)
            if to_center > radius + 3.0 and (
                    to_center - radius > 25.0 + speed * (settings.YIELD_GAP_SECONDS + 6.0) or
                    (speed > 0.5 and velocity.x * (center.x - location.x) + velocity.y * (center.y - location.y) < 0)):
                continue   # too far to matter yet / driving away from the intersection
            extent = vehicle.bounding_box.extent
            for other in self._vehicle_paths(vehicle, junction):
                if other["from"] == "same" and not other["inside"]:
                    continue                       # queue ahead of / behind us: BasicAgent keeps its distance
                meet = _first_conflict(path, other["points"], our_half.y + extent.y + 0.5)
                if meet is None:
                    continue
                our_in, our_out, their_in, their_out, merging = meet
                if collision_only and speed <= 0.3:
                    continue
                if other["inside"] and speed <= 0.3:
                    return vehicle, our_in         # standing in the intersection, in our way
                if speed <= 0.3:
                    continue                       # waiting at its line: arrival order decides
                will_stop = False
                if other["control"] == "stop" or (other["control"] == "light" and
                                                  (other["light"] == "red" or
                                                   (other["light"] is None and other["from"] in ("left", "right")))):
                    # Can it still stop at its line?  Then it will (it has the stop sign / red light).
                    # A car just moving off from its line has stopped already: it is going.
                    will_stop = other["entry_m"] > 4.0 and speed * speed / (2.0 * 4.5) + 1.0 < other["entry_m"]
                if will_stop and not other["inside"]:
                    continue
                if not other["inside"] and not collision_only and not self._gives_way_to(ours, other, turning_left):
                    continue                       # we have right of way: it must wait for us
                # Same place, same time?
                t_in = _time_to_cover(our_in - 1.0, our_speed, 1.8, our_top)
                t_out = _time_to_cover(our_out + 2.0 * our_half.x + 1.0, our_speed, 1.8, our_top)
                top = speed if speed >= 3.0 else max(speed, 8.0)    # moving off: it speeds up
                o_in = _time_to_cover(their_in - extent.x - 1.0, speed, 2.0, top)
                o_out = _time_to_cover(their_out + extent.x + 1.0, speed, 2.0, top)
                if merging:                        # into the same lane: keep a gap either way
                    lag = margin + max(0.0, speed - our_top) / 2.0
                    clash = t_in - margin < o_in < t_in + lag
                else:
                    clash = o_in < t_out + margin and o_out + margin > t_in
                if clash:
                    return vehicle, our_in
        return None, 0.0

    @staticmethod
    def _gives_way_to(ours: str, other: Dict[str, Any], turning_left: bool) -> bool:
        """The rules of the road: must we let this approaching (not yet stopping) vehicle go first?"""
        side, control = other["from"], other["control"]
        if ours in ("stop", "right_on_red"):
            return True                            # everyone with right of way (stopped cars: arrival order)
        if control in ("stop", "light") and ours in ("priority", "green", "uncontrolled"):
            # It has a stop sign / red light but will not stop in time (or has just pulled out): let it.
            if ours == "green" and side == "oncoming" and other["light"] != "red":
                return turning_left                # oncoming on green: only when we turn left across it
            return True
        if ours == "green":
            return turning_left and side == "oncoming"
        if ours == "priority":
            return turning_left and side == "oncoming"   # both on the through road
        return side == "right" or (turning_left and side == "oncoming")   # no signs: yield to the right

    def _update_turn_signal(self) -> None:
        """Signal like a driver must: right while pulling over, the side of a lane
        change (before and while changing), and the turn's side before a turning
        intersection.  Hazard lights, when on, take precedence."""
        if self.ego is None or self.state.hazard_lights_on:
            return
        wanted = None
        if self.mode == DrivingMode.PULLING_OVER:
            wanted = "right"
        elif self.mode == DrivingMode.AUTONOMOUS and self.route is not None:
            changed_at, changed_to = self._lane_changed
            if self.lane_target != self.lane_offset:
                wanted = "right" if self.lane_target > self.lane_offset else "left"
            elif time.monotonic() - changed_at < 3.0:
                wanted = changed_to
            else:
                front = self.state.route_traveled_m + self.ego.bounding_box.extent.x
                for junction in self._route_junctions():
                    if junction["entry_m"] - 30.0 <= front <= junction["exit_m"] and abs(junction["turn"]) > 30.0:
                        wanted = "right" if junction["turn"] > 0 else "left"
                        break
        if wanted == self._blinker:
            return
        self._blinker = wanted
        try:
            both = int(carla.VehicleLightState.LeftBlinker) | int(carla.VehicleLightState.RightBlinker)
            lights = int(self.ego.get_light_state()) & ~both
            if wanted is not None:
                lights |= int(carla.VehicleLightState.LeftBlinker if wanted == "left"
                              else carla.VehicleLightState.RightBlinker)
            self.ego.set_light_state(carla.VehicleLightState(lights))
        except (AttributeError, RuntimeError, TypeError):
            pass

    def _waiting_at_line(self, vehicle, junction: Dict[str, Any]) -> bool:
        """Is this vehicle standing in a lane that runs into this intersection, at its line
        (not parked at the side, not in our own lane in front of us)?"""
        lane = self.map.get_waypoint(vehicle.get_location())
        if lane is None or lane.is_junction or junction.get("id") is None:
            return False
        location, center = vehicle.get_location(), lane.transform.location
        right = lane.transform.get_right_vector()
        if abs((location.x - center.x) * right.x + (location.y - center.y) * right.y) > 1.3:
            return False                                   # at the side of the road: parked
        heading = lane.transform.get_forward_vector()
        ours = self.ego.get_transform().get_forward_vector()
        if heading.x * ours.x + heading.y * ours.y > 0.85:
            return False                                   # our own direction: queue ahead / behind us
        return any(ahead.is_junction and ahead.get_junction().id == junction["id"]
                   for distance in (2.0, 4.0, 6.0, 8.0, 10.0) for ahead in lane.next(distance))

    def _track_arrivals(self, junction: Dict[str, Any], now: float) -> None:
        """Remember when each other car came to a stop at its line (for arrival order)."""
        arrivals = junction.setdefault("arrivals", {})
        center, radius = junction["center"], junction["radius"]
        waiting = set()
        for vehicle in self.world.get_actors().filter("vehicle.*"):
            if vehicle.id == self.ego.id or vehicle.id in self.scene_actor_ids:
                continue
            location = vehicle.get_location()
            if math.hypot(center.x - location.x, center.y - location.y) > radius + 15.0:
                continue
            velocity = vehicle.get_velocity()
            if math.hypot(velocity.x, velocity.y) > 0.5 or not self._waiting_at_line(vehicle, junction):
                continue
            waiting.add(vehicle.id)
            arrivals.setdefault(vehicle.id, now)
        for vehicle_id in list(arrivals):
            if vehicle_id not in waiting:
                del arrivals[vehicle_id]                   # it has gone (or moved off: then it is cross traffic)

    def _has_priority_over_us(self, junction: Dict[str, Any], path) -> Optional[Any]:
        """At a stop: a car that stopped at its line before us, or at the same time on our
        right, and that may go where we are going (a car whose every way misses ours,
        e.g. one on our right going straight while we turn right, can go at the same time)."""
        ours = junction.get("arrived_at")
        if ours is None:
            return None                                    # we are not stopped at the line
        ego = self.ego.get_transform()
        right = ego.get_right_vector()
        for vehicle_id, arrived in sorted(junction.get("arrivals", {}).items(), key=lambda item: item[1]):
            vehicle = self.world.get_actor(vehicle_id)
            if vehicle is None:
                continue
            offset = vehicle.get_location() - ego.location
            first = arrived < ours - 0.5
            tie_on_right = abs(arrived - ours) <= 1.0 and offset.x * right.x + offset.y * right.y > 0
            if not (first or tie_on_right):
                continue
            width = self.ego.bounding_box.extent.y + vehicle.bounding_box.extent.y + 0.5
            if any(_first_conflict(path, other["points"], width) is not None
                   for other in self._vehicle_paths(vehicle, junction)) or not path:
                return vehicle
        return None

    def _exit_blocker(self, junction: Dict[str, Any]) -> Optional[Any]:
        """A standing vehicle just behind the intersection in our lane, leaving no room for us."""
        half = self.ego.bounding_box.extent.x
        lane_width = self.route.waypoints[min(self.route_index, len(self.route.waypoints) - 1)].lane_width
        for vehicle in self.world.get_actors().filter("vehicle.*"):
            if vehicle.id == self.ego.id:
                continue
            velocity = vehicle.get_velocity()
            if math.hypot(velocity.x, velocity.y) > 1.0:
                continue
            location = vehicle.get_location()
            route_m, lateral, _ = self.route.project(location.x, location.y, near_index=self.route_index, window=60)
            if junction["exit_m"] - 1.0 <= route_m <= junction["exit_m"] + 2 * half + 3.0 and \
                    abs(lateral - self.lane_offset * lane_width) < 1.6:
                return vehicle
        return None

    # =====================================================================
    # Commands
    # =====================================================================

    def resume_autonomy(self, obstacles_checked: bool = True) -> None:
        if self.ego is None:
            return
        if self.mode == DrivingMode.MANUAL:
            # The operator is driving (the Resume AV button ends manual control first):
            # an automatic resume -- a task, an answer -- must not take the wheel.
            print("[carla] Resume ignored: the operator is driving manually.")
            return
        if self._passenger_at_car():
            self._resume_when_clear = obstacles_checked
            print("[carla] Waiting for the passenger to be clear of the car before driving off.")
            return
        self._resume_when_clear = None
        # Obstacles the AV is waiting at are now "checked by the operator" -- not after a
        # detour, where the operator said the way ahead is closed.
        if obstacles_checked:
            for actor in self._actors_in_path(settings.RESUME_CLEARS_OBSTACLES_WITHIN_METERS, walkers=False):
                self.ignored_obstacles.add(actor.id)
        self.agent.set_offset(0.0)
        self.agent.set_target_speed(self.target_speed_mph * MPH_TO_KMH)
        if self.mode in (DrivingMode.MANUAL, DrivingMode.PULLED_OVER, DrivingMode.ROUTE_COMPLETE,
                         DrivingMode.EMERGENCY_STOP):
            self._replan_from_here()
        # Interrupted while pulling over at the route end: plan that pull-over again from here.
        self.route_finishing = False
        self.curb_after_m = None
        self._curb_pending = False            # a pull-over in progress is cancelled ...
        if self._overtaking is None and self._keep_lane_until_m is None:
            self.lane_target = 0              # ... and the car merges back into the route's lane when safe
        self._blocked_since = None
        if self._junction_asked is not None:    # the operator checked the blocked intersection: cross it
            self._junction_asked["cleared"] = True
            self._junction_asked = None
        self.mode = DrivingMode.AUTONOMOUS
        self.stop_reason = ""

    def soft_stop(self, reason: str = "") -> None:
        self.mode = DrivingMode.STOPPING
        self.stop_reason = reason

    def emergency_stop(self) -> None:
        self.mode = DrivingMode.EMERGENCY_STOP

    def pull_over(self) -> None:
        if self.ego is None:
            return
        if self.mode == DrivingMode.MANUAL:
            print("[carla] Pull over ignored: the operator is driving manually.")
            return
        if self.mode in (DrivingMode.PULLING_OVER, DrivingMode.PULLED_OVER, DrivingMode.DROPPING_OFF,
                         DrivingMode.PICKING_UP, DrivingMode.ROUTE_COMPLETE):
            return                                  # already pulling over / at the curb: nothing to do
        if self.mode != DrivingMode.AUTONOMOUS:
            self._replan_from_here()                # e.g. after an E-STOP: continue from where the car is
        traveled, half_length = self.state.route_traveled_m, self.ego.bounding_box.extent.x
        self.pull_over_end_m = traveled + settings.PULL_OVER_DISTANCE_METERS
        self.curb_after_m = None
        # Never stop in (or drift to the curb inside) an intersection: if one lies in
        # the stretch ahead, or the car is in one, drive through it normally first.
        for junction in self._route_junctions():
            if junction["exit_m"] <= traveled - half_length:
                continue                                  # already behind us
            if junction["entry_m"] > self.pull_over_end_m + half_length:
                break                                     # beyond where we will stop
            self.curb_after_m = junction["exit_m"] + half_length
            self.pull_over_end_m = junction["exit_m"] + settings.PULL_OVER_DISTANCE_METERS
        # Never into something: stop before the first obstacle or standing car in the
        # stretch (our lane, the curb lane and the curb), if there is room for that.
        obstacle = self._pull_over_obstacle(self.pull_over_end_m + 10.0)
        if obstacle is not None and obstacle[0] < self.pull_over_end_m:
            stop_m, actor = obstacle
            self.pull_over_end_m = max(traveled + settings.PULL_OVER_MIN_DISTANCE_METERS, stop_m)
            print("[carla] Pulling over before the {} ahead.".format(self.actor_names.get(actor.id) or _describe(actor)))
        if self.curb_after_m is None:
            self._steer_to_curb()
        else:
            print("[carla] Pulling over after the intersection ahead.")
        if self.agent.done():                    # nothing left to follow: just stop
            self._replan_from_here(settings.PULL_OVER_DISTANCE_METERS + 10)
        self.mode = DrivingMode.PULLING_OVER

    def _pull_over_at_route_end(self) -> None:
        """Pull over so the car stops at the curb where the route ends."""
        self.route_finishing = True
        self.curb_after_m = None          # only called past the last intersection
        self._steer_to_curb()
        self.pull_over_end_m = self.route.end_distance
        self.mode = DrivingMode.PULLING_OVER

    def _extend_route_past_junction(self) -> None:
        """Lengthen the route until its last metres are clear of any junction, so it
        never ends inside one and there is room to pull over after the last one."""
        clear = max(1, int(settings.ROUTE_END_CLEAR_OF_JUNCTION_METERS / ROUTE_STEP_METERS))
        waypoints = list(self.route.waypoints)
        added = 0
        while (added < settings.ROUTE_END_MAX_EXTENSION_METERS / ROUTE_STEP_METERS
               and any(waypoint.is_junction for waypoint in waypoints[-clear:])):
            options = waypoints[-1].next(ROUTE_STEP_METERS)
            if not options:
                break
            waypoints.append(min(options, key=lambda option: abs(_yaw_change(waypoints[-1], option))))
            added += 1
        if added:
            self.route = _Route(waypoints, self.route.cumulative[0])

    def _extend_route_to(self, route_m: float) -> None:
        """Lengthen the route along the road until it reaches ``route_m`` (and re-plan in the current lane)."""
        waypoints, end, added = list(self.route.waypoints), self.route.end_distance, False
        while end < route_m:
            options = waypoints[-1].next(ROUTE_STEP_METERS)
            if not options:
                break
            waypoints.append(min(options, key=lambda option: abs(_yaw_change(waypoints[-1], option))))
            end += ROUTE_STEP_METERS
            added = True
        if added:
            self._junctions_extended_from = self.route      # same route, longer: keep intersection state
            self.route = _Route(waypoints, self.route.cumulative[0])
            self.state.route_length_m = self.route.end_distance
            self._shift_plan(self.lane_offset)

    def _pull_over_obstacle(self, reach: float, lane_only: bool = False) -> Optional[Tuple[float, Any]]:
        """(route_m to stop at, actor) for the first placed obstacle or standing car ahead,
        within ``reach``, between our lane and the curb (``lane_only``: in our lane); None if none."""
        if self.route is None:
            return None
        ego = self.ego.get_location()
        ego_m, ego_lateral, _ = self.route.project(ego.x, ego.y, near_index=self.route_index)
        waypoint = self.route.waypoints[min(self.route_index, len(self.route.waypoints) - 1)]
        _, lanes_to_right = _rightmost_lane(waypoint)
        right_band = 1.6 if lane_only else (lanes_to_right - self.lane_offset + 1) * waypoint.lane_width + 1.0
        half = self.ego.bounding_box.extent.x
        candidates = [a for a in self.spawned if a.id in self.scene_actor_ids and not a.type_id.startswith("walker.")]
        for vehicle in self.world.get_actors().filter("vehicle.*"):
            velocity = vehicle.get_velocity()
            if vehicle.id != self.ego.id and math.hypot(velocity.x, velocity.y) < 1.0 and vehicle not in candidates:
                candidates.append(vehicle)
        best = None
        for actor in candidates:
            location = actor.get_location()
            if location.distance(ego) > reach + 15.0:
                continue
            actor_m, lateral, _ = self.route.project(location.x, location.y, near_index=self.route_index, window=60)
            ahead = actor_m - ego_m
            if not 2.0 < ahead < reach or not -1.6 < lateral - ego_lateral < right_band:
                continue
            box = getattr(actor, "bounding_box", None)
            stop_m = actor_m - half - (box.extent.x if box is not None else 0.5) - settings.CURB_SPOT_MARGIN_METERS - 1.0
            if best is None or stop_m < best[0]:
                best = (stop_m, actor)
        return best

    def _passenger_at_car(self) -> bool:
        """Is a passenger getting out (still beside the car / door open) or getting in?"""
        dropoff = self.dropoff
        if dropoff is not None and (dropoff["door_open"] or
                                    dropoff["walker"].get_location().distance(self.ego.get_location()) < 3.5):
            return True
        return self.pickup is not None and self.pickup.get("boarding", False)

    def _check_curb_spot(self) -> None:
        """Pulling over: if the stop spot at the curb is taken (parked car, obstacle, person),
        pull forward to the next free spot instead of stopping into it."""
        if self._curb_pending or self.curb_after_m is not None:
            return
        now = time.monotonic()
        if now < self._next_curb_check or \
                self.pull_over_end_m - self.state.route_traveled_m > settings.CURB_SPOT_CHECK_AHEAD_METERS:
            return
        self._next_curb_check = now + 0.5
        if self._curb_spot_free(self.pull_over_end_m):
            return
        old = self.pull_over_end_m
        candidate = old
        obstacle = self._pull_over_obstacle(old - self.state.route_traveled_m + settings.CURB_SPOT_SEARCH_METERS,
                                            lane_only=True)    # parked cars at the curb are simply passed
        limit = obstacle[0] if obstacle is not None and obstacle[0] > old else float("inf")
        found = False
        for _ in range(int(settings.CURB_SPOT_SEARCH_METERS / ROUTE_STEP_METERS)):
            candidate += ROUTE_STEP_METERS
            if candidate > limit:
                break                                # never past something standing in the way
            self._extend_route_to(candidate + 10.0)
            if self._curb_spot_free(candidate):
                found = True
                break
        if found:
            print("[carla] Curb spot taken; pulling forward {:.0f} m to a free one.".format(candidate - old))
            self.pull_over_end_m = candidate
        elif obstacle is not None and candidate > limit:
            print("[carla] No free curb spot before the obstacle ahead; stopping and asking the operator.")
            self.soft_stop("No free spot to pull over ahead")
        else:
            self.pull_over_end_m = candidate          # look again from there

    def _curb_spot_free(self, route_m: float) -> bool:
        """Is there room to stop at the curb here: no car, obstacle or person in the way,
        and not in (or right next to) an intersection?"""
        half = self.ego.bounding_box.extent.x
        for junction in self._route_junctions():
            if junction["entry_m"] - 3.0 < route_m + half and junction["exit_m"] + 3.0 > route_m - half:
                return False
        index = self.route.index_at(route_m)
        waypoint = self.route.waypoints[index]
        lane = _lanes_over(waypoint, self.lane_offset, driving_only=True) or waypoint
        forward, right = lane.transform.get_forward_vector(), lane.transform.get_right_vector()
        shift = route_m - self.route.cumulative[index]            # exactly at route_m, not the nearest point
        center = lane.transform.location + carla.Location(forward.x * shift, forward.y * shift, 0.0)
        edge = lane.lane_width / 2.0 + 0.5
        beside = lane.get_right_lane()
        if beside is not None and beside.lane_type in (carla.LaneType.Shoulder, carla.LaneType.Parking):
            edge += beside.lane_width
        passengers = self._passenger_ids()
        actors = [a for a in self.world.get_actors().filter("vehicle.*") if a.id != self.ego.id]
        actors += [a for a in self.world.get_actors().filter("walker.pedestrian.*") if a.id not in passengers]
        actors += [a for a in self.spawned if a.id in self.scene_actor_ids and a not in actors]
        for actor in actors:
            if actor.type_id.startswith("vehicle."):
                velocity = actor.get_velocity()
                if math.hypot(velocity.x, velocity.y) > 1.0:
                    continue                     # moving traffic passes; only standing things take the spot
            offset = actor.get_location() - center
            along = offset.x * forward.x + offset.y * forward.y
            side = offset.x * right.x + offset.y * right.y
            box = getattr(actor, "bounding_box", None)
            reach = half + (box.extent.x if box is not None else 0.5) + settings.CURB_SPOT_MARGIN_METERS
            if -reach < along < reach and -0.5 < side < edge:
                return False
        return True

    def _steer_to_curb(self) -> None:
        """Pull over: move to the rightmost lane (lane change made when safe), then
        follow the right edge of that lane slowly."""
        route_waypoint = self.route.waypoints[min(self.route_index, len(self.route.waypoints) - 1)]
        _, lanes_to_right = _rightmost_lane(route_waypoint)
        self.lane_target = lanes_to_right
        self._overtaking = None
        self._curb_pending = True
        self.agent.set_offset(0.0)
        self.agent.set_target_speed(min(self.target_speed_mph, settings.INTERSECTION_APPROACH_SPEED_MPH) * MPH_TO_KMH)
        self._update_lane()
        self._reach_curb_lane()

    def _reach_curb_lane(self) -> None:
        """Once in the rightmost lane: drive along its right edge (or the shoulder / parking lane), slowly."""
        if not self._curb_pending or self.lane_offset != self.lane_target:
            return
        self._curb_pending = False
        route_waypoint = self.route.waypoints[min(self.route_index, len(self.route.waypoints) - 1)]
        rightmost = _lanes_over(route_waypoint, self.lane_offset, driving_only=True) or route_waypoint
        offset = rightmost.lane_width / 2.0 - 1.0 - settings.PULL_OVER_EDGE_MARGIN_METERS
        right = rightmost.get_right_lane()
        if right is not None and right.lane_type in (carla.LaneType.Shoulder, carla.LaneType.Parking):
            offset += right.lane_width * 0.6
        self.agent.set_offset(max(0.0, offset))
        self.agent.set_target_speed(min(self.target_speed_mph, settings.PULL_OVER_SPEED_MPH) * MPH_TO_KMH)

    def change_lane(self, direction: str) -> bool:
        if self.ego is None or self.mode == DrivingMode.MANUAL:
            return False
        here = self.map.get_waypoint(self.ego.get_location())
        target = here.get_left_lane() if direction == "left" else here.get_right_lane()
        if target is None or target.lane_type != carla.LaneType.Driving:
            return False
        opposite = (target.lane_id > 0) != (here.lane_id > 0)
        self.lane_target = self.lane_offset + (-1 if direction == "left" else 1)   # made when it is safe
        # Stay in the new lane until past what the operator is going around (or for a while).
        obstacle = self._pull_over_obstacle(60.0, lane_only=True)
        if obstacle is not None:
            self._overtaking = obstacle[1].id
        else:
            self._keep_lane_until_m = self.state.route_traveled_m + OPPOSITE_LANE_RETURN_AFTER_METERS
            if opposite:   # do not stay on the wrong side of the road
                self.lane_return_at_m = self.state.route_traveled_m + OPPOSITE_LANE_RETURN_AFTER_METERS
        self._update_lane()
        return True

    def reroute(self, alternate_index: int) -> bool:
        if self.ego is None or self.scene is None:
            return False
        routes = self.scene.alternate_routes
        spec = routes[alternate_index] if alternate_index < len(routes) else RouteSpec({})
        if alternate_index >= max(1, len(routes)):
            return False
        # A detour must not lead through what the car stopped for: try the requested
        # turn, then the other one, then straight on; refuse if all are blocked.
        preferred = "left" if alternate_index % 2 == 0 else "right"
        start = self.map.get_waypoint(self.ego.get_location())
        blocker = None
        for turn in (preferred, "right" if preferred == "left" else "left", ""):
            waypoints = self._plan_route(start, spec, turn_at_first_junction=turn)
            blocker = self._obstacle_along(waypoints, settings.DETOUR_CHECK_METERS)
            if blocker is None:
                break
        if blocker is not None:
            name = self.actor_names.get(blocker.id) or _describe(blocker)
            print("[carla] No detour around the {} from here.".format(name))
            self.soft_stop("No detour around the {} from here: turn around with Manual Control".format(name))
            return False
        if turn != preferred:
            print("[carla] Detour: the {} turn is blocked; going {} instead.".format(preferred, turn or "straight on"))
        self.route = _Route(waypoints, self.state.route_traveled_m)
        self._extend_route_past_junction()
        self.route_index = 0
        self.lane_return_at_m = None
        self._follow_route_lane()
        return True

    def _obstacle_along(self, waypoints: List[Any], reach: float) -> Optional[Any]:
        """The first placed prop or vehicle within ``reach`` metres along these waypoints, or None."""
        path = _Route(waypoints)
        points = path.locations
        for actor in self.spawned:
            if actor.id not in self.scene_actor_ids or actor.type_id.startswith("walker."):
                continue
            location = actor.get_location()
            for index in range(len(points) - 1):       # true distance to each piece of the path
                if path.cumulative[index] > reach:
                    break
                (x1, y1), (x2, y2) = points[index], points[index + 1]
                dx, dy = x2 - x1, y2 - y1
                t = max(0.0, min(1.0, ((location.x - x1) * dx + (location.y - y1) * dy) / (dx * dx + dy * dy or 1e-9)))
                if math.hypot(location.x - x1 - dx * t, location.y - y1 - dy * t) < \
                        settings.OBSTACLE_LANE_HALF_WIDTH_METERS:
                    return actor
        return None

    def start_next_trip(self, pickup_after_m: float, trip_length_m: float) -> bool:
        if self.ego is None or self.mode != DrivingMode.ROUTE_COMPLETE:
            return False
        self.next_trip_length_m = trip_length_m
        self.route_end_action = "pickup"
        self.state.trip_leg = "pickup"
        self._follow_new_route(pickup_after_m)
        self._spawn_waiting_passenger()
        print("[carla] Driving on to pick up the next passenger ({:.0f} m).".format(
            self.route.end_distance - self.route.cumulative[0]))
        return True

    def _follow_new_route(self, length_m: float) -> None:
        """Leave the curb in AV mode and follow the road for ``length_m`` from here."""
        start = self.map.get_waypoint(self.ego.get_location())
        self.route = _Route(self._plan_route(start, RouteSpec({"length_m": length_m})), self.state.route_traveled_m)
        self._extend_route_past_junction()
        self.state.route_length_m = self.route.end_distance   # the session reads it this frame
        self.route_index = 0
        self.lane_return_at_m = None
        self.route_finishing = False
        self.curb_after_m = None
        self.agent.set_offset(0.0)
        self.agent.set_target_speed(self.target_speed_mph * MPH_TO_KMH)
        self._follow_route_lane()
        self.set_hazard_lights(False)
        self.stop_reason = ""
        self.mode = DrivingMode.AUTONOMOUS

    def set_manual_control(self, enabled: bool) -> None:
        if enabled:
            self.mode = DrivingMode.MANUAL
            self.manual_input = (0.0, 0.0, settings.MANUAL_COAST_BRAKE, False)
            self._curb_pending = False
            self._resume_when_clear = None
        elif self.mode == DrivingMode.MANUAL:
            self.mode = DrivingMode.HOLDING
            self._replan_from_here()

    def apply_manual_input(self, throttle: float, steer: float, brake: float, reverse: bool) -> None:
        self.manual_input = (throttle, steer, brake, reverse)

    def set_hazard_lights(self, on: bool) -> None:
        self.state.hazard_lights_on = on
        self._blinker = None                  # hazards own the blinkers; turn signals re-applied after
        if self.ego is None:
            return
        state = self.ego.get_light_state()
        blinkers = carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.RightBlinker
        value = int(state) | int(blinkers) if on else int(state) & ~int(blinkers)
        self.ego.set_light_state(carla.VehicleLightState(value))

    def set_target_speed(self, mph: float) -> None:
        self.target_speed_mph = mph
        if self.agent is not None and self.mode == DrivingMode.AUTONOMOUS:
            self.agent.set_target_speed(mph * MPH_TO_KMH)

    # =====================================================================
    # World access for tasks
    # =====================================================================

    def spawn_actor(self, blueprint: str, at: Placement, name: str = "") -> Optional[Any]:
        spec = ActorSpec({"blueprint": blueprint, "at": at.to_dict(), "name": name or blueprint}, 0)
        return self._spawn_scene_actor(spec)

    def spawn_walker(self, at: Placement, walk_to: Placement, speed_mps: float = 1.4,
                     blueprint: str = "") -> Optional[Any]:
        spec = ActorSpec({"blueprint": blueprint or "walker.pedestrian.0001", "at": at.to_dict(),
                          "walk_to": walk_to.to_dict(), "walk_speed": speed_mps}, 0)
        return self._spawn_scene_actor(spec)

    def destroy_actor(self, handle: Any) -> None:
        if handle is None:
            return
        self.walker_targets.pop(handle.id, None)
        self.scene_actor_ids.discard(handle.id)
        if handle in self.spawned:
            self.spawned.remove(handle)
        try:
            handle.destroy()
        except RuntimeError:
            pass

    def distance_to_route_point(self, route_m: float) -> float:
        return route_m - self.state.route_traveled_m

    def project_to_route(self, x: float, y: float) -> float:
        return self.route.project(x, y)[0] if self.route else 0.0

    def road_polylines(self) -> List[List[Tuple[float, float]]]:
        name = self.map.name
        if name not in self._road_cache:
            polylines = []
            for start, end in self.map.get_topology():
                points = [(start.transform.location.x, start.transform.location.y)]
                waypoint = start
                for _ in range(500):
                    following = waypoint.next(ROUTE_STEP_METERS * 2)
                    if not following:
                        break
                    waypoint = following[0]
                    location = waypoint.transform.location
                    points.append((location.x, location.y))
                    if location.distance(end.transform.location) < ROUTE_STEP_METERS * 2:
                        break
                points.append((end.transform.location.x, end.transform.location.y))
                polylines.append(points)
            self._road_cache[name] = polylines
        return self._road_cache[name]

    def route_polyline(self) -> List[Tuple[float, float]]:
        if self.route is None:
            return []
        return self.route.locations[self.route_index:]

    def hazard_points(self) -> List[Tuple[float, float]]:
        points = []
        for actor in self.spawned:
            if actor.id in self.scene_actor_ids and not actor.type_id.startswith("walker."):
                location = actor.get_location()
                points.append((location.x, location.y))
        return points

    # =====================================================================
    # Internals: world setup
    # =====================================================================

    def _make_asynchronous(self) -> None:
        """A previous crash may have left the server in synchronous mode (frozen)."""
        world_settings = self.world.get_settings()
        if world_settings.synchronous_mode:
            world_settings.synchronous_mode = False
            world_settings.fixed_delta_seconds = None
            self.world.apply_settings(world_settings)

    def _load_map(self, map_name: Optional[str]) -> None:
        current = _short_map_name(self.world.get_map().name)
        if map_name and _short_map_name(map_name).lower() != current.lower():
            print("[carla] Loading map {} (this can take a while)...".format(map_name))
            self.world = self.client.load_world(_short_map_name(map_name))
            self._make_asynchronous()
        self.map = self.world.get_map()

    def _route_planner(self):
        name = self.map.name
        if name not in self._planner_cache:
            self._planner_cache[name] = self._GlobalRoutePlanner(self.map, ROUTE_STEP_METERS)
        return self._planner_cache[name]

    def _apply_environment(self, time_of_day: str, weather: str, lights: str) -> None:
        parameters = self.world.get_weather()
        for name, value in settings.WEATHER_PRESETS.get(weather, {}).items():
            setattr(parameters, name, value)
        altitude, azimuth = settings.SUN_PRESETS.get(time_of_day, settings.SUN_PRESETS["day"])
        parameters.sun_altitude_angle, parameters.sun_azimuth_angle = altitude, azimuth
        self.world.set_weather(parameters)
        if lights == "auto":
            on, intensity = (time_of_day == "night"), None
        else:
            on, intensity = settings.LIGHT_PRESETS.get(lights, (False, None))
        manager = self.world.get_lightmanager()
        street_lights = manager.get_all_lights(carla.LightGroup.Street)
        if on:
            manager.turn_on(street_lights)
            if intensity is not None:
                manager.set_intensity(street_lights, float(intensity))
        else:
            manager.turn_off(street_lights)

    def _spawn_ego(self, scene: Scene) -> None:
        library = self.world.get_blueprint_library()
        blueprint = _find_blueprint(library, scene.ego_blueprint) or library.find(settings.DEFAULT_EGO_BLUEPRINT)
        blueprint.set_attribute("role_name", settings.EGO_ROLE_NAME)
        spawn_points = self.map.get_spawn_points()
        if scene.ego_start is not None and not scene.ego_start.is_route_relative:
            start = scene.ego_start
            transform = carla.Transform(carla.Location(start.x, start.y, start.z_m + 0.5), carla.Rotation(yaw=start.yaw))
        else:
            index = scene.ego_spawn_point if scene.ego_spawn_point is not None else 0
            if not 0 <= index < len(spawn_points):
                print("[carla] Spawn point {} does not exist on this map (0-{}); using 0.".format(
                    index, len(spawn_points) - 1))
                index = 0
            transform = spawn_points[index]
        ego = self.world.try_spawn_actor(blueprint, transform)
        if ego is None:   # something is in the way: try slightly higher, then other spawn points
            for candidate in [carla.Transform(transform.location + carla.Location(z=1.0), transform.rotation)] + spawn_points:
                ego = self.world.try_spawn_actor(blueprint, candidate)
                if ego is not None:
                    print("[carla] Warning: start position occupied; ego placed elsewhere.")
                    break
        if ego is None:
            raise RuntimeError("Could not spawn the ego vehicle.")
        self.ego = ego
        self.spawned.append(ego)
        self.world.wait_for_tick()

    def _plan_route(self, start, spec: RouteSpec, turn_at_first_junction: str = "") -> List[Any]:
        """Waypoints from ``start``: to the spec's destination, or following the road."""
        destination = None
        if spec.destination is not None and not spec.destination.is_route_relative:
            destination = carla.Location(spec.destination.x, spec.destination.y, spec.destination.z_m)
        elif spec.destination_spawn_point is not None:
            spawn_points = self.map.get_spawn_points()
            if 0 <= spec.destination_spawn_point < len(spawn_points):
                destination = spawn_points[spec.destination_spawn_point].location
        if destination is not None:
            end = self.map.get_waypoint(destination)
            plan = self._route_planner().trace_route(start.transform.location, end.transform.location)
            if plan:
                return [waypoint for waypoint, _ in plan]
            print("[carla] No route to the destination; following the road instead.")
        # Follow the road, going straight at junctions (or turning once, for detours).
        waypoints, travelled, waypoint = [start], 0.0, start
        turn = turn_at_first_junction
        while travelled < spec.length_m:
            options = waypoint.next(ROUTE_STEP_METERS)
            if not options:
                break
            if len(options) > 1 and turn:
                change = [_yaw_change(waypoint, option) for option in options]
                chosen = options[change.index(min(change) if turn == "left" else max(change))]
                if abs(_yaw_change(waypoint, chosen)) > 20:
                    turn = ""
            else:
                chosen = min(options, key=lambda option: abs(_yaw_change(waypoint, option)))
            travelled += ROUTE_STEP_METERS
            waypoints.append(chosen)
            waypoint = chosen
        return waypoints

    def _set_agent_plan(self, waypoints: List[Any]) -> None:
        plan = [(waypoint, self._RoadOption.LANEFOLLOW) for waypoint in waypoints]
        self.agent.set_global_plan(plan, stop_waypoint_creation=True, clean_queue=True)

    def _remaining_route_waypoints(self) -> List[Any]:
        return self.route.waypoints[self.route_index:] if self.route else []

    def _follow_route_lane(self) -> None:
        """Plan along the route in the route's own lane (a new or changed route)."""
        self.lane_offset = self.lane_target = 0
        self._overtaking = None
        self._keep_lane_until_m = None
        self._set_agent_plan(self.route.waypoints[self.route_index:] or self.route.waypoints)

    def _shift_plan(self, lanes: int) -> None:
        """Follow the remaining route ``lanes`` lanes to the right (negative = left), 0 = original lane.

        Through an intersection the plan stays in the lane the car is in (no lane
        change inside it): from the last lane point before it, it follows that lane's
        own continuation, the one heading the way the route does."""
        remaining = self._remaining_route_waypoints()
        changing = lanes != self.lane_offset
        skip = int(LANE_CHANGE_SKIP_METERS / ROUTE_STEP_METERS) if changing else 1
        self.lane_offset = lanes
        shifted, traced = [], None
        for waypoint in remaining[skip:]:
            if lanes == 0:
                shifted.append(waypoint)
                continue
            if waypoint.is_junction and traced is not None:
                options = traced.next(ROUTE_STEP_METERS)
                if options:
                    traced = min(options, key=lambda option: abs(
                        (option.transform.rotation.yaw - waypoint.transform.rotation.yaw + 180.0) % 360.0 - 180.0))
                shifted.append(traced)
                continue
            target = _lanes_over(waypoint, lanes, driving_only=True)
            traced = target if target is not None else waypoint
            shifted.append(traced)
        if shifted:
            self._set_agent_plan(shifted)

    # --- lane changes (all of them: pull over, operator, passing, returning) ----

    def _update_lane(self) -> None:
        """Move one lane toward ``lane_target`` when that is safe; otherwise keep the lane."""
        if self.lane_target == self.lane_offset or self.route is None:
            return
        step = self.lane_offset + (1 if self.lane_target > self.lane_offset else -1)
        blocker = self._lane_change_blocker(step)
        if blocker is None:
            side = "right" if step > self.lane_offset else "left"
            print("[carla] Lane change {}.".format(side))
            self._lane_changed = (time.monotonic(), side)
            self._shift_plan(step)

    def _lane_change_blocker(self, lanes: int) -> Optional[Any]:
        """Why a change to ``lanes`` is not safe now (an object or a reason string), or None."""
        here = self.map.get_waypoint(self.ego.get_location())
        if here is None or here.is_junction:
            return "in an intersection"
        front = self.state.route_traveled_m + self.ego.bounding_box.extent.x
        returning_for_turn = lanes == 0 and self._turn_ahead_needs_route_lane()
        clearance = 8.0 if returning_for_turn else settings.LANE_CHANGE_CLEAR_OF_INTERSECTION_METERS
        for junction in self._route_junctions():
            if junction["entry_m"] - clearance < front + LANE_CHANGE_SKIP_METERS and \
                    junction["exit_m"] + 2.0 > front - 2 * self.ego.bounding_box.extent.x:
                return "near an intersection"
        index = min(self.route_index, len(self.route.waypoints) - 1)
        ahead = min(index + int((LANE_CHANGE_SKIP_METERS + 10.0) / ROUTE_STEP_METERS), len(self.route.waypoints) - 1)
        for waypoint in (self.route.waypoints[index], self.route.waypoints[ahead]):
            if lanes != 0 and _lanes_over(waypoint, lanes, driving_only=True) is None:
                return "no such lane"
        return self._vehicle_in_lane(lanes)

    def _vehicle_in_lane(self, lanes: int) -> Optional[Any]:
        """A vehicle (or placed obstacle) in the gap the car would change into, or None."""
        route, traveled = self.route, self.state.route_traveled_m
        lane_width = self.route.waypoints[min(self.route_index, len(route.waypoints) - 1)].lane_width
        target_lateral = lanes * lane_width
        my_speed, ego = self.state.speed_mps, self.ego.get_location()
        candidates = [a for a in self.world.get_actors().filter("vehicle.*") if a.id != self.ego.id]
        candidates += [a for a in self.spawned if a.id in self.scene_actor_ids and not a.type_id.startswith("walker.")
                       and a not in candidates]
        for actor in candidates:
            location = actor.get_location()
            if location.distance(ego) > 70.0:
                continue
            route_m, lateral, _ = route.project(location.x, location.y, near_index=max(0, self.route_index - 15),
                                                window=90)
            if abs(lateral - target_lateral) > 0.6 * lane_width:
                continue
            velocity = actor.get_velocity()
            closing = math.hypot(velocity.x, velocity.y) - my_speed      # how fast it catches up from behind
            behind = max(settings.LANE_CHANGE_GAP_BEHIND_METERS, closing * settings.LANE_CHANGE_TIME_GAP_SECONDS + 6.0)
            ahead = settings.LANE_CHANGE_GAP_AHEAD_METERS + my_speed
            if -behind < route_m - traveled < ahead:
                return actor
        return None

    def _turn_ahead_needs_route_lane(self) -> bool:
        """Does the route turn at an intersection within 60 m (so the car must be in the route's lane)?"""
        front = self.state.route_traveled_m + self.ego.bounding_box.extent.x
        return any(0.0 < junction["entry_m"] - front < 60.0 and abs(junction["turn"]) > 30.0
                   for junction in self._route_junctions())

    def _manage_lanes(self, hazard_vehicle: Optional[Any]) -> None:
        """AV mode: pass a stopped car in the next lane, then go back to the route's lane."""
        now = time.monotonic()
        stopped = self.state.speed_mph < settings.STOPPED_SPEED_MPH
        if self._overtaking is not None or (self.lane_target != 0 and not self._curb_pending):
            passed = True
            if self._overtaking is not None:
                actor = self.world.get_actor(self._overtaking)
                if actor is not None:
                    location = actor.get_location()
                    actor_m = self.route.project(location.x, location.y, near_index=max(0, self.route_index - 15),
                                                 window=90)[0]
                    passed = actor_m < self.state.route_traveled_m - 2 * self.ego.bounding_box.extent.x - 6.0
            if self._keep_lane_until_m is not None and self.state.route_traveled_m < self._keep_lane_until_m:
                passed = False                        # the operator's lane change: not yet
            if passed or self._turn_ahead_needs_route_lane():
                self._overtaking = None
                self._keep_lane_until_m = None
                self.lane_target = 0                  # keep right except to pass
        if hazard_vehicle is None or not stopped or not settings.OVERTAKE_STOPPED_VEHICLES:
            self._stuck_behind_since = None
            return
        if self._stuck_behind_since is None:
            self._stuck_behind_since = now
            return
        if now - self._stuck_behind_since < settings.OVERTAKE_AFTER_SECONDS or self.lane_target != self.lane_offset:
            return
        if self._turn_ahead_needs_route_lane():
            return
        waypoint = self.route.waypoints[min(self.route_index, len(self.route.waypoints) - 1)]
        for side in (-1, 1):                          # left lane first, else a free lane on the right
            lane = _lanes_over(waypoint, self.lane_offset + side, driving_only=True)
            if lane is not None and (lane.lane_id > 0) == (waypoint.lane_id > 0):   # same direction only
                if self._lane_change_blocker(self.lane_offset + side) is None:
                    print("[carla] Passing a stopped {} on the {}.".format(_describe(hazard_vehicle),
                                                                          "left" if side < 0 else "right"))
                    self._overtaking = hazard_vehicle.id
                    self.lane_target = self.lane_offset + side
                    self._stuck_behind_since = None
                    self._update_lane()
                    return

    def _replan_from_here(self, minimum_length: float = 0.0) -> None:
        """New plan from the car's current position along the rest of the route."""
        if self.route is None:
            return
        _, lateral, index = self.route.project(self.ego.get_location().x, self.ego.get_location().y)
        remaining = self.route.waypoints[index + 1:]
        if len(remaining) * ROUTE_STEP_METERS < minimum_length:
            start = self.map.get_waypoint(self.ego.get_location())
            remaining = self._plan_route(start, RouteSpec({"length_m": minimum_length}))[1:]
            self.lane_offset = self.lane_target = 0
            if remaining:
                self._set_agent_plan(remaining)
            return
        # Continue in the lane the car is in now (e.g. after manual driving); lane
        # changes back to the route's lane are then made when safe.
        waypoint = self.route.waypoints[index]
        lanes = int(round(lateral / max(1.0, waypoint.lane_width)))
        while lanes != 0 and _lanes_over(waypoint, lanes, driving_only=True) is None:
            lanes -= 1 if lanes > 0 else -1
        self.route_index = index
        self.lane_offset = lanes
        self.lane_target = 0
        self._overtaking = None
        if lanes == 0:
            if remaining:
                self._set_agent_plan(remaining)
        else:
            self._shift_plan(lanes)

    # --- actors -------------------------------------------------------------

    def _placement_transform(self, at: Placement, kind: str):
        """Map transform for a placement (absolute or along the route)."""
        height = SPAWN_HEIGHT.get(kind, 0.1)
        if not at.is_route_relative:
            return carla.Transform(carla.Location(at.x, at.y, at.z_m + height),
                                   carla.Rotation(pitch=at.pitch, yaw=at.yaw, roll=at.roll))
        route = self.route
        index = route.index_at(at.route_m)
        base = route.waypoints[index]
        # Extrapolate beyond the end of the route if needed.
        overshoot = at.route_m - route.cumulative[index]
        if overshoot > 0.5:
            following = base.next(overshoot)
            base = following[0] if following else base
        waypoint = _lanes_over(base, at.lane) if at.lane != 0 else base
        if waypoint is None:
            waypoint = base
        location = waypoint.transform.location
        if waypoint is base and at.lane != 0:   # no such lane: shift by lane widths instead
            right = base.transform.get_right_vector()
            location = location + carla.Location(right.x * at.lane * base.lane_width, right.y * at.lane * base.lane_width)
        right = base.transform.get_right_vector()
        location = carla.Location(location.x + right.x * at.lateral_m, location.y + right.y * at.lateral_m,
                                  location.z + at.z_m + height)
        yaw = base.transform.rotation.yaw + at.yaw   # relative to the route direction
        return carla.Transform(location, carla.Rotation(pitch=at.pitch, yaw=yaw, roll=at.roll))

    def _spawn_scene_actor(self, spec: ActorSpec) -> Optional[Any]:
        library = self.world.get_blueprint_library()
        blueprint = _find_blueprint(library, spec.blueprint)
        if blueprint is None:
            print("[carla] Warning: blueprint '{}' does not exist; skipped.".format(spec.blueprint))
            return None
        for key, value in spec.attributes.items():
            if blueprint.has_attribute(key):
                blueprint.set_attribute(key, value)
        if blueprint.has_attribute("is_invincible"):
            blueprint.set_attribute("is_invincible", "false")
        kind = "walker" if spec.is_walker else "vehicle" if spec.is_vehicle else "prop"
        transform = self._placement_transform(spec.at, kind)
        actor = self.world.try_spawn_actor(blueprint, transform)
        if actor is None:   # e.g. overlapping something: try a little higher
            transform.location.z += 0.5
            actor = self.world.try_spawn_actor(blueprint, transform)
        if actor is None:
            print("[carla] Warning: could not place '{}' at {}.".format(spec.blueprint, spec.at))
            return None
        self.spawned.append(actor)
        self.scene_actor_ids.add(actor.id)
        self.actor_names[actor.id] = spec.name
        if spec.is_vehicle:
            actor.set_simulate_physics(True)
            light_value = 0
            for name in spec.lights:
                light_value |= int(getattr(carla.VehicleLightState, name, 0))
            if light_value:
                actor.set_light_state(carla.VehicleLightState(light_value))
            actor.apply_control(carla.VehicleControl(hand_brake=True))
        elif spec.is_walker:
            if spec.walk_to is not None:
                target = self._placement_transform(spec.walk_to, "walker").location
                self.walker_targets[actor.id] = (target, spec.walk_speed)
        else:
            actor.set_simulate_physics(spec.simulate_physics)
        return actor

    def _move_walkers(self) -> None:
        """Walk scripted pedestrians in a straight line to their targets
        (CARLA's AI walkers only use sidewalks and crosswalks)."""
        for actor in self.spawned:
            target = self.walker_targets.get(actor.id)
            if target is None:
                continue
            location, speed = target
            if self._walker_stuck(actor):
                actor.apply_control(carla.WalkerControl(carla.Vector3D(0, 0, 0), 0.0))
                print("[carla] Pedestrian '{}' is blocked; it stops there.".format(
                    self.actor_names.get(actor.id) or actor.type_id))
            elif not _walk_towards(actor, location, speed):
                continue
            del self.walker_targets[actor.id]
            self.walker_progress.pop(actor.id, None)

    def _walk_step(self, walker, target, speed: float, progress: Dict[str, Any]) -> bool:
        """Walk ``walker`` toward ``target``; True once there.  Getting no closer for
        WALKER_NO_PROGRESS_SECONDS (a curb, a pole, the car's door): first a jump, then
        a step straight up onto the target point, as a person would."""
        now, here = time.monotonic(), walker.get_location()
        distance = math.hypot(target.x - here.x, target.y - here.y)
        if distance < 0.3:
            walker.apply_control(carla.WalkerControl(carla.Vector3D(0, 0, 0), 0.0))
            return True
        if distance < progress.get("best", float("inf")) - 0.2:
            progress.update(best=distance, since=now)
        elif now - progress.setdefault("since", now) > settings.WALKER_NO_PROGRESS_SECONDS:
            if not progress.get("jumped"):
                progress.update(jumped=True, since=now)
                walker.apply_control(carla.WalkerControl(
                    carla.Vector3D((target.x - here.x) / distance, (target.y - here.y) / distance, 0.0), speed, True))
                return False
            walker.set_location(carla.Location(target.x, target.y, target.z + SPAWN_HEIGHT["walker"]))
            walker.apply_control(carla.WalkerControl(carla.Vector3D(0, 0, 0), 0.0))
            progress["stepped"] = True
            print("[carla] Pedestrian stepped up onto the curb / past the obstacle.")
            return True
        walker.apply_control(carla.WalkerControl(
            carla.Vector3D((target.x - here.x) / distance, (target.y - here.y) / distance, 0.0), speed))
        return False

    def _walker_stuck(self, walker) -> bool:
        """True once a scripted walker has not moved for WALKER_STUCK_SECONDS."""
        now, here = time.monotonic(), walker.get_location()
        last = self.walker_progress.get(walker.id)
        if last is None or here.distance(last[0]) > 0.3:
            self.walker_progress[walker.id] = (here, now)
            return False
        if now - last[1] < settings.WALKER_STUCK_SECONDS:
            return False
        self.walker_progress[walker.id] = (here, now)   # count again from here
        return True

    # --- passenger drop-off at the route end ----------------------------------

    def _start_dropoff(self) -> bool:
        """Let the passenger out on the curb side.  False if there is no drop-off."""
        if not settings.DROP_OFF_PASSENGER_AT_ROUTE_END or self.passenger_dropped:
            return False
        self.passenger_dropped = True
        transform = self.ego.get_transform()
        forward, right = transform.get_forward_vector(), transform.get_right_vector()
        blueprint = self._passenger_blueprint(self.passenger_look)
        walker, door = None, None
        for gap in (0.5, 1.0, 1.5):     # beside the rear door; further out if something is in the way
            door = self._beside_rear_door(gap, SPAWN_HEIGHT["walker"])
            walker = self.world.try_spawn_actor(
                blueprint, carla.Transform(door, carla.Rotation(yaw=transform.rotation.yaw + 90.0)))
            if walker is not None:
                break
        if walker is None:
            print("[carla] Warning: no room beside the car for the passenger to get out; drop-off skipped.")
            return False
        self.spawned.append(walker)
        self._set_passenger_door(True)
        self.dropoff = {"walker": walker, "path": self._passenger_path(door, forward, right),
                        "started": time.monotonic(), "door_open": True}
        print("[carla] Passenger getting out ({}).".format(walker.type_id))
        return True

    def _passenger_blueprint(self, look: str = ""):
        """The pedestrian blueprint ``look`` (so a passenger looks the same getting in and out), else a random one."""
        library = self.world.get_blueprint_library()
        blueprint = _find_blueprint(library, look) if look else None
        if blueprint is None:
            blueprint = random.choice(library.filter("walker.pedestrian.*"))
        if blueprint.has_attribute("is_invincible"):
            blueprint.set_attribute("is_invincible", "false")
        return blueprint

    def _beside_rear_door(self, gap: float, height: float = 0.0):
        """Point ``gap`` metres out from the car's right side, level with the rear door."""
        transform = self.ego.get_transform()
        forward, right = transform.get_forward_vector(), transform.get_right_vector()
        side = self.ego.bounding_box.extent.y + gap
        return transform.location + carla.Location(right.x * side - forward.x * 0.4,
                                                   right.y * side - forward.y * 0.4, height)

    def _passenger_ids(self) -> set:
        """The passengers getting out or in: never obstacles, never collisions."""
        return {trip["walker"].id for trip in (self.dropoff, self.pickup) if trip is not None}

    def _passenger_path(self, door, forward, right) -> List[Any]:
        """Points the passenger walks through: to the sidewalk, then along it in the driving direction."""
        away = settings.PASSENGER_WALK_AWAY_METERS
        sidewalk = self.map.get_waypoint(door, project_to_road=True, lane_type=carla.LaneType.Sidewalk)
        if sidewalk is None or sidewalk.transform.location.distance(door) > settings.PASSENGER_MAX_SIDEWALK_DISTANCE_METERS:
            curb = door + carla.Location(right.x * 2.5, right.y * 2.5, 0.0)
            return [curb, curb + carla.Location(forward.x * away, forward.y * away, 0.0)]
        along = sidewalk.transform.get_forward_vector()
        same_direction = along.x * forward.x + along.y * forward.y >= 0.0
        path, current = [sidewalk.transform.location], sidewalk
        for _ in range(int(away / 3.0)):
            options = current.next(3.0) if same_direction else current.previous(3.0)
            if not options:
                break
            current = options[0]
            path.append(current.transform.location)
        return path

    def _update_dropoff(self) -> None:
        """Walk the passenger away; remove them when gone, which completes the route."""
        dropoff = self.dropoff
        if dropoff is None:
            return
        walker, path = dropoff["walker"], dropoff["path"]
        elapsed = time.monotonic() - dropoff["started"]
        if dropoff["door_open"] and elapsed >= settings.PASSENGER_DOOR_OPEN_SECONDS:
            self._set_passenger_door(False)
            dropoff["door_open"] = False
        if path and self._walk_step(walker, path[0], settings.PASSENGER_WALK_SPEED_MPS,
                                    dropoff.setdefault("progress", {})):
            path.pop(0)
            # One step-up is a curb; needing another one straight after means the
            # passenger cannot walk here at all: end the walk.
            stepped = dropoff["progress"].get("stepped", False)
            if stepped and dropoff.get("stepped_last"):
                del path[:]
            dropoff["stepped_last"] = stepped
            dropoff["progress"] = {}
        if path and elapsed < settings.PASSENGER_DROPOFF_TIMEOUT_SECONDS:
            return
        if dropoff["door_open"]:
            self._set_passenger_door(False)
        self.dropoff = None
        self.walker_progress.pop(walker.id, None)
        self.destroy_actor(walker)
        print("[carla] Passenger dropped off{}.".format(
            " (time limit reached before the end of the walk)" if path else ""))
        if self.mode == DrivingMode.DROPPING_OFF:
            self.mode = DrivingMode.ROUTE_COMPLETE

    def _set_passenger_door(self, open_door: bool) -> None:
        try:
            if open_door:
                self.ego.open_door(carla.VehicleDoor.RR)
            else:
                self.ego.close_door(carla.VehicleDoor.RR)
        except (AttributeError, RuntimeError):
            pass   # CARLA before 0.9.13, or a car model without opening doors

    # --- picking up the next passenger -----------------------------------------

    def _spawn_waiting_passenger(self) -> None:
        """A pedestrian waiting on the sidewalk beside the curb lane where the route ends."""
        self.pickup = None
        curb_lane, _ = _rightmost_lane(self.route.waypoints[-1])
        road = curb_lane.transform
        forward, right = road.get_forward_vector(), road.get_right_vector()
        sidewalk = self.map.get_waypoint(road.location, project_to_road=True, lane_type=carla.LaneType.Sidewalk)
        if sidewalk is not None and \
                sidewalk.transform.location.distance(road.location) <= settings.PASSENGER_MAX_SIDEWALK_DISTANCE_METERS:
            spot = sidewalk.transform.location
        else:
            side = curb_lane.lane_width / 2.0 + 1.5
            spot = road.location + carla.Location(right.x * side, right.y * side, 0.0)
        blueprint = self._passenger_blueprint()
        for back in (0.0, 1.5, 3.0):    # slightly earlier along the road if the spot is taken
            location = spot + carla.Location(-forward.x * back, -forward.y * back, SPAWN_HEIGHT["walker"])
            walker = self.world.try_spawn_actor(
                blueprint, carla.Transform(location, carla.Rotation(yaw=road.rotation.yaw - 90.0)))   # facing the road
            if walker is not None:
                self.spawned.append(walker)
                self.pickup = {"walker": walker, "boarding": False}
                return
        print("[carla] Warning: no room for the waiting passenger; the car will stop there anyway.")

    def _start_boarding(self) -> None:
        """Stopped at the pickup point: the waiting passenger walks to the rear door and gets in."""
        pickup = self.pickup
        if pickup is None:               # nobody could be placed: an unseen passenger gets in
            self.passenger_look = ""
            self._start_destination_leg()
            return
        if not pickup["boarding"]:
            target = self._beside_rear_door(0.4)
            walk = pickup["walker"].get_location().distance(target)     # longer if the car pulled forward
            pickup.update(boarding=True, target=target, started=time.monotonic(),
                          timeout=max(settings.PASSENGER_BOARDING_TIMEOUT_SECONDS,
                                      walk / settings.PASSENGER_WALK_SPEED_MPS + 8.0))
            self.walker_progress.pop(pickup["walker"].id, None)
        self._set_passenger_door(True)
        print("[carla] Passenger getting in.")

    def _update_pickup(self) -> None:
        """Walk the boarding passenger to the door; once there (or blocked), they are in the car."""
        pickup = self.pickup
        if pickup is None or not pickup["boarding"]:
            return
        walker = pickup["walker"]
        arrived = self._walk_step(walker, pickup["target"], settings.PASSENGER_WALK_SPEED_MPS,
                                  pickup.setdefault("progress", {}))
        if not arrived and time.monotonic() - pickup["started"] < pickup.get("timeout", settings.PASSENGER_BOARDING_TIMEOUT_SECONDS):
            return
        self.pickup = None
        self.passenger_look = walker.type_id   # the same person gets out at the destination
        self.walker_progress.pop(walker.id, None)
        self.destroy_actor(walker)
        self._set_passenger_door(False)
        print("[carla] Passenger on board ({}).".format(walker.type_id))
        if self.mode == DrivingMode.PICKING_UP:   # otherwise the operator has taken over; Resume AV continues
            self._start_destination_leg()

    def _start_destination_leg(self) -> None:
        self.route_end_action = "dropoff"
        self.passenger_dropped = False
        self.state.trip_leg = "destination"
        self._follow_new_route(self.next_trip_length_m)
        print("[carla] Driving the passenger to their destination ({:.0f} m).".format(
            self.route.end_distance - self.route.cumulative[0]))

    def _on_collision(self, event) -> None:
        """Runs on CARLA's thread.  A passenger brushing the car while getting out or in is not a collision."""
        if event.other_actor.id in self._passenger_ids():
            return
        self._collisions.append(event.other_actor.type_id)

    def _spawn_traffic(self, vehicles: int, walkers: int) -> None:
        """Background traffic driven by CARLA's Traffic Manager and walker AI."""
        self._wanted_cars, self._wanted_walkers = max(0, vehicles), max(0, walkers)
        if vehicles <= 0 and walkers <= 0:
            return
        traffic_manager = self.client.get_trafficmanager()
        self._traffic_manager = traffic_manager
        traffic_manager.set_synchronous_mode(False)
        for setting, value in (("set_global_distance_to_leading_vehicle", settings.NPC_FOLLOW_DISTANCE_METERS),
                               ("global_percentage_speed_difference", settings.NPC_SPEED_BELOW_LIMIT_PERCENT),
                               ("set_respawn_dormant_vehicles", False)):      # never remove cars far away
            try:
                getattr(traffic_manager, setting)(value)
            except (AttributeError, RuntimeError):
                pass
        cars = sum(self._spawn_npc_car(away_from_ego=15.0) for _ in range(vehicles))
        people = sum(self._spawn_npc_walker() for _ in range(walkers))
        print("[carla] Background traffic: {} of {} cars, {} of {} pedestrians.".format(cars, vehicles, people, walkers))

    def _npc_spawn_points(self, away_from_ego: float) -> List[Any]:
        """Free spawn points at least ``away_from_ego`` from the AV and 25 m from the scene's obstacles."""
        ego = self.ego.get_location()
        avoid = [a.get_location() for a in self.spawned if a.id in self.scene_actor_ids]
        taken = [car.get_location() for car in self._npc_cars]
        points = [p for p in self.map.get_spawn_points()
                  if p.location.distance(ego) > away_from_ego
                  and all(p.location.distance(spot) > 25.0 for spot in avoid)
                  and all(p.location.distance(spot) > 8.0 for spot in taken)]
        random.shuffle(points)
        return points

    def _spawn_npc_car(self, away_from_ego: float) -> bool:
        library = self.world.get_blueprint_library()
        blueprints = [b for b in library.filter("vehicle.*")
                      if b.has_attribute("number_of_wheels") and b.get_attribute("number_of_wheels").as_int() == 4]
        for point in self._npc_spawn_points(away_from_ego)[:10]:     # keep trying until one is free
            car = self.world.try_spawn_actor(random.choice(blueprints), point)
            if car is None:
                continue
            car.set_autopilot(True, self._traffic_manager.get_port())
            for setting, value in (("auto_lane_change", True), ("update_vehicle_lights", True)):
                try:
                    getattr(self._traffic_manager, setting)(car, value)    # pass stopped cars; lights at night
                except (AttributeError, RuntimeError):
                    pass
            self.spawned.append(car)
            self._npc_cars.append(car)
            return True
        return False

    def _spawn_npc_walker(self) -> bool:
        library = self.world.get_blueprint_library()
        for _ in range(5):
            location = self.world.get_random_location_from_navigation()
            if location is None:
                continue
            walker = self.world.try_spawn_actor(random.choice(library.filter("walker.pedestrian.*")),
                                                carla.Transform(location + carla.Location(z=1.0)))
            if walker is None:
                continue
            controller = self.world.spawn_actor(library.find("controller.ai.walker"), carla.Transform(), walker)
            self.spawned += [walker, controller]
            controller.start()
            controller.go_to_location(self.world.get_random_location_from_navigation())
            controller.set_max_speed(1.0 + random.random() * 0.6)
            self._npc_walkers.append((walker, controller))
            return True
        return False

    def _manage_traffic(self) -> None:
        """Once a second: keep the background traffic moving and complete.

        * A car standing still for NPC_STUCK_SECONDS (not at a red light) is
          nudged: a lane change, and -- out of the AV's way -- briefly pushing
          past the car that blocks it (gridlock).  Still stuck after two nudges
          and out of the operator's view: it is moved to a free spawn point.
        * A pedestrian standing idle gets a new destination.
        * A car or pedestrian that CARLA removed (fell through the map, ...) is
          reported and replaced out of view, so the traffic stays as set."""
        now = time.monotonic()
        if self._traffic_manager is None or now < self._next_traffic_check:
            return
        self._next_traffic_check = now + 1.0
        manager, ego = self._traffic_manager, self.ego.get_location()
        alive = []
        for car in self._npc_cars:
            if not car.is_alive:
                print("[carla] A background car was removed by CARLA (fell through the map or was destroyed); "
                      "replacing it out of view.")
                if car in self.spawned:
                    self.spawned.remove(car)
                continue
            alive.append(car)
            watch = self._npc_watch.setdefault(car.id, {"spot": car.get_location(), "since": now, "tries": 0,
                                                        "nudged_until": None})
            here = car.get_location()
            if watch["nudged_until"] is not None and now >= watch["nudged_until"]:
                manager.ignore_vehicles_percentage(car, 0)
                watch["nudged_until"] = None
            if here.distance(watch["spot"]) > 1.0:
                watch.update(spot=here, since=now, tries=0)
                continue
            if now - watch["since"] < settings.NPC_STUCK_SECONDS:
                continue
            watch["since"] = now
            try:
                if car.is_at_traffic_light() and car.get_traffic_light_state() != carla.TrafficLightState.Green:
                    continue                                   # just waiting for its light
            except (AttributeError, RuntimeError):
                pass
            distance = here.distance(ego)
            if watch["tries"] < 2:
                watch["tries"] += 1
                try:
                    manager.force_lane_change(car, watch["tries"] == 1)    # left first, then right
                except (AttributeError, RuntimeError):
                    pass
                if distance > settings.NPC_NUDGE_MIN_DISTANCE_METERS:     # never pushes past near the AV
                    manager.ignore_vehicles_percentage(car, 100)
                    watch["nudged_until"] = now + 3.0
                continue
            if distance > settings.NPC_RELOCATE_MIN_DISTANCE_METERS:
                points = self._npc_spawn_points(settings.NPC_RELOCATE_MIN_DISTANCE_METERS)
                if points:
                    car.set_transform(points[0])
                    watch.update(spot=points[0].location, since=now, tries=0)
                    print("[carla] A background car stuck out of view was moved to a free spot.")
        self._npc_cars = alive
        for _ in range(self._wanted_cars - len(self._npc_cars)):
            self._spawn_npc_car(away_from_ego=settings.NPC_RELOCATE_MIN_DISTANCE_METERS)

        walkers = []
        for walker, controller in self._npc_walkers:
            if not walker.is_alive:
                print("[carla] A background pedestrian was removed by CARLA; replacing it.")
                for actor in (walker, controller):
                    if actor in self.spawned:
                        self.spawned.remove(actor)
                try:
                    controller.stop()
                    controller.destroy()
                except RuntimeError:
                    pass
                continue
            walkers.append((walker, controller))
            watch = self._npc_watch.setdefault(walker.id, {"spot": walker.get_location(), "since": now})
            if walker.get_location().distance(watch["spot"]) > 0.5:
                watch.update(spot=walker.get_location(), since=now)
            elif now - watch["since"] > settings.NPC_PEDESTRIAN_IDLE_SECONDS:
                controller.go_to_location(self.world.get_random_location_from_navigation())   # off again
                watch["since"] = now
        self._npc_walkers = walkers
        for _ in range(self._wanted_walkers - len(self._npc_walkers)):
            self._spawn_npc_walker()

    def _attach_sensors(self, cameras: Sequence[CameraRequest]) -> None:
        library = self.world.get_blueprint_library()
        collision = self.world.spawn_actor(library.find("sensor.other.collision"), carla.Transform(), attach_to=self.ego)
        collision.listen(self._on_collision)
        self.spawned.append(collision)
        for request in cameras:
            mount = settings.CAMERA_MOUNTS.get(request.name)
            if mount is None:
                continue
            blueprint = library.find("sensor.camera.rgb")
            blueprint.set_attribute("image_size_x", str(request.width))
            blueprint.set_attribute("image_size_y", str(request.height))
            blueprint.set_attribute("fov", str(mount["fov"]))
            blueprint.set_attribute("sensor_tick", str(CAMERA_SENSOR_TICK_SECONDS))
            transform = carla.Transform(carla.Location(mount["x"], mount["y"], mount["z"]),
                                        carla.Rotation(pitch=mount["pitch"], yaw=mount["yaw"]))
            camera = self.world.spawn_actor(blueprint, transform, attach_to=self.ego)
            camera.listen(self._camera_callback(request.name))
            self.cameras[request.name] = camera
            self.spawned.append(camera)

    def _camera_callback(self, name: str):
        """Runs on CARLA's thread: convert BGRA bytes to an RGB array and store it."""
        def callback(image) -> None:
            array = numpy.frombuffer(image.raw_data, dtype=numpy.uint8).reshape((image.height, image.width, 4))
            rgb = array[:, :, 2::-1].copy()
            with self._frame_lock:
                self._frames[name] = (time.monotonic(), rgb)
        return callback

    # --- state and obstacle checks --------------------------------------------

    def _update_state(self) -> None:
        state = self.state
        transform = self.ego.get_transform()
        location = transform.location
        velocity = self.ego.get_velocity()
        state.speed_mps = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        state.x, state.y, state.z = location.x, location.y, location.z
        state.yaw_degrees = transform.rotation.yaw
        if self._tick_count % 10 == 1:
            geo = self.map.transform_to_geolocation(location)
            state.latitude, state.longitude = geo.latitude, geo.longitude
        if self.route is not None:
            route_m, _, index = self.route.project(location.x, location.y, near_index=self.route_index)
            self.route_index = index
            # Progress never goes backwards (e.g. when reversing manually).
            state.route_traveled_m = max(state.route_traveled_m, route_m)
            state.route_length_m = self.route.end_distance
        while self._collisions:
            state.collision_count += 1
            state.last_collision_with = self._collisions.pop(0)
            if self.mode not in (DrivingMode.MANUAL, DrivingMode.EMERGENCY_STOP, DrivingMode.DROPPING_OFF,
                                 DrivingMode.PICKING_UP):
                self.soft_stop("")
        state.mode = self.mode
        state.stop_reason = self.stop_reason if self.mode in (DrivingMode.STOPPING, DrivingMode.HOLDING) else ""

    def _actors_in_path(self, reach: float, walkers: bool = True) -> List[Any]:
        """Scene props/vehicles (and any pedestrians) in the ego's lane within ``reach`` metres."""
        if self.route is None:
            return []
        ego_location = self.ego.get_location()
        ego_m, ego_lateral, _ = self.route.project(ego_location.x, ego_location.y, near_index=self.route_index)
        found = []
        candidates = [a for a in self.spawned if a.id in self.scene_actor_ids and a.id not in self.ignored_obstacles]
        if walkers:
            passengers = self._passenger_ids()
            candidates += [a for a in self.world.get_actors().filter("walker.pedestrian.*")
                           if a not in candidates and a.id not in passengers]
        for actor in candidates:
            if actor.type_id.startswith("walker.") and not walkers:
                continue
            location = actor.get_location()
            if location.distance(ego_location) > reach + 10:
                continue
            actor_m, actor_lateral, _ = self.route.project(location.x, location.y, near_index=self.route_index, window=40)
            ahead = actor_m - ego_m
            if 2.0 < ahead < reach and abs(actor_lateral - ego_lateral) < settings.OBSTACLE_LANE_HALF_WIDTH_METERS:
                found.append(actor)
        return found

    def _scan_path(self) -> Tuple[Optional[Any], bool]:
        """(first prop/vehicle blocking the lane or None, pedestrian in the path?)"""
        reach = settings.OBSTACLE_DETECTION_DISTANCE_METERS + self.state.speed_mps * 1.2
        blocking, walker = None, False
        for actor in self._actors_in_path(reach):
            if actor.type_id.startswith("walker."):
                walker = True
            elif blocking is None:
                blocking = actor
        return blocking, walker


# =============================================================================
# Small helpers
# =============================================================================

def _short_map_name(name: str) -> str:
    """'/Game/Carla/Maps/Town10HD_Opt' -> 'Town10HD_Opt'."""
    return name.replace("\\", "/").rstrip("/").split("/")[-1]


def _find_blueprint(library, name: str):
    try:
        return library.find(name)
    except (IndexError, RuntimeError):
        matches = library.filter(name)
        return matches[0] if len(matches) else None


def _lanes_over(waypoint, lanes: int, driving_only: bool = False):
    """The waypoint ``lanes`` lanes to the right (negative = left) of ``waypoint``,
    as seen in ``waypoint``'s direction of travel, or None if there is no such lane.

    CARLA's get_left_lane()/get_right_lane() are relative to each waypoint's
    OWN direction, and get_left_lane() of lane +-1 crosses the centre line into
    the oncoming lane.  Once across, "our" left is that lane's right, so the
    step direction is flipped whenever the lane-id sign changes.
    """
    our_side = waypoint.lane_id > 0
    current = waypoint
    for _ in range(abs(lanes)):
        go_right = (lanes > 0) == ((current.lane_id > 0) == our_side)
        neighbour = current.get_right_lane() if go_right else current.get_left_lane()
        if neighbour is None or (driving_only and neighbour.lane_type != carla.LaneType.Driving):
            return None
        current = neighbour
    return current


def _rightmost_lane(waypoint):
    """(rightmost driving lane in ``waypoint``'s direction of travel, how many lanes to the right it is)."""
    rightmost, lanes = waypoint, 0
    while True:
        right = rightmost.get_right_lane()
        if right is None or right.lane_type != carla.LaneType.Driving or \
                (right.lane_id > 0) != (waypoint.lane_id > 0):
            return rightmost, lanes
        rightmost, lanes = right, lanes + 1


def _walk_towards(walker, location, speed: float) -> bool:
    """Steer a walker straight at ``location``.  True (and standing still) once it is there."""
    here = walker.get_location()
    dx, dy = location.x - here.x, location.y - here.y
    distance = math.hypot(dx, dy)
    if distance < 0.3:
        walker.apply_control(carla.WalkerControl(carla.Vector3D(0, 0, 0), 0.0))
        return True
    walker.apply_control(carla.WalkerControl(carla.Vector3D(dx / distance, dy / distance, 0.0), speed))
    return False


def _yaw_change(a, b) -> float:
    """Signed heading change from waypoint a to b in degrees (positive = right turn)."""
    return (b.transform.rotation.yaw - a.transform.rotation.yaw + 180.0) % 360.0 - 180.0


def _junction_id(waypoint) -> Optional[int]:
    try:
        return waypoint.get_junction().id
    except (AttributeError, RuntimeError):
        return None


def _densify(points: List[Tuple[float, float, float]], step: float = 1.0) -> List[Tuple[float, float, float]]:
    """Fill in points (x, y, distance) so that they are at most ``step`` metres apart."""
    out = points[:1]
    for (x1, y1, s1), (x2, y2, s2) in zip(points, points[1:]):
        gap = math.hypot(x2 - x1, y2 - y1)
        pieces = int(gap / step)
        for k in range(1, pieces + 1):
            t = k / (pieces + 1.0)
            out.append((x1 + (x2 - x1) * t, y1 + (y2 - y1) * t, s1 + (s2 - s1) * t))
        out.append((x2, y2, s2))
    return out


def _first_conflict(ours, theirs, width: float):
    """Where two paths (lists of (x, y, distance)) come closer than ``width``:
    (our first and last distance there, theirs, merging into the same lane?) or None."""
    if not ours or not theirs:
        return None
    xs, ys = [p[0] for p in ours], [p[1] for p in ours]
    low_x, high_x, low_y, high_y = min(xs) - width, max(xs) + width, min(ys) - width, max(ys) + width
    near = [p for p in theirs if low_x <= p[0] <= high_x and low_y <= p[1] <= high_y]
    if not near:
        return None
    limit = width * width
    our_in = our_out = their_in = their_out = None
    for ox, oy, os_ in ours:
        for tx, ty, ts in near:
            if (ox - tx) ** 2 + (oy - ty) ** 2 < limit:
                if our_in is None:
                    our_in = os_
                our_out = os_
                their_in = ts if their_in is None else min(their_in, ts)
                their_out = ts if their_out is None else max(their_out, ts)
    if our_in is None:
        return None
    # Merging: at the end both paths run the same way, side by side.
    a, b = ours[-1], ours[max(0, len(ours) - 4)]
    c = min(theirs, key=lambda p: (p[0] - a[0]) ** 2 + (p[1] - a[1]) ** 2)
    k = theirs.index(c)
    d = theirs[max(0, k - 3)]
    ux, uy, vx, vy = a[0] - b[0], a[1] - b[1], c[0] - d[0], c[1] - d[1]
    norm = math.hypot(ux, uy) * math.hypot(vx, vy)
    merging = norm > 0 and (ux * vx + uy * vy) / norm > 0.8 and (c[0] - a[0]) ** 2 + (c[1] - a[1]) ** 2 < limit
    return our_in, our_out, their_in, their_out, merging


def _time_to_cover(distance: float, speed: float, accel: float, top: float) -> float:
    """Seconds to drive ``distance`` metres from ``speed``, speeding up at ``accel`` to ``top``."""
    if distance <= 0.0:
        return 0.0
    speed, top = max(0.0, speed), max(top, speed, 0.1)
    if speed >= top:
        return distance / top
    ramp_t = (top - speed) / accel
    ramp_d = (speed + top) / 2.0 * ramp_t
    if distance <= ramp_d:
        return (-speed + math.sqrt(speed * speed + 2.0 * accel * distance)) / accel
    return ramp_t + (distance - ramp_d) / top


def _describe(actor) -> str:
    name = actor.type_id.split(".")[-1]
    return {"constructioncone": "cone", "streetbarrier": "barrier", "dirtdebris01": "debris"}.get(name, name)
