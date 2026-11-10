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
        self.ignored_obstacles: set = set()
        self.spawned: List[Any] = []                # everything we must destroy
        self.scene_actor_ids: set = set()           # props/vehicles placed by the scene or tasks
        self.actor_names: Dict[int, str] = {}       # actor id -> name from the scene file
        self.walker_targets: Dict[int, Tuple[Any, float]] = {}   # walker id -> (target location, speed)
        self.walker_progress: Dict[int, Tuple[Any, float]] = {}  # walker id -> (where it last moved, when)
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
        self._junction_asked: Optional[Dict[str, Any]] = None   # blocked intersection the operator was asked about
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
        self._set_agent_plan(self.route.waypoints)
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
        control = self._control_for_mode()
        self.ego.apply_control(control)
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
            # Moving toward the curb: still wait for pedestrians crossing in front.
            if self._tick_count % OBSTACLE_CHECK_EVERY_TICKS == 0:
                self._yielding = self._scan_path()[1]
            if self._yielding:
                return carla.VehicleControl(throttle=0.0, brake=settings.SOFT_STOP_BRAKE)
            return self.agent.run_step()

        # --- autonomous -------------------------------------------------------
        if self.lane_return_at_m is not None and self.state.route_traveled_m >= self.lane_return_at_m:
            self.lane_return_at_m = None
            self._shift_plan(0)      # back to the original lane
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
        self._note_blocked(self._agent_hazard(control))
        return control

    def _agent_hazard(self, control) -> str:
        """Why CARLA's BasicAgent is braking for a hazard: "red light", "vehicle ahead" or ""."""
        if control.throttle > 0.0 or control.brake <= 0.0 or self.state.speed_mph > 3.0:
            return ""
        agent, speed = self.agent, self.state.speed_mps
        try:
            if agent._affected_by_traffic_light(agent._lights_list,
                                                agent._base_tlight_threshold + agent._speed_ratio * speed)[0]:
                return "red light"
            if agent._vehicle_obstacle_detected(self.world.get_actors().filter("*vehicle*"),
                                                agent._base_vehicle_threshold + agent._speed_ratio * speed)[0]:
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

    def _note_blocked(self, reason: str) -> None:
        """Show why the AV is standing still; if a vehicle or pedestrian keeps it there
        for AV_BLOCKED_ASK_SECONDS, stop and ask the operator (red lights just wait)."""
        if reason:
            self.state.waiting_for = reason
        stopped = self.state.speed_mph < settings.STOPPED_SPEED_MPH
        if reason == "vehicle ahead" and self._in_queue():
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
        self._light_locations = [light.get_location()
                                 for light in self.world.get_actors().filter("traffic.traffic_light")]
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
            junctions.append({"entry_m": entry_m, "exit_m": exit_m, "center": center, "radius": radius,
                              "id": junction_id,
                              "turn": _yaw_change(before, after), "signalized": signalized, "stop_m": stop_m,
                              "stopped_at": None, "stop_done": False, "cleared": False, "waiting_since": None})
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
        junction = next((j for j in self._route_junctions()
                         if front - 0.5 < j["entry_m"] <= front + settings.INTERSECTION_LOOKAHEAD_METERS), None)
        if junction is not None and not junction["signalized"]:
            now = time.monotonic()
            stopped = self.state.speed_mph < settings.STOPPED_SPEED_MPH
            desired_kmh = min(desired_kmh, settings.INTERSECTION_APPROACH_SPEED_MPH * MPH_TO_KMH)
            stop_sign = settings.OBEY_STOP_SIGNS and junction["stop_m"] is not None
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
            if settings.YIELD_AT_INTERSECTIONS and not junction["cleared"] and \
                    (not stop_sign or junction["stop_done"]):
                to_entry = junction["entry_m"] - front
                speed = self.state.speed_mps
                if to_entry <= max(12.0, 3.0 + speed * speed / 6.0):   # early enough to stop gently at the line
                    other = self._cross_traffic(junction)
                    if other is None:
                        junction["waiting_since"] = None
                        if stopped or to_entry <= 3.5:
                            junction["cleared"] = True          # committed: no second-guessing mid-crossing
                    else:
                        self.state.waiting_for = "cross traffic"
                        if junction["waiting_since"] is None:
                            junction["waiting_since"] = now
                            print("[carla] Yielding to {} at an intersection.".format(_describe(other)))
                        elif now - junction["waiting_since"] > settings.YIELD_MAX_WAIT_SECONDS:
                            self._junction_asked = junction
                            self.soft_stop("Intersection blocked by {}".format(_describe(other)))
                        if to_entry > 3.0:        # not at the line yet: roll up to it, then hold
                            desired_kmh = min(desired_kmh, max(3.0, math.sqrt(
                                2.0 * settings.STOP_SIGN_DECELERATION_MPS2 * (to_entry - 2.5)) * 3.6))
                        else:
                            return carla.VehicleControl(throttle=0.0, brake=settings.SOFT_STOP_BRAKE)
        self.agent.set_target_speed(desired_kmh)
        return None

    def _cross_traffic(self, junction: Dict[str, Any]) -> Optional[Any]:
        """A vehicle the AV must give way to at this intersection, or None.

        Uses the road network: a vehicle counts if it is inside this very
        intersection (moving, or standing on or next to our path), or if its lane
        runs into this intersection and it will be there within YIELD_GAP_SECONDS.
        Parked cars, cars waiting at their own line, and cars on roads that do not
        lead in do not count."""
        ego = self.ego.get_transform()
        forward = ego.get_forward_vector()
        center, radius = junction["center"], junction["radius"]
        turning_left = junction["turn"] < -30.0
        for vehicle in self.world.get_actors().filter("vehicle.*"):
            if vehicle.id == self.ego.id or vehicle.id in self.scene_actor_ids:
                continue   # placed obstacles are handled by the obstacle check
            location = vehicle.get_location()
            to_center = math.hypot(center.x - location.x, center.y - location.y)
            if to_center > radius + settings.YIELD_WATCH_METERS:
                continue
            if (location.x - ego.location.x) * forward.x + (location.y - ego.location.y) * forward.y < -1.0:
                continue   # behind us
            velocity = vehicle.get_velocity()
            speed = math.hypot(velocity.x, velocity.y)
            along = velocity.x * forward.x + velocity.y * forward.y
            if speed > 0.5 and along > 0.7 * speed:
                continue   # going our way (ahead of us): BasicAgent keeps its distance
            if speed > 0.5 and along < -0.7 * speed and not turning_left:
                continue   # oncoming: only in our way when we turn left across its lane
            if junction.get("id") is not None:
                if self._conflicts_in_junction(vehicle, location, speed, junction):
                    return vehicle
                continue
            if to_center <= radius:
                return vehicle             # in the intersection
            if speed < 1.0:
                continue                   # waiting at its own stop line / light
            closing = (velocity.x * (center.x - location.x) + velocity.y * (center.y - location.y)) / max(to_center, 0.1)
            if closing > 0.5 * speed and (to_center - radius) / closing < settings.YIELD_GAP_SECONDS:
                return vehicle             # arriving before we could be across
        return None

    def _conflicts_in_junction(self, vehicle, location, speed: float, junction: Dict[str, Any]) -> bool:
        """Road-network test for ``_cross_traffic`` (see there)."""
        lane = self.map.get_waypoint(location)
        if lane is None:
            return False
        if lane.is_junction and lane.get_junction().id == junction["id"]:
            if speed > 0.5:
                return True                # crossing right now
            # Standing in the intersection: only in the way if on or next to our path through it.
            path_m, lateral, _ = self.route.project(location.x, location.y)
            return junction["entry_m"] - 2.0 <= path_m <= junction["exit_m"] + 2.0 and abs(lateral) < 3.5
        if speed < 1.0:
            return False                   # waiting at its own stop line / light, or parked
        velocity, along = vehicle.get_velocity(), lane.transform.get_forward_vector()
        if velocity.x * along.x + velocity.y * along.y < 0.5 * speed:
            return False                   # not driving along that lane (matched to the lane beside it)
        reach = min(settings.YIELD_WATCH_METERS, speed * settings.YIELD_GAP_SECONDS)
        distance = 2.0
        while distance <= reach:           # does its lane run into this intersection that soon?
            if any(ahead.is_junction and ahead.get_junction().id == junction["id"] for ahead in lane.next(distance)):
                return True
            distance += 2.0
        return False

    # =====================================================================
    # Commands
    # =====================================================================

    def resume_autonomy(self, obstacles_checked: bool = True) -> None:
        if self.ego is None:
            return
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

    def _steer_to_curb(self) -> None:
        """Follow the rest of the plan along the right edge of the rightmost lane, slowly."""
        # 1. Move to the rightmost driving lane (same direction of travel).
        route_waypoint = self.route.waypoints[min(self.route_index, len(self.route.waypoints) - 1)]
        rightmost, lanes_to_right = _rightmost_lane(route_waypoint)
        self._shift_plan(lanes_to_right)
        # 2. Drive along the right edge of that lane (or on the shoulder / parking lane).
        offset = rightmost.lane_width / 2.0 - 1.0 - settings.PULL_OVER_EDGE_MARGIN_METERS
        right = rightmost.get_right_lane()
        if right is not None and right.lane_type in (carla.LaneType.Shoulder, carla.LaneType.Parking):
            offset += right.lane_width * 0.6
        self.agent.set_offset(max(0.0, offset))
        self.agent.set_target_speed(min(self.target_speed_mph, settings.PULL_OVER_SPEED_MPH) * MPH_TO_KMH)

    def change_lane(self, direction: str) -> bool:
        if self.ego is None:
            return False
        here = self.map.get_waypoint(self.ego.get_location())
        target = here.get_left_lane() if direction == "left" else here.get_right_lane()
        if target is None or target.lane_type != carla.LaneType.Driving:
            return False
        opposite = (target.lane_id > 0) != (here.lane_id > 0)
        self._shift_plan(-1 if direction == "left" else 1)
        if opposite:   # do not stay on the wrong side of the road
            self.lane_return_at_m = self.state.route_traveled_m + OPPOSITE_LANE_RETURN_AFTER_METERS
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
        self._set_agent_plan(self.route.waypoints)
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
        self._set_agent_plan(self.route.waypoints)
        self.set_hazard_lights(False)
        self.stop_reason = ""
        self.mode = DrivingMode.AUTONOMOUS

    def set_manual_control(self, enabled: bool) -> None:
        if enabled:
            self.mode = DrivingMode.MANUAL
            self.manual_input = (0.0, 0.0, settings.MANUAL_COAST_BRAKE, False)
        elif self.mode == DrivingMode.MANUAL:
            self.mode = DrivingMode.HOLDING
            self._replan_from_here()

    def apply_manual_input(self, throttle: float, steer: float, brake: float, reverse: bool) -> None:
        self.manual_input = (throttle, steer, brake, reverse)

    def set_hazard_lights(self, on: bool) -> None:
        self.state.hazard_lights_on = on
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

    def _shift_plan(self, lanes: int) -> None:
        """Follow the remaining route ``lanes`` lanes to the right (negative = left), 0 = original lane."""
        remaining = self._remaining_route_waypoints()
        skip = int(LANE_CHANGE_SKIP_METERS / ROUTE_STEP_METERS)
        shifted = []
        for waypoint in remaining[skip:]:
            target = _lanes_over(waypoint, lanes, driving_only=True)
            shifted.append(target if target is not None else waypoint)
        if shifted:
            self._set_agent_plan(shifted)

    def _replan_from_here(self, minimum_length: float = 0.0) -> None:
        """New plan from the car's current position along the rest of the route."""
        if self.route is None:
            return
        _, _, index = self.route.project(self.ego.get_location().x, self.ego.get_location().y)
        remaining = self.route.waypoints[index + 1:]
        if len(remaining) * ROUTE_STEP_METERS < minimum_length:
            start = self.map.get_waypoint(self.ego.get_location())
            remaining = self._plan_route(start, RouteSpec({"length_m": minimum_length}))[1:]
        if remaining:
            self._set_agent_plan(remaining)

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
        if path and self._walker_stuck(walker):
            if dropoff.get("skipped"):   # blocked again: end the walk here
                del path[:]
            else:                        # blocked: head for the next point instead
                path.pop(0)
                dropoff["skipped"] = True
                print("[carla] Passenger blocked; walking on to the next point.")
        if path and _walk_towards(walker, path[0], settings.PASSENGER_WALK_SPEED_MPS):
            path.pop(0)
            dropoff["skipped"] = False
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
            pickup.update(boarding=True, target=self._beside_rear_door(0.4), started=time.monotonic())
            self.walker_progress.pop(pickup["walker"].id, None)
        self._set_passenger_door(True)
        print("[carla] Passenger getting in.")

    def _update_pickup(self) -> None:
        """Walk the boarding passenger to the door; once there (or blocked), they are in the car."""
        pickup = self.pickup
        if pickup is None or not pickup["boarding"]:
            return
        walker = pickup["walker"]
        arrived = self._walker_stuck(walker) or \
            _walk_towards(walker, pickup["target"], settings.PASSENGER_WALK_SPEED_MPS)
        if not arrived and time.monotonic() - pickup["started"] < settings.PASSENGER_BOARDING_TIMEOUT_SECONDS:
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
        if vehicles <= 0 and walkers <= 0:
            return
        library = self.world.get_blueprint_library()
        traffic_manager = self.client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(False)
        ego_location = self.ego.get_location()
        spawn_points = [p for p in self.map.get_spawn_points() if p.location.distance(ego_location) > 15.0]
        random.shuffle(spawn_points)
        car_blueprints = [b for b in library.filter("vehicle.*")
                          if b.has_attribute("number_of_wheels") and b.get_attribute("number_of_wheels").as_int() == 4]
        for point in spawn_points[:vehicles]:
            actor = self.world.try_spawn_actor(random.choice(car_blueprints), point)
            if actor is not None:
                actor.set_autopilot(True, traffic_manager.get_port())
                self.spawned.append(actor)
        controller_blueprint = library.find("controller.ai.walker")
        for _ in range(walkers):
            location = self.world.get_random_location_from_navigation()
            if location is None:
                continue
            walker = self.world.try_spawn_actor(random.choice(library.filter("walker.pedestrian.*")),
                                                carla.Transform(location + carla.Location(z=1.0)))
            if walker is None:
                continue
            self.spawned.append(walker)
            controller = self.world.spawn_actor(controller_blueprint, carla.Transform(), walker)
            self.spawned.append(controller)
            controller.start()
            controller.go_to_location(self.world.get_random_location_from_navigation())
            controller.set_max_speed(1.2)

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


def _describe(actor) -> str:
    name = actor.type_id.split(".")[-1]
    return {"constructioncone": "cone", "streetbarrier": "barrier", "dirtdebris01": "debris"}.get(name, name)
