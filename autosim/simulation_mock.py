"""
simulation_mock.py -- a CARLA-free stand-in simulation (``--mock``).

Why it exists
-------------
CARLA needs a big GPU machine.  The mock lets anyone run the full operator
interface, scenario files, task logic and data logging on a laptop, and it is
what the automated tests use.  It is deliberately simple:

* The world is the ego route: a road with three lanes (the ego lane, one lane
  to the right and an oncoming lane to the left).  Every object has a
  position ``s`` (metres along the route) and ``d`` (metres to the right of
  the ego lane centre).
* The camera images are simple pseudo-3-D drawings, not photographs.

It honours the same commands as the CARLA backend, so a scenario behaves the
same way in both (apart from looks and physics).
"""

import math
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy
import pygame

from autosim import settings
from autosim.scene import ActorSpec, Placement, Scene
from autosim.simulation import CameraRequest, DrivingMode, Simulation, VehicleState

LANE = settings.MOCK_LANE_WIDTH_METERS
LANE_OFFSETS = (-1, 0, 1)          # oncoming lane, ego lane, right lane
CAR_HALF_WIDTH = 0.95
CAR_HALF_LENGTH = 2.3
FLAT_PROP_WORDS = ("debris", "puddle", "manhole", "pothole")   # can be driven over
MPH = 0.44704                      # metres per second in one mph


class _MockActor:
    """Something placed in the mock world."""

    def __init__(self, blueprint: str, s: float, d: float, name: str = "") -> None:
        self.blueprint = blueprint
        self.name = name or blueprint
        self.s = s
        self.d = d
        self.target: Optional[Tuple[float, float]] = None    # walkers: (s, d) to walk to
        self.speed = 0.0
        self.hit = False

    @property
    def kind(self) -> str:
        if self.blueprint.startswith("walker."):
            return "walker"
        if self.blueprint.startswith("vehicle."):
            return "vehicle"
        return "prop"

    @property
    def is_flat(self) -> bool:
        return any(word in self.blueprint for word in FLAT_PROP_WORDS)

    @property
    def size(self) -> Tuple[float, float, float]:
        """(length, width, height) in metres -- used for drawing and collisions."""
        if self.blueprint == "scenery.building":
            return 14.0, 14.0, 9.0 + (int(abs(self.s)) % 3) * 4.0
        if self.kind == "walker":
            return 0.5, 0.5, 1.75
        if self.kind == "vehicle":
            return 4.6, 1.9, 1.6
        if "barrier" in self.blueprint or "barrel" in self.blueprint:
            return 0.6, 2.0, 1.1
        if self.is_flat:
            return 1.5, 1.5, 0.05
        if "box" in self.blueprint:
            return 0.8, 0.8, 0.6
        return 0.4, 0.4, 0.8     # cones and other small props

    def color(self) -> Tuple[int, int, int]:
        name = self.blueprint
        if name == "scenery.building":
            shade = 120 + (int(abs(self.s)) * 7) % 60
            return (shade, shade - 15, shade - 30)
        if self.kind == "walker":
            return (60, 170, 90)
        if "police" in name:
            return (25, 25, 35)
        if "ambulance" in name or "firetruck" in name:
            return (230, 230, 230)
        if self.kind == "vehicle":
            return (70, 110, 200)
        if "cone" in name:
            return (255, 130, 20)
        if "barrier" in name:
            return (230, 60, 50)
        if self.is_flat:
            return (85, 85, 95)
        return (170, 120, 70)

    def update(self, dt: float) -> None:
        if self.target is None or self.speed <= 0:
            return
        ds, dd = self.target[0] - self.s, self.target[1] - self.d
        distance = math.hypot(ds, dd)
        if distance < 0.05:
            self.target = None
            return
        step = min(distance, self.speed * dt)
        self.s += ds / distance * step
        self.d += dd / distance * step


class _Route:
    """A polyline with distance lookup: s -> (x, y, heading)."""

    def __init__(self, points: List[Tuple[float, float]]) -> None:
        self.points = points
        self.cumulative = [0.0]
        for (x1, y1), (x2, y2) in zip(points, points[1:]):
            self.cumulative.append(self.cumulative[-1] + math.hypot(x2 - x1, y2 - y1))

    @property
    def length(self) -> float:
        return self.cumulative[-1]

    def pose(self, s: float) -> Tuple[float, float, float]:
        """(x, y, heading in radians) at distance ``s`` (extrapolates beyond the ends)."""
        s_clamped = min(max(s, 0.0), self.length)
        index = 0
        while index < len(self.cumulative) - 2 and self.cumulative[index + 1] < s_clamped:
            index += 1
        (x1, y1), (x2, y2) = self.points[index], self.points[index + 1]
        segment = max(1e-6, self.cumulative[index + 1] - self.cumulative[index])
        fraction = (s_clamped - self.cumulative[index]) / segment
        heading = math.atan2(y2 - y1, x2 - x1)
        overshoot = s - s_clamped
        x = x1 + (x2 - x1) * fraction + math.cos(heading) * overshoot
        y = y1 + (y2 - y1) * fraction + math.sin(heading) * overshoot
        return x, y, heading

    def point(self, s: float, d: float) -> Tuple[float, float]:
        """Map position of route coordinates (s, d); d > 0 is to the right."""
        x, y, heading = self.pose(s)
        # Map frame like CARLA: x east, y south, heading clockwise from +x,
        # so the right-hand normal of heading h is (-sin h, cos h).
        return x - math.sin(heading) * d, y + math.cos(heading) * d

    def project(self, x: float, y: float) -> Tuple[float, float]:
        """Closest route coordinates (s, d) of map point (x, y)."""
        best = (float("inf"), 0.0, 0.0)
        for index, ((x1, y1), (x2, y2)) in enumerate(zip(self.points, self.points[1:])):
            dx, dy = x2 - x1, y2 - y1
            length_sq = dx * dx + dy * dy or 1e-9
            t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / length_sq))
            px, py = x1 + dx * t, y1 + dy * t
            distance = math.hypot(x - px, y - py)
            if distance < best[0]:
                heading = math.atan2(dy, dx)
                side = (x - px) * -math.sin(heading) + (y - py) * math.cos(heading)
                best = (distance, self.cumulative[index] + t * math.sqrt(length_sq), side)
        return best[1], best[2]


def _make_route(start: Tuple[float, float], heading: float, length: float, turn: float = math.pi / 2,
                first_straight: Optional[float] = None) -> List[Tuple[float, float]]:
    """Straight road, a 90-degree curve (radius 25 m), then straight again."""
    points = [start]
    x, y = start
    if first_straight is None:
        first_straight = max(20.0, 0.55 * length)
    radius = 25.0
    arc_length = abs(turn) * radius
    x, y = x + math.cos(heading) * first_straight, y + math.sin(heading) * first_straight
    points.append((x, y))
    steps = 12
    for _ in range(steps):
        heading += turn / steps
        x += math.cos(heading) * arc_length / steps
        y += math.sin(heading) * arc_length / steps
        points.append((x, y))
    remaining = max(10.0, length - first_straight - arc_length)
    points.append((x + math.cos(heading) * remaining, y + math.sin(heading) * remaining))
    return points


class MockSimulation(Simulation):
    """See module docstring."""

    def __init__(self) -> None:
        self.state = VehicleState()
        self.scene: Optional[Scene] = None
        self.route = _Route(_make_route((0.0, 0.0), 0.0, settings.DEFAULT_ROUTE_LENGTH_METERS))
        self.route_start_s = 0.0
        self.actors: List[_MockActor] = []
        self.cameras: List[CameraRequest] = []
        self.time_of_day = "day"
        self.weather = "clear"
        # Ego dynamics in route coordinates.
        self.s = 0.0
        self.d = 0.0
        self.speed = 0.0
        self.target_speed = 0.0
        self.target_d = 0.0
        self.cruise_speed = settings.AUTONOMOUS_TARGET_SPEED_MPH * MPH
        self.mode = DrivingMode.HOLDING
        self.manual_input = (0.0, 0.0, 0.0, False)
        self.ignored_obstacles: set = set()
        self.pull_over_end_s = 0.0
        self.lane_return_s: Optional[float] = None   # leave the oncoming lane here
        self._route_finishing = False            # pulling over because the route ended
        self.collisions = 0
        self.stop_reason = ""
        self._frames: Dict[str, Tuple[float, Any]] = {}
        self._frame_counter = 0

    @property
    def is_mock(self) -> bool:
        return True

    # ------------------------------------------------------------- lifecycle

    def connect(self) -> None:
        if not pygame.get_init():
            pygame.init()
        print("[mock] Using the mock simulation (no CARLA).")

    def load_scenario(self, scene: Scene, time_of_day: str, weather: str, lights: str,
                      cameras: Sequence[CameraRequest], target_speed_mph: float,
                      start_stopped: bool) -> None:
        self.scene = scene
        self.time_of_day, self.weather = time_of_day, weather
        self.cameras = list(cameras)
        self.route = _Route(_make_route((0.0, 0.0), 0.0, scene.route.length_m))
        self.route_start_s = 0.0
        self.s, self.d, self.target_d = 0.0, 0.0, 0.0
        self.cruise_speed = target_speed_mph * MPH
        start_speed = scene.ego_start_speed_mph
        if start_stopped or start_speed == 0:
            self.speed, self.target_speed, self.mode = 0.0, 0.0, DrivingMode.HOLDING
        else:
            self.speed = (start_speed if start_speed is not None else target_speed_mph) * MPH
            self.target_speed, self.mode = self.cruise_speed, DrivingMode.AUTONOMOUS
        self.actors = []
        for spec in scene.actors:
            self._spawn_from_spec(spec)
        self.ignored_obstacles = set()
        self._route_finishing = False
        self.lane_return_s = None
        self.collisions = 0
        self.stop_reason = ""
        self.state = VehicleState()
        self.state.route_length_m = self.route.length
        self._frames = {}

    def cleanup_scenario(self) -> None:
        self.actors = []
        self._frames = {}

    def close(self) -> None:
        self.cleanup_scenario()

    # -------------------------------------------------------------- per frame

    def tick(self, dt: float) -> VehicleState:
        dt = max(0.0, min(dt, 0.2))
        for actor in self.actors:
            actor.update(dt)
        self._drive(dt)
        self._check_collisions()
        self._update_state()
        self._render_cameras()
        return self.state

    def camera_frames(self) -> Dict[str, Tuple[float, Any]]:
        return dict(self._frames)

    # -------------------------------------------------------------- commands

    def resume_autonomy(self) -> None:
        # The operator has looked: obstacles the AV is waiting at are cleared.
        for actor in self._obstacles_ahead(settings.RESUME_CLEARS_OBSTACLES_WITHIN_METERS):
            if actor.kind != "walker":
                self.ignored_obstacles.add(id(actor))
        self.target_d = round(self.d / LANE) * LANE if self.mode == DrivingMode.MANUAL else self.target_d
        if self.mode == DrivingMode.PULLED_OVER or self.mode == DrivingMode.ROUTE_COMPLETE:
            self.target_d = 0.0
        self.mode = DrivingMode.AUTONOMOUS
        self.target_speed = self.cruise_speed
        self.stop_reason = ""

    def soft_stop(self, reason: str = "") -> None:
        self.mode = DrivingMode.STOPPING
        self.target_speed = 0.0
        self.stop_reason = reason

    def emergency_stop(self) -> None:
        self.mode = DrivingMode.EMERGENCY_STOP
        self.target_speed = 0.0

    def pull_over(self) -> None:
        self.mode = DrivingMode.PULLING_OVER
        self.target_speed = min(self.cruise_speed, settings.PULL_OVER_SPEED_MPH * MPH)
        self.target_d = 1.5 * LANE - CAR_HALF_WIDTH - settings.PULL_OVER_EDGE_MARGIN_METERS
        self.pull_over_end_s = self.s + settings.PULL_OVER_DISTANCE_METERS

    def change_lane(self, direction: str) -> bool:
        current_lane = int(round(self.target_d / LANE))
        new_lane = current_lane + (1 if direction == "right" else -1)
        if new_lane not in LANE_OFFSETS:
            return False
        self.target_d = new_lane * LANE
        # Same as the CARLA backend: do not stay on the wrong side of the road.
        self.lane_return_s = self.s + 35.0 if new_lane < 0 else None
        return True

    def reroute(self, alternate_index: int) -> bool:
        if self.scene is None or alternate_index >= max(1, len(self.scene.alternate_routes)):
            return False
        # Branch off at the current position: left turn for the first
        # alternate, right turn for the second, and so on.
        x, y, heading = self.route.pose(self.s)
        turn = -math.pi / 2 if alternate_index % 2 == 0 else math.pi / 2
        length = (self.scene.alternate_routes[alternate_index].length_m
                  if self.scene.alternate_routes else settings.DEFAULT_ROUTE_LENGTH_METERS)
        kept = [p for p, c in zip(self.route.points, self.route.cumulative) if c < self.s] + [(x, y)]
        branch = _make_route((x, y), heading, length, turn, first_straight=3.0)[1:]
        old_actors = [(a, self.route.point(a.s, a.d)) for a in self.actors]
        self.route = _Route(kept + branch)
        for actor, (ax, ay) in old_actors:   # keep actors where they are on the map
            actor.s, actor.d = self.route.project(ax, ay)
        self.state.route_length_m = self.route.length
        return True

    def set_manual_control(self, enabled: bool) -> None:
        if enabled:
            self.mode = DrivingMode.MANUAL
            self.manual_input = (0.0, 0.0, settings.MANUAL_COAST_BRAKE, False)
        elif self.mode == DrivingMode.MANUAL:
            self.mode = DrivingMode.HOLDING
            self.target_speed = 0.0

    def apply_manual_input(self, throttle: float, steer: float, brake: float, reverse: bool) -> None:
        self.manual_input = (throttle, steer, brake, reverse)

    def set_hazard_lights(self, on: bool) -> None:
        self.state.hazard_lights_on = on

    def set_target_speed(self, mph: float) -> None:
        self.cruise_speed = mph * MPH
        if self.mode == DrivingMode.AUTONOMOUS:
            self.target_speed = self.cruise_speed

    # ---------------------------------------------------------- world access

    def spawn_actor(self, blueprint: str, at: Placement, name: str = "") -> Optional[Any]:
        s, d = self._placement_to_sd(at)
        actor = _MockActor(blueprint, s, d, name)
        self.actors.append(actor)
        return actor

    def spawn_walker(self, at: Placement, walk_to: Placement, speed_mps: float = 1.4,
                     blueprint: str = "") -> Optional[Any]:
        actor = self.spawn_actor(blueprint or "walker.pedestrian.0001", at)
        actor.target = self._placement_to_sd(walk_to)
        actor.speed = speed_mps
        return actor

    def destroy_actor(self, handle: Any) -> None:
        if handle in self.actors:
            self.actors.remove(handle)

    def distance_to_route_point(self, route_m: float) -> float:
        return route_m - self.s

    def project_to_route(self, x: float, y: float) -> float:
        return self.route.project(x, y)[0]

    def road_polylines(self) -> List[List[Tuple[float, float]]]:
        roads = [list(self.route.points)]
        # A few cross streets so the map looks like a map.
        for s in range(60, int(self.route.length), 90):
            x, y, heading = self.route.pose(float(s))
            nx, ny = -math.sin(heading), math.cos(heading)
            roads.append([(x - nx * 80, y - ny * 80), (x + nx * 80, y + ny * 80)])
        return roads

    def route_polyline(self) -> List[Tuple[float, float]]:
        start = self.route_start_s
        return [p for p, c in zip(self.route.points, self.route.cumulative) if c >= start] or self.route.points

    def hazard_points(self) -> List[Tuple[float, float]]:
        return [self.route.point(a.s, a.d) for a in self.actors if a.kind != "walker"]

    # ------------------------------------------------------------- internals

    def _placement_to_sd(self, at: Placement) -> Tuple[float, float]:
        if at.is_route_relative:
            return at.route_m, at.lane * LANE + at.lateral_m
        return self.route.project(at.x, at.y)

    def _spawn_from_spec(self, spec: ActorSpec) -> None:
        if spec.is_walker and spec.walk_to is not None:
            self.spawn_walker(spec.at, spec.walk_to, spec.walk_speed, spec.blueprint)
        else:
            self.spawn_actor(spec.blueprint, spec.at, spec.name)

    def _obstacles_ahead(self, reach: Optional[float] = None) -> List[_MockActor]:
        """Actors in the ego's path within the detection distance (or ``reach``)."""
        if reach is None:
            reach = settings.OBSTACLE_DETECTION_DISTANCE_METERS + self.speed * 1.2
        found = []
        for actor in self.actors:
            if id(actor) in self.ignored_obstacles:
                continue
            ahead = actor.s - self.s
            if CAR_HALF_LENGTH < ahead < reach and abs(actor.d - self.d) < settings.OBSTACLE_LANE_HALF_WIDTH_METERS:
                found.append(actor)
        return found

    def _drive(self, dt: float) -> None:
        mode = self.mode
        if mode == DrivingMode.MANUAL:
            self._drive_manual(dt)
            return

        if mode == DrivingMode.AUTONOMOUS:
            if self.lane_return_s is not None and self.s >= self.lane_return_s:
                self.lane_return_s = None
                self.target_d = 0.0
            blocking = self._obstacles_ahead()
            walkers = [a for a in blocking if a.kind == "walker"]
            others = [a for a in blocking if a.kind != "walker"]
            if others:
                # Like a real AV: stop and ask for help.
                self.soft_stop("Obstacle detected ahead: {}".format(others[0].name))
            elif walkers:
                self.target_speed = 0.0      # yield, continue by itself when clear
            else:
                self.target_speed = self.cruise_speed
            if self.s >= self.route.length - 1.0:
                self.pull_over()
                self.pull_over_end_s = self.s + 8.0
                self._route_finishing = True

        if mode == DrivingMode.PULLING_OVER and self.s >= self.pull_over_end_s:
            self.target_speed = 0.0

        deceleration = (9.0 if mode == DrivingMode.EMERGENCY_STOP else settings.MOCK_BRAKE_DECELERATION_MPS2
                        if mode in (DrivingMode.STOPPING, DrivingMode.HOLDING) else 4.0)
        if self.speed < self.target_speed:
            self.speed = min(self.target_speed, self.speed + settings.MOCK_ACCELERATION_MPS2 * dt)
        else:
            self.speed = max(self.target_speed, self.speed - deceleration * dt)
        self.s += self.speed * dt
        if self.speed > 0.3:
            lateral_rate = 1.2 if mode != DrivingMode.PULLING_OVER else 0.8
            step = max(-lateral_rate * dt, min(lateral_rate * dt, self.target_d - self.d))
            self.d += step

        stopped = self.speed < settings.STOPPED_SPEED_MPH * MPH
        if stopped and mode == DrivingMode.STOPPING:
            self.mode = DrivingMode.HOLDING
        if stopped and mode == DrivingMode.PULLING_OVER and self.target_speed == 0.0:
            if self._route_finishing:
                self.mode = DrivingMode.ROUTE_COMPLETE
                self._route_finishing = False
            else:
                self.mode = DrivingMode.PULLED_OVER
            self.state.hazard_lights_on = True

    def _drive_manual(self, dt: float) -> None:
        throttle, steer, brake, reverse = self.manual_input
        direction = -1.0 if reverse else 1.0
        acceleration = throttle * 6.0 * direction
        if abs(self.speed) * 2.23694 > settings.MANUAL_SPEED_LIMIT_MPH:
            acceleration = 0.0
        friction = (brake * 8.0 + 0.3) * (1 if self.speed > 0 else -1 if self.speed < 0 else 0)
        new_speed = self.speed + (acceleration - friction) * dt
        if self.speed != 0 and (new_speed * self.speed < 0) and throttle == 0:
            new_speed = 0.0
        self.speed = new_speed
        self.s += self.speed * dt
        # Steering moves the car sideways in proportion to forward motion.
        self.d += steer * 0.35 * self.speed * dt * 1.0
        self.target_d = self.d

    def _check_collisions(self) -> None:
        for actor in self.actors:
            if actor.hit or actor.is_flat:
                continue
            length, width, _ = actor.size
            if abs(actor.s - self.s) < CAR_HALF_LENGTH + length / 2 and abs(actor.d - self.d) < CAR_HALF_WIDTH + width / 2:
                actor.hit = True
                self.collisions += 1
                self.state.last_collision_with = actor.blueprint
                if self.mode != DrivingMode.MANUAL:
                    self.speed = 0.0
                    self.soft_stop("")     # the session shows "Collision detected"

    def _update_state(self) -> None:
        state = self.state
        x, y = self.route.point(self.s, self.d)
        _, _, heading = self.route.pose(self.s)
        state.x, state.y = x, y
        state.yaw_degrees = math.degrees(heading)
        state.speed_mps = abs(self.speed)
        state.mode = self.mode
        state.route_traveled_m = max(0.0, self.s - self.route_start_s)
        state.route_length_m = self.route.length - self.route_start_s
        state.collision_count = self.collisions
        state.stop_reason = self.stop_reason if self.mode in (DrivingMode.STOPPING, DrivingMode.HOLDING) else ""
        state.lane_id = int(round(self.d / LANE))
        latitude0, longitude0 = settings.MOCK_GEO_ORIGIN
        state.latitude = latitude0 - y / 111320.0
        state.longitude = longitude0 + x / (111320.0 * math.cos(math.radians(latitude0)))

    # ------------------------------------------------------------- rendering

    def _render_cameras(self) -> None:
        now = time.monotonic()
        for camera in self.cameras:
            surface = pygame.Surface((camera.width, camera.height))
            if camera.name == "top_down":
                self._draw_top_down(surface)
            else:
                mount = settings.CAMERA_MOUNTS.get(camera.name, {})
                self._draw_perspective(surface, mount)
            array = pygame.surfarray.array3d(surface).swapaxes(0, 1)
            self._frames[camera.name] = (now, array)

    def _sky_and_ground(self) -> Tuple[Tuple[int, int, int], Tuple[int, int, int]]:
        sky = {"day": (130, 180, 235), "sunset": (235, 150, 110), "night": (15, 20, 40)}.get(self.time_of_day, (130, 180, 235))
        if self.weather in ("overcast", "rain", "storm", "heavy_fog"):
            sky = tuple(int(c * 0.45 + 120 * 0.55) for c in sky)
        ground = (70, 95, 60) if self.time_of_day != "night" else (20, 28, 22)
        return sky, ground

    def _relative(self, s: float, d: float) -> Tuple[float, float]:
        """Position (forward, right) of route point (s, d) in the ego frame."""
        x, y = self.route.point(s, d)
        ex, ey = self.route.point(self.s, self.d)
        _, _, heading = self.route.pose(self.s)
        dx, dy = x - ex, y - ey
        forward = dx * math.cos(heading) + dy * math.sin(heading)
        right = -dx * math.sin(heading) + dy * math.cos(heading)
        return forward, right

    def _draw_perspective(self, surface: pygame.Surface, mount: Dict[str, float]) -> None:
        width, height = surface.get_size()
        sky, ground = self._sky_and_ground()
        horizon = int(height * 0.45)
        surface.fill(sky)
        pygame.draw.rect(surface, ground, (0, horizon, width, height - horizon))
        yaw = math.radians(mount.get("yaw", 0.0))
        camera_height = mount.get("z", 1.5)
        focal = (width / 2) / math.tan(math.radians(mount.get("fov", 90.0)) / 2)
        offset_forward, offset_right = mount.get("x", 0.0), mount.get("y", 0.0)

        def project(forward: float, right: float, up: float = 0.0) -> Optional[Tuple[float, float, float]]:
            forward -= offset_forward
            right -= offset_right
            depth = forward * math.cos(yaw) + right * math.sin(yaw)
            across = -forward * math.sin(yaw) + right * math.cos(yaw)
            if depth < 0.5:
                return None
            return (width / 2 + focal * across / depth,
                    horizon + focal * (camera_height - up) / depth, depth)

        # Road surface and lane markings, sampled along the route.
        road_color = (65, 65, 72) if self.time_of_day != "night" else (35, 35, 40)
        edge_left, edge_right = -1.5 * LANE, 1.5 * LANE
        step = 2.0
        s_values = [self.s - 60 + i * step for i in range(int(180 / step))]
        for s1, s2 in zip(s_values, s_values[1:]):
            corners = [project(*self._relative(s1, edge_left)), project(*self._relative(s2, edge_left)),
                       project(*self._relative(s2, edge_right)), project(*self._relative(s1, edge_right))]
            if all(corners):
                pygame.draw.polygon(surface, road_color, [(c[0], c[1]) for c in corners])
        for boundary, color, dashed in ((-0.5 * LANE, (240, 200, 60), False), (0.5 * LANE, (235, 235, 235), True),
                                        (edge_left, (235, 235, 235), False), (edge_right, (235, 235, 235), False)):
            for index, (s1, s2) in enumerate(zip(s_values, s_values[1:])):
                if dashed and index % 3 == 0:
                    continue
                a, b = project(*self._relative(s1, boundary)), project(*self._relative(s2, boundary))
                if a and b:
                    pygame.draw.line(surface, color, a[:2], b[:2], max(1, int(width / 400)))

        # Actors and roadside buildings, far to near.
        drawn = []
        for actor in self.actors + self._scenery_near():
            forward, right = self._relative(actor.s, actor.d)
            length, actor_width, actor_height = actor.size
            base = project(forward, right)
            top = project(forward, right, actor_height)
            if not base or not top:
                continue
            half = focal * actor_width / 2 / base[2]
            drawn.append((base[2], actor, base, top, half))
        for depth, actor, base, top, half in sorted(drawn, key=lambda item: -item[0]):
            rect = pygame.Rect(int(base[0] - half), int(top[1]), max(2, int(2 * half)), max(2, int(base[1] - top[1])))
            pygame.draw.rect(surface, actor.color(), rect)
            if "police" in actor.blueprint or "ambulance" in actor.blueprint:
                pygame.draw.rect(surface, (40, 80, 255), (rect.x, rect.y, rect.w // 2, max(2, rect.h // 6)))
                pygame.draw.rect(surface, (255, 40, 40), (rect.centerx, rect.y, rect.w // 2, max(2, rect.h // 6)))
            if "barrier" in actor.blueprint:
                stripe = max(2, rect.w // 6)
                for x in range(rect.x, rect.right, stripe * 2):
                    pygame.draw.rect(surface, (245, 245, 245), (x, rect.y + rect.h // 3, stripe, rect.h // 3))

        self._weather_overlay(surface)

    def _scenery_near(self) -> List[_MockActor]:
        """Simple buildings along both sides of the road (drawing only, no collisions)."""
        buildings = []
        first = int((self.s - 60) // 18)
        for index in range(first, first + 14):
            for side in (-1, 1):
                building = _MockActor("scenery.building", index * 18.0 + 9 * (side > 0), side * (2.0 * LANE + 9.0))
                buildings.append(building)
        return buildings

    def _draw_top_down(self, surface: pygame.Surface) -> None:
        width, height = surface.get_size()
        _, ground = self._sky_and_ground()
        surface.fill(ground)
        mount = settings.CAMERA_MOUNTS["top_down"]
        meters_across = 2 * mount["z"] * math.tan(math.radians(mount["fov"]) / 2)
        scale = width / meters_across

        def to_screen(forward: float, right: float) -> Tuple[float, float]:
            return width / 2 + right * scale, height / 2 - forward * scale

        road_color = (65, 65, 72)
        s_values = [self.s - 40 + i for i in range(80)]
        for s1, s2 in zip(s_values, s_values[1:]):
            corners = [to_screen(*self._relative(s1, -1.5 * LANE)), to_screen(*self._relative(s2, -1.5 * LANE)),
                       to_screen(*self._relative(s2, 1.5 * LANE)), to_screen(*self._relative(s1, 1.5 * LANE))]
            pygame.draw.polygon(surface, road_color, corners)
        for boundary, color in ((-0.5 * LANE, (240, 200, 60)), (0.5 * LANE, (235, 235, 235))):
            points = [to_screen(*self._relative(s, boundary)) for s in s_values[::2]]
            pygame.draw.lines(surface, color, False, points, 2)
        for actor in self.actors:
            forward, right = self._relative(actor.s, actor.d)
            length, actor_width, _ = actor.size
            cx, cy = to_screen(forward, right)
            rect = pygame.Rect(0, 0, max(3, int(actor_width * scale)), max(3, int(length * scale)))
            rect.center = (int(cx), int(cy))
            pygame.draw.rect(surface, actor.color(), rect)
        ego = pygame.Rect(0, 0, int(2 * CAR_HALF_WIDTH * scale), int(2 * CAR_HALF_LENGTH * scale))
        ego.center = (width // 2, height // 2)
        pygame.draw.rect(surface, (235, 235, 240), ego, border_radius=4)
        pygame.draw.rect(surface, (90, 140, 220), (ego.x + 2, ego.y + ego.h // 5, ego.w - 4, ego.h // 4))
        self._weather_overlay(surface)

    def _weather_overlay(self, surface: pygame.Surface) -> None:
        width, height = surface.get_size()
        if self.weather in ("heavy_fog", "dust_storm"):
            fog = pygame.Surface((width, height))
            fog.fill((200, 200, 200) if self.weather == "heavy_fog" else (200, 170, 110))
            fog.set_alpha(150)
            surface.blit(fog, (0, 0))
        if self.weather in ("rain", "storm"):
            self._frame_counter += 1
            for index in range(60 if self.weather == "rain" else 120):
                x = (index * 97 + self._frame_counter * 13) % width
                y = (index * 53 + self._frame_counter * 29) % height
                pygame.draw.line(surface, (190, 200, 220), (x, y), (x - 3, y + 12), 1)
