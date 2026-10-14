"""
simulation.py -- what the rest of AUTOSIM may ask of the simulator.

The operator interface, the experiment runner and task scripts never call
CARLA directly.  They use a ``Simulation`` object, which has two versions:

  CarlaSimulation  (simulation_carla.py)  -- the real thing.
  MockSimulation   (simulation_mock.py)   -- a tiny 2-D stand-in so the
                                             interface can be developed and
                                             demonstrated without CARLA.

To support another simulator, write a third class with these methods.
"""

from typing import Any, Dict, List, Optional, Sequence, Tuple

from autosim.scene import Placement, Scene


class DrivingMode:
    """What is currently controlling the ego vehicle.  (Plain strings so they
    print nicely and can be written straight into the data files.)"""

    AUTONOMOUS = "autonomous"          # following the route
    STOPPING = "stopping"              # soft stop requested (task / obstacle)
    HOLDING = "holding"                # stopped and waiting
    EMERGENCY_STOP = "emergency_stop"  # E-STOP pressed
    PULLING_OVER = "pulling_over"
    PULLED_OVER = "pulled_over"
    MANUAL = "manual"                  # operator drives with the keyboard
    ROUTE_COMPLETE = "route_complete"

    # Text shown under the speed read-out.
    LABELS = {
        AUTONOMOUS: "AV mode",
        STOPPING: "Stopping",
        HOLDING: "Stopped - awaiting operator",
        EMERGENCY_STOP: "EMERGENCY STOP",
        PULLING_OVER: "Pulling over...",
        PULLED_OVER: "Pulled over",
        MANUAL: "Manual control",
        ROUTE_COMPLETE: "Route complete",
    }


class VehicleState:
    """Snapshot of the ego vehicle, refreshed every frame by ``Simulation.tick``."""

    def __init__(self) -> None:
        self.speed_mps: float = 0.0
        self.x: float = 0.0
        self.y: float = 0.0
        self.z: float = 0.0
        self.yaw_degrees: float = 0.0
        self.latitude: float = 0.0
        self.longitude: float = 0.0
        self.mode: str = DrivingMode.HOLDING
        self.route_traveled_m: float = 0.0       # distance driven along the route
        self.route_length_m: float = 0.0
        self.hazard_lights_on: bool = False
        self.collision_count: int = 0
        self.last_collision_with: str = ""       # type of the actor hit last
        self.lane_id: int = 0
        self.stop_reason: str = ""               # why the AV stopped by itself, if it did

    @property
    def speed_mph(self) -> float:
        return self.speed_mps * 2.23694

    def as_log_fields(self) -> Dict[str, Any]:
        """Fields added to every event-log row."""
        return {
            "speed_mph": round(self.speed_mph, 2),
            "x": round(self.x, 2),
            "y": round(self.y, 2),
            "mode": self.mode,
            "route_traveled_m": round(self.route_traveled_m, 1),
        }


class CameraRequest:
    """Which camera to create and how big its images should be."""

    def __init__(self, name: str, width: int, height: int) -> None:
        self.name = name
        self.width = max(16, int(width))
        self.height = max(16, int(height))

    def __repr__(self) -> str:
        return "CameraRequest({}, {}x{})".format(self.name, self.width, self.height)


class Simulation:
    """Interface every simulation backend implements.

    Coordinates are map metres.  Rotation is degrees.  "route_m" means
    distance along the ego vehicle's route from its starting point.
    """

    # ----------------------------------------------------------- lifecycle

    def connect(self) -> None:
        """Connect to the simulator.  Raise RuntimeError with a helpful message on failure."""
        raise NotImplementedError

    def load_scenario(self, scene: Scene, time_of_day: str, weather: str, lights: str,
                      cameras: Sequence[CameraRequest], target_speed_mph: float,
                      start_stopped: bool) -> None:
        """Build the world for one scenario: map, environment, actors, ego, cameras."""
        raise NotImplementedError

    def cleanup_scenario(self) -> None:
        """Destroy everything ``load_scenario`` (and task scripts) spawned."""
        raise NotImplementedError

    def close(self) -> None:
        """Clean up and disconnect."""
        raise NotImplementedError

    # --------------------------------------------------------------- per frame

    def tick(self, dt: float) -> VehicleState:
        """Advance driving logic by one GUI frame and return the vehicle state."""
        raise NotImplementedError

    def camera_frames(self) -> Dict[str, Tuple[float, Any]]:
        """Newest frame per camera: name -> (capture time, HxWx3 uint8 numpy array).

        Capture times use the same clock as ``time.monotonic()``.
        """
        raise NotImplementedError

    # --------------------------------------------------------- vehicle commands
    # Called by the interface (after the simulated input delay) and by tasks.

    def resume_autonomy(self) -> None:
        """Hand control back to the automated driver, continuing the route."""
        raise NotImplementedError

    def soft_stop(self, reason: str = "") -> None:
        """Brake gently to a stop and hold (used by tasks: 'the AV asks for help')."""
        raise NotImplementedError

    def emergency_stop(self) -> None:
        """Full brake immediately; hold until resumed or manual control."""
        raise NotImplementedError

    def pull_over(self) -> None:
        """Slow down, move to the road edge, stop, hazard lights on."""
        raise NotImplementedError

    def change_lane(self, direction: str) -> bool:
        """'left' or 'right'.  Returns False if there is no lane that way."""
        raise NotImplementedError

    def reroute(self, alternate_index: int) -> bool:
        """Follow the scene's alternate route (0-based).  False if it does not exist."""
        raise NotImplementedError

    def set_manual_control(self, enabled: bool) -> None:
        """Enter/leave manual (keyboard) driving."""
        raise NotImplementedError

    def apply_manual_input(self, throttle: float, steer: float, brake: float, reverse: bool) -> None:
        """One frame of manual driving input (already limited by the caller)."""
        raise NotImplementedError

    def set_hazard_lights(self, on: bool) -> None:
        raise NotImplementedError

    def set_target_speed(self, mph: float) -> None:
        raise NotImplementedError

    # ---------------------------------------------------- world (for tasks)

    def spawn_actor(self, blueprint: str, at: Placement, name: str = "") -> Optional[Any]:
        """Spawn an extra actor during a scenario.  Returns a handle or None."""
        raise NotImplementedError

    def spawn_walker(self, at: Placement, walk_to: Placement, speed_mps: float = 1.4,
                     blueprint: str = "") -> Optional[Any]:
        """Spawn a pedestrian that walks from ``at`` to ``walk_to``."""
        raise NotImplementedError

    def destroy_actor(self, handle: Any) -> None:
        raise NotImplementedError

    def distance_to_route_point(self, route_m: float) -> float:
        """Distance (metres along the route) from the ego to ``route_m`` (negative = passed)."""
        raise NotImplementedError

    def trigger_route_m(self, scene: Scene, trigger_name: str) -> Optional[float]:
        """Route distance of a named scene trigger (None if unknown)."""
        trigger = scene.triggers.get(trigger_name)
        if trigger is None:
            return None
        if trigger.is_route_relative:
            return trigger.route_m
        return self.project_to_route(trigger.x, trigger.y)

    def project_to_route(self, x: float, y: float) -> float:
        """Route distance of the route point closest to (x, y)."""
        raise NotImplementedError

    # -------------------------------------------------------------- the map

    def road_polylines(self) -> List[List[Tuple[float, float]]]:
        """Road centre lines for the map panel (computed once per map)."""
        raise NotImplementedError

    def route_polyline(self) -> List[Tuple[float, float]]:
        """The ego vehicle's planned route as (x, y) points."""
        raise NotImplementedError

    def hazard_points(self) -> List[Tuple[float, float]]:
        """Positions of placed props/vehicles, drawn as markers on the map."""
        return []

    @property
    def is_mock(self) -> bool:
        return False
