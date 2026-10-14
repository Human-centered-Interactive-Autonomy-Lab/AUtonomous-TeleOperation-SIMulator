"""
scene.py -- the physical world setup of a scenario, stored as a JSON file.

A *scene* says WHERE things are: which map, where the ego vehicle starts,
where it drives to, and which props / vehicles / pedestrians are placed in
the world.  A *scenario* (see scenario.py) says everything else: weather,
vehicle status, alerts and task logic, and points to one scene file.

Positions can be written two ways (see docs/SCENES.md):

  Absolute map coordinates (as shown by the scene editor):
      {"x": -47.0, "y": 21.3, "z": 0.5, "yaw": 90}

  Relative to the ego vehicle's route -- portable and easy to write by hand:
      {"route_m": 60, "lane": 0, "lateral_m": 0.5}
        route_m    distance along the ego route from the start (metres)
        lane       0 = ego lane, 1 = next lane to the right, -1 = to the left
        lateral_m  extra sideways shift, positive = right (metres)
        z_m        height above the road (metres, default 0)
        yaw        rotation relative to the road direction (degrees)

Example scene file::

    {
      "description": "Construction barrier blocking the right lane",
      "map": "Town10HD_Opt",
      "ego": {"spawn_point": 12, "blueprint": "vehicle.tesla.model3"},
      "route": {"length_m": 300},
      "actors": [
        {"blueprint": "static.prop.streetbarrier", "at": {"route_m": 70, "yaw": 90}},
        {"blueprint": "static.prop.constructioncone", "at": {"route_m": 64, "lateral_m": -1.2}}
      ],
      "triggers": {"barrier_stop": {"route_m": 55}}
    }
"""

import json
import os
from typing import Any, Dict, List, Optional

from autosim import settings


class SceneError(ValueError):
    """Raised when a scene file is missing or malformed."""


class Placement:
    """Where to put something: absolute pose or a pose relative to the route."""

    def __init__(self, data: Dict[str, Any], context: str = "") -> None:
        if not isinstance(data, dict):
            raise SceneError("{}: position must be an object, got {!r}".format(context, data))
        self.raw = dict(data)
        self.is_route_relative = "route_m" in data
        if not self.is_route_relative and not ("x" in data and "y" in data):
            raise SceneError("{}: position needs either 'route_m' or 'x' and 'y'".format(context))
        try:
            self.route_m = float(data.get("route_m", 0.0))
            self.lane = int(data.get("lane", 0))
            self.lateral_m = float(data.get("lateral_m", 0.0))
            self.z_m = float(data.get("z_m", data.get("z", 0.0)))
            self.x = float(data.get("x", 0.0))
            self.y = float(data.get("y", 0.0))
            self.yaw = float(data.get("yaw", 0.0))
            self.pitch = float(data.get("pitch", 0.0))
            self.roll = float(data.get("roll", 0.0))
        except (TypeError, ValueError):
            raise SceneError("{}: position values must be numbers: {!r}".format(context, data))

    def to_dict(self) -> Dict[str, Any]:
        return dict(self.raw)

    def __repr__(self) -> str:
        return "Placement({})".format(self.raw)


class ActorSpec:
    """One actor to spawn (prop, parked vehicle, pedestrian...)."""

    def __init__(self, data: Dict[str, Any], index: int) -> None:
        context = "actor #{}".format(index + 1)
        if "blueprint" not in data:
            raise SceneError("{}: missing 'blueprint'".format(context))
        if "at" not in data:
            raise SceneError("{}: missing 'at' (position)".format(context))
        self.blueprint: str = str(data["blueprint"])
        self.at = Placement(data["at"], context)
        self.name: str = str(data.get("name", "{}_{}".format(self.blueprint.split(".")[-1], index + 1)))
        # Vehicle light names (carla.VehicleLightState members), e.g. ["Special1"]
        # turns on the siren lights of emergency vehicles.
        self.lights: List[str] = list(data.get("lights", []))
        # Pedestrians: walk to this position at this speed (m/s).
        self.walk_to: Optional[Placement] = (
            Placement(data["walk_to"], context + " walk_to") if "walk_to" in data else None
        )
        self.walk_speed: float = float(data.get("walk_speed", 1.4))
        # Physics off = the actor never moves even if hit (good for props).
        self.simulate_physics: bool = bool(data.get("simulate_physics", False))
        self.attributes: Dict[str, str] = {str(k): str(v) for k, v in data.get("attributes", {}).items()}

    @property
    def is_walker(self) -> bool:
        return self.blueprint.startswith("walker.")

    @property
    def is_vehicle(self) -> bool:
        return self.blueprint.startswith("vehicle.")


class RouteSpec:
    """How the ego route is defined: a destination or a length."""

    def __init__(self, data: Dict[str, Any], context: str = "route") -> None:
        data = data or {}
        self.destination: Optional[Placement] = (
            Placement(data["destination"], context + " destination") if "destination" in data else None
        )
        self.destination_spawn_point: Optional[int] = (
            int(data["destination_spawn_point"]) if "destination_spawn_point" in data else None
        )
        self.length_m: float = float(data.get("length_m", settings.DEFAULT_ROUTE_LENGTH_METERS))
        self.label: str = str(data.get("label", ""))


class Scene:
    """Parsed contents of a scene JSON file."""

    def __init__(self, data: Dict[str, Any], source: str = "<default>") -> None:
        if not isinstance(data, dict):
            raise SceneError("{}: top level must be a JSON object".format(source))
        self.source = source
        self.description: str = str(data.get("description", ""))
        # None = keep whatever map is loaded.
        self.map_name: Optional[str] = data.get("map") or None

        ego = data.get("ego", {})
        self.ego_blueprint: str = str(ego.get("blueprint", settings.DEFAULT_EGO_BLUEPRINT))
        self.ego_spawn_point: Optional[int] = int(ego["spawn_point"]) if "spawn_point" in ego else None
        self.ego_start: Optional[Placement] = (
            Placement(ego["start"], "ego start") if "start" in ego else None
        )
        # Starting speed: 0 = start stopped.
        self.ego_start_speed_mph: Optional[float] = (
            float(ego["start_speed_mph"]) if "start_speed_mph" in ego else None
        )

        self.route = RouteSpec(data.get("route", {}))
        self.alternate_routes: List[RouteSpec] = [
            RouteSpec(item, "alternate route #{}".format(i + 1))
            for i, item in enumerate(data.get("alternate_routes", []))
        ]
        self.actors: List[ActorSpec] = [ActorSpec(item, i) for i, item in enumerate(data.get("actors", []))]
        self.triggers: Dict[str, Placement] = {
            str(name): Placement(value, "trigger '{}'".format(name))
            for name, value in data.get("triggers", {}).items()
        }
        traffic = data.get("traffic", {})
        self.traffic_vehicles: int = int(traffic.get("vehicles", 0))
        self.traffic_walkers: int = int(traffic.get("walkers", 0))
        self.raw = data

    @staticmethod
    def default() -> "Scene":
        """Scene used when a scenario names no scene file."""
        return Scene({}, "<default>")

    def __repr__(self) -> str:
        return "Scene({}, map={}, actors={})".format(self.source, self.map_name, len(self.actors))


def load_scene(path: str) -> Scene:
    """Read and validate a scene JSON file."""
    if not os.path.isfile(path):
        raise SceneError("Scene file not found: {}".format(path))
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
    except ValueError as error:
        raise SceneError("{}: not valid JSON ({})".format(path, error))
    return Scene(data, path)


def save_scene(data: Dict[str, Any], path: str) -> None:
    """Write a scene dictionary to disk (used by the scene editor)."""
    Scene(data, path)  # validate before writing
    folder = os.path.dirname(os.path.abspath(path))
    if not os.path.isdir(folder):
        os.makedirs(folder)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2)
        handle.write("\n")
