"""
fake_carla.py -- a tiny stand-in for the ``carla`` module, for tests only.

It models a straight east-west road (+x) with four lanes at y = -3.5
(oncoming), 0 (ego lane), 3.5 (right lane) and 7 (shoulder), and crude
vehicle physics.  It implements only what simulation_carla.py uses, so the
CARLA backend's logic can be tested without a CARLA server.  It is NOT a
model of CARLA's behaviour in any detail.
"""

import enum
import fnmatch
import math

DT = 1.0 / 30.0


class Vector3D:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def length(self):
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)


class Location(Vector3D):
    def distance(self, other):
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2)

    def __add__(self, other):
        return Location(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Location(self.x - other.x, self.y - other.y, self.z - other.z)


class Rotation:
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
        self.pitch, self.yaw, self.roll = pitch, yaw, roll


class Transform:
    def __init__(self, location=None, rotation=None):
        self.location = location or Location()
        self.rotation = rotation or Rotation()

    def get_forward_vector(self):
        yaw = math.radians(self.rotation.yaw)
        return Vector3D(math.cos(yaw), math.sin(yaw), 0)

    def get_right_vector(self):
        yaw = math.radians(self.rotation.yaw)
        return Vector3D(-math.sin(yaw), math.cos(yaw), 0)


class VehicleControl:
    def __init__(self, throttle=0.0, steer=0.0, brake=0.0, hand_brake=False, reverse=False):
        self.throttle, self.steer, self.brake = throttle, steer, brake
        self.hand_brake, self.reverse = hand_brake, reverse


class WalkerControl:
    def __init__(self, direction=None, speed=0.0, jump=False):
        self.direction, self.speed = direction or Vector3D(), speed


class VehicleLightState(enum.IntFlag):
    NONE = 0
    Position = 1
    LowBeam = 2
    HighBeam = 4
    Brake = 8
    RightBlinker = 16
    LeftBlinker = 32
    Reverse = 64
    Fog = 128
    Interior = 256
    Special1 = 512
    Special2 = 1024
    All = 2047


class LaneType(enum.IntFlag):
    NONE = 0
    Driving = 2
    Shoulder = 8
    Parking = 64
    Sidewalk = 128


class LightGroup(enum.Enum):
    NONE = 0
    Street = 2


class WeatherParameters:
    def __init__(self):
        for name in ("cloudiness", "precipitation", "precipitation_deposits", "wind_intensity", "fog_density",
                     "fog_distance", "fog_falloff", "wetness", "dust_storm", "sun_altitude_angle", "sun_azimuth_angle"):
            setattr(self, name, 0.0)


class GeoLocation:
    def __init__(self, latitude, longitude):
        self.latitude, self.longitude = latitude, longitude


# ----------------------------------------------------------------- the road

LANES = {0: (0.0, LaneType.Driving), 1: (3.5, LaneType.Driving), 2: (7.0, LaneType.Shoulder),
         -1: (-3.5, LaneType.Driving)}


class Waypoint:
    def __init__(self, x, lane=0):
        self.lane = lane
        y, self.lane_type = LANES[lane]
        self.lane_id = 1 if lane < 0 else -1          # opposite direction has the other sign
        self.lane_width = 3.5
        self.transform = Transform(Location(x, y, 0.0), Rotation(yaw=180.0 if lane < 0 else 0.0))

    def next(self, distance):
        step = -distance if self.lane < 0 else distance
        return [Waypoint(self.transform.location.x + step, self.lane)]

    def get_left_lane(self):
        lane = self.lane - 1 if self.lane > 0 else (-1 if self.lane == 0 else None)
        return Waypoint(self.transform.location.x, lane) if lane in LANES else None

    def get_right_lane(self):
        lane = self.lane + 1 if self.lane >= 0 else 0
        return Waypoint(self.transform.location.x, lane) if lane in LANES else None


class Map:
    def __init__(self, name):
        self.name = name

    def get_waypoint(self, location, project_to_road=True, lane_type=None):
        lane = min(LANES, key=lambda key: abs(LANES[key][0] - location.y))
        if LANES[lane][1] != LaneType.Driving:
            lane = 1
        return Waypoint(location.x, lane)

    def get_spawn_points(self):
        return [Transform(Location(x, 0.0, 0.5), Rotation()) for x in (0.0, 100.0, 200.0, 300.0)]

    def get_topology(self):
        return [(Waypoint(-50), Waypoint(600)), (Waypoint(600, -1), Waypoint(-50, -1))]

    def transform_to_geolocation(self, location):
        return GeoLocation(-location.y / 111320.0, location.x / 111320.0)


# --------------------------------------------------------------- actors

class AttributeValue:
    def __init__(self, value):
        self.value = value

    def as_int(self):
        return int(self.value)

    def as_str(self):
        return str(self.value)


class Blueprint:
    def __init__(self, type_id):
        self.id = type_id
        self.attributes = {}
        if type_id.startswith("vehicle."):
            self.attributes["number_of_wheels"] = "4"
            self.attributes["role_name"] = ""
        if type_id.startswith("walker."):
            self.attributes["is_invincible"] = "true"
        if type_id.startswith("sensor.camera"):
            self.attributes.update(image_size_x="800", image_size_y="600", fov="90", sensor_tick="0")

    def has_attribute(self, name):
        return name in self.attributes

    def set_attribute(self, name, value):
        if name not in self.attributes:
            raise IndexError(name)
        self.attributes[name] = value

    def get_attribute(self, name):
        return AttributeValue(self.attributes[name])


KNOWN_BLUEPRINTS = [
    "vehicle.tesla.model3", "vehicle.carlamotors.carlacola", "vehicle.dodge.charger_police_2020",
    "vehicle.lincoln.mkz_2020", "walker.pedestrian.0001", "walker.pedestrian.0012",
    "static.prop.constructioncone", "static.prop.streetbarrier", "static.prop.warningconstruction",
    "static.prop.dirtdebris01", "sensor.camera.rgb", "sensor.other.collision", "controller.ai.walker",
]


class BlueprintLibrary:
    def find(self, name):
        if name not in KNOWN_BLUEPRINTS:
            raise IndexError("blueprint '{}' not found".format(name))
        return Blueprint(name)

    def filter(self, pattern):
        return [Blueprint(name) for name in KNOWN_BLUEPRINTS if fnmatch.fnmatch(name, pattern)]


class ActorList(list):
    def filter(self, pattern):
        return ActorList(a for a in self if fnmatch.fnmatch(a.type_id, pattern))


class Actor:
    _next_id = 100

    def __init__(self, world, blueprint, transform, parent=None):
        Actor._next_id += 1
        self.id = Actor._next_id
        self.world = world
        self.type_id = blueprint.id
        self.attributes = dict(blueprint.attributes)
        self.transform = Transform(Location(transform.location.x, transform.location.y, transform.location.z),
                                   Rotation(transform.rotation.pitch, transform.rotation.yaw, transform.rotation.roll))
        self.parent = parent
        self.speed = 0.0
        self.light_state = VehicleLightState.NONE
        self.alive = True
        self.callback = None
        self.walker_control = None
        self.last_control = None

    def get_location(self):
        return self.transform.location

    def get_transform(self):
        return self.transform

    def set_transform(self, transform):
        self.transform = transform

    def get_velocity(self):
        forward = self.transform.get_forward_vector()
        return Vector3D(forward.x * self.speed, forward.y * self.speed, 0)

    def set_target_velocity(self, velocity):
        self.speed = velocity.length()

    def apply_control(self, control):
        if isinstance(control, WalkerControl):
            self.walker_control = control
            return
        self.last_control = control
        # Crude longitudinal physics plus steering as a lateral drift.
        if control.hand_brake or control.brake > 0:
            self.speed = max(0.0, self.speed - (8.0 * max(control.brake, 0.6 if control.hand_brake else 0)) * DT)
        direction = -1.0 if control.reverse else 1.0
        self.speed += control.throttle * 4.0 * DT * direction
        self.speed = max(-3.0, min(self.speed, 30.0))
        self.transform.location.x += self.speed * DT
        self.transform.location.y += control.steer * 2.0 * abs(self.speed) * DT
        self.world.advance()

    def set_light_state(self, state):
        self.light_state = VehicleLightState(int(state))

    def get_light_state(self):
        return self.light_state

    def set_simulate_physics(self, enabled):
        self.physics = enabled

    def set_autopilot(self, enabled, port=8000):
        self.autopilot = enabled

    def listen(self, callback):
        self.callback = callback

    def stop(self):
        self.callback = None

    def start(self):
        pass

    def go_to_location(self, location):
        pass

    def set_max_speed(self, speed):
        pass

    def destroy(self):
        self.alive = False
        if self in self.world.actors:
            self.world.actors.remove(self)
        return True


class Image:
    def __init__(self, width, height):
        self.width, self.height = width, height
        self.raw_data = bytes(width * height * 4)


class CollisionEvent:
    def __init__(self, other):
        self.other_actor = other


class WorldSettings:
    synchronous_mode = False
    fixed_delta_seconds = None


class LightManager:
    def get_all_lights(self, group=None):
        return []

    def turn_on(self, lights):
        pass

    def turn_off(self, lights):
        pass

    def set_intensity(self, lights, value):
        pass


class World:
    def __init__(self, map_name):
        self.map = Map(map_name)
        self.actors = ActorList()
        self.weather = WeatherParameters()
        self.settings = WorldSettings()
        self.frame = 0
        self.collided = set()

    def get_map(self):
        return self.map

    def get_settings(self):
        return self.settings

    def apply_settings(self, settings):
        self.settings = settings

    def get_weather(self):
        return self.weather

    def set_weather(self, weather):
        self.weather = weather

    def get_lightmanager(self):
        return LightManager()

    def get_blueprint_library(self):
        return BlueprintLibrary()

    def try_spawn_actor(self, blueprint, transform, attach_to=None):
        return self.spawn_actor(blueprint, transform, attach_to)

    def spawn_actor(self, blueprint, transform, attach_to=None):
        actor = Actor(self, blueprint, transform, attach_to)
        self.actors.append(actor)
        return actor

    def get_actors(self):
        return ActorList(self.actors)

    def wait_for_tick(self):
        self.advance()

    def get_random_location_from_navigation(self):
        return Location(50.0, 12.0, 0.0)

    def advance(self):
        """Move walkers, deliver camera images and collisions."""
        self.frame += 1
        hero = [a for a in self.actors if a.attributes.get("role_name") == "hero"]
        for actor in list(self.actors):
            if actor.type_id.startswith("walker.") and actor.walker_control is not None:
                control = actor.walker_control
                actor.transform.location.x += control.direction.x * control.speed * DT
                actor.transform.location.y += control.direction.y * control.speed * DT
            if actor.callback and actor.type_id == "sensor.camera.rgb" and self.frame % 2 == 0:
                actor.callback(Image(int(actor.attributes["image_size_x"]), int(actor.attributes["image_size_y"])))
        if hero:
            ego = hero[0]
            for actor in self.actors:
                if actor is ego or actor.parent is not None or actor.id in self.collided:
                    continue
                if actor.type_id.startswith(("vehicle.", "static.prop.street", "walker.")):
                    if abs(actor.transform.location.x - ego.transform.location.x) < 3.0 and \
                            abs(actor.transform.location.y - ego.transform.location.y) < 1.5:
                        self.collided.add(actor.id)
                        for sensor in self.actors:
                            if sensor.type_id == "sensor.other.collision" and sensor.callback:
                                sensor.callback(CollisionEvent(actor))


class TrafficManager:
    def set_synchronous_mode(self, enabled):
        pass

    def get_port(self):
        return 8000


class _DestroyActor:
    def __init__(self, actor):
        self.actor = actor


class command:   # noqa: N801 - mirrors carla.command
    DestroyActor = _DestroyActor


class Client:
    def __init__(self, host, port):
        self.world = World("Carla/Maps/Town10HD_Opt")

    def set_timeout(self, seconds):
        pass

    def get_server_version(self):
        return "0.9.15"

    def get_client_version(self):
        return "0.9.15"

    def get_world(self):
        return self.world

    def load_world(self, name):
        self.world = World("Carla/Maps/" + name)
        return self.world

    def get_available_maps(self):
        return ["/Game/Carla/Maps/Town10HD_Opt", "/Game/Carla/Maps/Town01"]

    def apply_batch_sync(self, commands, do_tick=False):
        for item in commands:
            item.actor.destroy()
        return []

    def get_trafficmanager(self, port=8000):
        return TrafficManager()


class FakeAgent:
    """Replaces CARLA's BasicAgent: drives straight to the end of the plan."""

    def __init__(self, vehicle, target_speed=20, opt_dict=None, map_inst=None, grp_inst=None):
        self.vehicle = vehicle
        self.target_kmh = target_speed
        self.plan = []
        self.offset = 0.0

    def set_global_plan(self, plan, stop_waypoint_creation=True, clean_queue=True):
        self.plan = list(plan)

    def set_target_speed(self, kmh):
        self.target_kmh = kmh

    def set_offset(self, offset):
        self.offset = offset

    def done(self):
        return not self.plan or self.vehicle.get_location().x >= self.plan[-1][0].transform.location.x - 1.0

    def run_step(self):
        speed = self.vehicle.speed * 3.6
        # Steer toward the plan's lane (plus offset).
        target_y = self.plan[min(5, len(self.plan) - 1)][0].transform.location.y + self.offset if self.plan else 0
        steer = max(-1.0, min(1.0, (target_y - self.vehicle.get_location().y) * 0.3))
        if speed < self.target_kmh - 1:
            return VehicleControl(throttle=0.6, steer=steer)
        if speed > self.target_kmh + 2:
            return VehicleControl(brake=0.3, steer=steer)
        return VehicleControl(throttle=0.05, steer=steer)
