# Scene files

A scene is the **physical world setup** of a scenario: the map, where the
car starts, where it drives, and what objects are placed. Scenes are JSON
files in `scenarios/scenes/`. A scenario row points to one with its
`scene` column. Several scenarios can share a scene. For example,
`open_road.json` is used for the maintenance, passenger and weather tasks.

You can write scenes by hand or make them with the scene editor
(`tools/scene_editor.py`, which needs a running CARLA server).

## Example

```json
{
  "description": "Construction barriers close the ego lane; cones taper into it.",
  "map": "Town10HD_Opt",
  "ego": {"spawn_point": 1, "blueprint": "vehicle.tesla.model3"},
  "route": {"length_m": 300},
  "alternate_routes": [{"label": "Detour via next left turn", "length_m": 250}],
  "actors": [
    {"blueprint": "static.prop.constructioncone", "at": {"route_m": 58, "lateral_m": 1.3}},
    {"blueprint": "static.prop.streetbarrier", "at": {"route_m": 70, "yaw": 90}},
    {"blueprint": "vehicle.dodge.charger_police_2020", "at": {"route_m": 80, "lane": -1},
     "lights": ["Special1", "Special2"]}
  ],
  "triggers": {"before_barrier": {"route_m": 40}},
  "traffic": {"vehicles": 10, "walkers": 10}
}
```

## Fields

| Field              | Meaning                                                                                  |
|--------------------|------------------------------------------------------------------------------------------|
| `description`      | Free text.                                                                               |
| `map`              | Any CARLA map name (`Town01` ... `Town10HD_Opt`). Omit it to keep the current map. Maps the server does not have are reported at startup, and their scenarios are skipped. |
| `ego.blueprint`    | Car model (default in settings).                                                         |
| `ego.spawn_point`  | Index into the map's spawn points, **or**                                                 |
| `ego.start`        | An absolute position `{"x":..,"y":..,"z":..,"yaw":..}`.                                  |
| `ego.start_speed_mph` | Speed at the start (0 = standing still).                                              |
| `route.destination` | `{"x":..,"y":..}`: the route is planned there with CARLA's route planner, **or**        |
| `route.destination_spawn_point` | A spawn-point index as the destination, **or**                              |
| `route.length_m`   | Follow the road for this many metres, going straight at junctions.                       |
| `alternate_routes` | Routes offered by "Reroute" and the `reroute` outcome. Same fields as `route`, plus `label`. A route given by length turns left (1st alternate) or right (2nd) at the next junction. |
| `actors`           | Objects to place (below).                                                                 |
| `triggers`         | Named positions used by `task_trigger` and task scripts.                                  |
| `traffic`          | Number of background vehicles and pedestrians (driven by CARLA's traffic AI).            |

## Positions

Every `at`, `walk_to`, trigger and destination is a position, written in
one of two ways.

**Relative to the ego route.** This is portable and easy to write by hand:

| Key         | Meaning                                                      |
|-------------|--------------------------------------------------------------|
| `route_m`   | metres along the route from the car's start                   |
| `lane`      | 0 = the route's lane, 1 = one lane to the right, -1 = left   |
| `lateral_m` | extra sideways shift in metres, positive = right             |
| `z_m`       | height above the road                                        |
| `yaw`       | rotation relative to the road direction (degrees)            |

**Absolute map coordinates** (as shown by the scene editor):
`{"x": -47.0, "y": 21.3, "z": 0.5, "yaw": 90}`.

## Actors

| Key                | Meaning                                                                    |
|--------------------|----------------------------------------------------------------------------|
| `blueprint`        | CARLA blueprint id, e.g. `static.prop.constructioncone`, `vehicle.dodge.charger_police_2020`, `walker.pedestrian.0001` |
| `at`               | position (above)                                                           |
| `name`             | optional label (shown in "Obstacle detected" alerts)                        |
| `lights`           | vehicle lights to switch on, e.g. `["Special1", "Special2"]` (emergency lights) |
| `walk_to`          | pedestrians: walk in a straight line to this position                      |
| `walk_speed`       | pedestrians: metres per second (default 1.4)                               |
| `simulate_physics` | props: `true` lets them be knocked over (default false)                    |
| `attributes`       | extra blueprint attributes, e.g. `{"color": "255,0,0"}`                     |

A blueprint that does not exist in your CARLA version is skipped with a
warning. Placed vehicles have their hand brake on.

## How the automated car reacts to scene objects

* **Vehicles and red lights**: CARLA's BasicAgent slows and stops.
* **Props and parked vehicles in the lane**: the car stops, holds, and
  shows "Obstacle detected ahead: <name>" in the Alerts panel. It waits for
  the operator. When the operator lets it continue, objects up to 30 m ahead
  count as checked (setting `RESUME_CLEARS_OBSTACLES_WITHIN_METERS`).
* **Pedestrians in the lane**: the car waits and continues by itself once
  the lane is clear.

## Scene editor

```bash
python tools/scene_editor.py --map Town10HD_Opt
python tools/scene_editor.py --scene scenarios/scenes/police_blockage.json
```

The view is top-down over the running CARLA world. The keys are shown on
screen: 1-9 choose an object, left-click places it, right-click selects,
arrow keys nudge, `,` and `.` rotate, Delete removes, G sets the ego start,
H sets the destination, T adds a trigger, Ctrl+S saves. The editor saves
absolute coordinates. Change any position to a route-relative one by hand if
you want the scene to adapt to a different start point.
