"""
scene_editor.py -- build scene JSON files by clicking in a top-down CARLA view.

Requires a running CARLA server.  Usage:

    python tools/scene_editor.py --map Town10HD_Opt
    python tools/scene_editor.py --scene scenarios/scenes/police_blockage.json

Controls (also shown on screen)
-------------------------------
  W A S D         move the camera            mouse wheel / Q E   zoom out / in
  1 ... 9         choose what to place (see PALETTE below)
  left click      place the chosen object    right click         select nearest object
  arrow keys      nudge the selection        , .                 rotate selection -15 / +15 degrees
  Delete          remove the selection
  G               ego vehicle start here      H                   route destination here
  T               add a named trigger point here
  Ctrl+S          save the scene              Esc                 quit

Positions are saved as absolute map coordinates.  You can later replace any
position by a route-relative one ({"route_m": ...}) by hand -- see docs/SCENES.md.
"""

import argparse
import json
import math
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy  # noqa: E402
import pygame  # noqa: E402

from autosim import settings  # noqa: E402
from autosim.scene import save_scene  # noqa: E402

try:
    import carla
except ImportError:
    carla = None

# ----------------------------------------------------------------------------
# Settings for this tool
# ----------------------------------------------------------------------------

WINDOW_SIZE: Tuple[int, int] = (1200, 900)
CAMERA_FIELD_OF_VIEW: float = 90.0
CAMERA_START_HEIGHT: float = 40.0
CAMERA_MIN_HEIGHT: float = 8.0
CAMERA_MAX_HEIGHT: float = 200.0
CAMERA_PAN_SPEED: float = 0.8          # fraction of the visible width per second
NUDGE_METERS: float = 0.25
ROTATE_DEGREES: float = 15.0

# Keys 1-9: what the left mouse button places.
PALETTE: List[Tuple[str, str]] = [
    ("Traffic cone", "static.prop.constructioncone"),
    ("Street barrier", "static.prop.streetbarrier"),
    ("Construction sign", "static.prop.warningconstruction"),
    ("Debris", "static.prop.dirtdebris01"),
    ("Cardboard box", "static.prop.creasedbox03"),
    ("Police car", "vehicle.dodge.charger_police_2020"),
    ("Parked van", "vehicle.carlamotors.carlacola"),
    ("Parked car", "vehicle.lincoln.mkz_2020"),
    ("Pedestrian", "walker.pedestrian.0001"),
]

MARKER_COLORS = {"ego": (40, 220, 90), "destination": (60, 160, 255), "trigger": (255, 200, 40),
                 "selected": (255, 60, 60), "actor": (255, 255, 255)}


class SceneEditor:
    def __init__(self, client, map_name: Optional[str], scene_path: Optional[str]) -> None:
        self.client = client
        self.scene_path = scene_path or ""
        self.data: Dict[str, Any] = {"description": "", "actors": [], "triggers": {}}
        if scene_path and os.path.isfile(scene_path):
            with open(scene_path, "r", encoding="utf-8") as handle:
                self.data = json.load(handle)
            map_name = map_name or self.data.get("map")
        self.world = client.get_world()
        if map_name and self.world.get_map().name.split("/")[-1] != map_name:
            print("Loading map {}...".format(map_name))
            self.world = client.load_world(map_name)
        self.map = self.world.get_map()
        self.data["map"] = self.map.name.split("/")[-1]
        self.library = self.world.get_blueprint_library()
        self.spawned: Dict[int, Any] = {}          # index in data["actors"] -> carla actor
        self.selected: Optional[int] = None
        self.palette_index = 0
        self.camera_x, self.camera_y, self.camera_height = self._initial_camera()
        self.camera = None
        self.latest_image = None
        self.message = "Ready."
        for index in range(len(self.data.get("actors", []))):
            self._spawn(index)

    # ------------------------------------------------------------ camera

    def _initial_camera(self) -> Tuple[float, float, float]:
        start = self.data.get("ego", {}).get("start")
        if start:
            return float(start["x"]), float(start["y"]), CAMERA_START_HEIGHT
        point = self.map.get_spawn_points()[0].location
        return point.x, point.y, CAMERA_START_HEIGHT

    def _place_camera(self, size: Tuple[int, int]) -> None:
        transform = carla.Transform(carla.Location(self.camera_x, self.camera_y, self.camera_height),
                                    carla.Rotation(pitch=-90.0, yaw=0.0))
        if self.camera is None:
            blueprint = self.library.find("sensor.camera.rgb")
            blueprint.set_attribute("image_size_x", str(size[0]))
            blueprint.set_attribute("image_size_y", str(size[1]))
            blueprint.set_attribute("fov", str(CAMERA_FIELD_OF_VIEW))
            self.camera = self.world.spawn_actor(blueprint, transform)
            self.camera.listen(self._on_image)
        else:
            self.camera.set_transform(transform)

    def _on_image(self, image) -> None:
        array = numpy.frombuffer(image.raw_data, dtype=numpy.uint8).reshape((image.height, image.width, 4))
        self.latest_image = array[:, :, 2::-1].copy()

    def meters_per_pixel(self, width: int) -> float:
        return 2 * self.camera_height * math.tan(math.radians(CAMERA_FIELD_OF_VIEW) / 2) / width

    def screen_to_world(self, position: Tuple[int, int], size: Tuple[int, int]) -> Tuple[float, float]:
        """Top-down camera with yaw 0: image up = +x, image right = +y."""
        scale = self.meters_per_pixel(size[0])
        return (self.camera_x + (size[1] / 2 - position[1]) * scale,
                self.camera_y + (position[0] - size[0] / 2) * scale)

    def world_to_screen(self, x: float, y: float, size: Tuple[int, int]) -> Tuple[int, int]:
        scale = self.meters_per_pixel(size[0])
        return int(size[0] / 2 + (y - self.camera_y) / scale), int(size[1] / 2 - (x - self.camera_x) / scale)

    # ------------------------------------------------------------ editing

    def _ground_z(self, x: float, y: float) -> float:
        waypoint = self.map.get_waypoint(carla.Location(x, y, 0.0), project_to_road=True)
        return waypoint.transform.location.z if waypoint else 0.0

    def place(self, x: float, y: float) -> None:
        label, blueprint = PALETTE[self.palette_index]
        actor = {"blueprint": blueprint, "name": "{}_{}".format(label.lower().replace(" ", "_"), len(self.data["actors"]) + 1),
                 "at": {"x": round(x, 2), "y": round(y, 2), "z": round(self._ground_z(x, y), 2), "yaw": 0.0}}
        self.data["actors"].append(actor)
        index = len(self.data["actors"]) - 1
        if self._spawn(index):
            self.selected = index
            self.message = "Placed {}.".format(label)
        else:
            self.data["actors"].pop()
            self.message = "Could not place {} there (blocked?).".format(label)

    def _spawn(self, index: int) -> bool:
        spec = self.data["actors"][index]
        at = spec["at"]
        if "x" not in at:
            return True   # route-relative positions are not shown in the editor
        try:
            blueprint = self.library.find(spec["blueprint"])
        except (IndexError, RuntimeError):
            print("Unknown blueprint {}".format(spec["blueprint"]))
            return False
        height = 1.0 if spec["blueprint"].startswith("walker.") else 0.3
        transform = carla.Transform(carla.Location(at["x"], at["y"], at.get("z", 0.0) + height),
                                    carla.Rotation(yaw=at.get("yaw", 0.0)))
        actor = self.world.try_spawn_actor(blueprint, transform)
        if actor is None:
            return False
        if not spec["blueprint"].startswith("walker."):
            actor.set_simulate_physics(False)
        self.spawned[index] = actor
        return True

    def select_nearest(self, x: float, y: float) -> None:
        best, best_distance = None, 5.0
        for index, spec in enumerate(self.data["actors"]):
            at = spec["at"]
            if "x" in at:
                distance = math.hypot(at["x"] - x, at["y"] - y)
                if distance < best_distance:
                    best, best_distance = index, distance
        self.selected = best
        self.message = "Selected {}.".format(self.data["actors"][best]["name"]) if best is not None else "Nothing there."

    def move_selected(self, dx: float, dy: float, dyaw: float) -> None:
        if self.selected is None:
            return
        at = self.data["actors"][self.selected]["at"]
        at["x"], at["y"] = round(at["x"] + dx, 2), round(at["y"] + dy, 2)
        at["yaw"] = round((at.get("yaw", 0.0) + dyaw) % 360.0, 1)
        actor = self.spawned.get(self.selected)
        if actor is not None:
            height = 1.0 if actor.type_id.startswith("walker.") else 0.3
            actor.set_transform(carla.Transform(carla.Location(at["x"], at["y"], at.get("z", 0.0) + height),
                                                carla.Rotation(yaw=at["yaw"])))

    def delete_selected(self) -> None:
        if self.selected is None:
            return
        actor = self.spawned.pop(self.selected, None)
        if actor is not None:
            actor.destroy()
        removed = self.data["actors"].pop(self.selected)
        # Re-key the remaining spawned actors.
        self.spawned = {(i if i < self.selected else i - 1): a for i, a in self.spawned.items()}
        self.selected = None
        self.message = "Removed {}.".format(removed["name"])

    def set_ego_start(self, x: float, y: float) -> None:
        waypoint = self.map.get_waypoint(carla.Location(x, y, 0.0))
        location, yaw = waypoint.transform.location, waypoint.transform.rotation.yaw
        self.data.setdefault("ego", {})["start"] = {"x": round(location.x, 2), "y": round(location.y, 2),
                                                    "z": round(location.z, 2), "yaw": round(yaw, 1)}
        self.data["ego"].pop("spawn_point", None)
        self.message = "Ego start set (snapped to the lane centre)."

    def set_destination(self, x: float, y: float) -> None:
        waypoint = self.map.get_waypoint(carla.Location(x, y, 0.0))
        location = waypoint.transform.location
        self.data["route"] = {"destination": {"x": round(location.x, 2), "y": round(location.y, 2),
                                              "z": round(location.z, 2)}}
        self.message = "Route destination set."

    def add_trigger(self, x: float, y: float) -> None:
        name = "trigger_{}".format(len(self.data.setdefault("triggers", {})) + 1)
        self.data["triggers"][name] = {"x": round(x, 2), "y": round(y, 2)}
        self.message = "Added trigger '{}' (rename it in the JSON file).".format(name)

    def save(self) -> None:
        if not self.scene_path:
            self.scene_path = input("Save scene as (e.g. scenarios/scenes/my_scene.json): ").strip()
        if not self.scene_path:
            self.message = "Not saved."
            return
        save_scene(self.data, self.scene_path)
        self.message = "Saved {}.".format(self.scene_path)
        print(self.message)

    def close(self) -> None:
        if self.camera is not None:
            self.camera.stop()
            self.camera.destroy()
        for actor in self.spawned.values():
            actor.destroy()

    # ------------------------------------------------------------ drawing

    def draw(self, screen: pygame.Surface, font: pygame.font.Font) -> None:
        size = screen.get_size()
        if self.latest_image is not None:
            surface = pygame.surfarray.make_surface(self.latest_image.swapaxes(0, 1))
            screen.blit(pygame.transform.scale(surface, size), (0, 0))
        else:
            screen.fill((30, 30, 30))
        ego = self.data.get("ego", {}).get("start")
        if ego:
            self._marker(screen, ego["x"], ego["y"], MARKER_COLORS["ego"], "EGO", ego.get("yaw"))
        destination = self.data.get("route", {}).get("destination")
        if destination and "x" in destination:
            self._marker(screen, destination["x"], destination["y"], MARKER_COLORS["destination"], "DEST")
        for name, point in self.data.get("triggers", {}).items():
            if "x" in point:
                self._marker(screen, point["x"], point["y"], MARKER_COLORS["trigger"], name)
        for index, spec in enumerate(self.data["actors"]):
            at = spec["at"]
            if "x" in at:
                color = MARKER_COLORS["selected"] if index == self.selected else MARKER_COLORS["actor"]
                self._marker(screen, at["x"], at["y"], color, spec["name"], at.get("yaw"))
        lines = ["Placing [{}]: {}".format(self.palette_index + 1, PALETTE[self.palette_index][0]),
                 "1-9 choose | click place | right-click select | arrows nudge | , . rotate | Del remove",
                 "G ego start | H destination | T trigger | WASD move | wheel/QE zoom | Ctrl+S save | Esc quit",
                 self.message]
        for row, text in enumerate(lines):
            label = font.render(text, True, (255, 255, 255), (0, 0, 0))
            screen.blit(label, (8, 8 + row * (font.get_height() + 2)))

    def _marker(self, screen, x, y, color, label, yaw=None) -> None:
        px, py = self.world_to_screen(x, y, screen.get_size())
        pygame.draw.circle(screen, color, (px, py), 7, 2)
        if yaw is not None:   # heading tick (yaw 0 = image up)
            angle = math.radians(yaw)
            pygame.draw.line(screen, color, (px, py), (px + 14 * math.sin(angle), py - 14 * math.cos(angle)), 2)
        font = pygame.font.Font(None, 18)
        screen.blit(font.render(label, True, color), (px + 9, py - 6))


def run(client, map_name: Optional[str], scene_path: Optional[str], max_frames: Optional[int] = None) -> None:
    pygame.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("AUTOSIM scene editor")
    font = pygame.font.Font(None, 22)
    editor = SceneEditor(client, map_name, scene_path)
    clock = pygame.time.Clock()
    frames = 0
    try:
        while max_frames is None or frames < max_frames:
            frames += 1
            dt = clock.tick(30) / 1000.0
            editor._place_camera(screen.get_size())
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                if event.type == pygame.MOUSEBUTTONDOWN:
                    x, y = editor.screen_to_world(event.pos, screen.get_size())
                    if event.button == 1:
                        editor.place(x, y)
                    elif event.button == 3:
                        editor.select_nearest(x, y)
                if event.type == pygame.MOUSEWHEEL:
                    editor.camera_height = min(CAMERA_MAX_HEIGHT, max(CAMERA_MIN_HEIGHT, editor.camera_height * (0.9 ** event.y)))
                if event.type == pygame.KEYDOWN:
                    mouse = editor.screen_to_world(pygame.mouse.get_pos(), screen.get_size())
                    control = pygame.key.get_mods() & pygame.KMOD_CTRL
                    if event.key == pygame.K_ESCAPE:
                        return
                    elif control and event.key == pygame.K_s:
                        editor.save()
                    elif pygame.K_1 <= event.key <= pygame.K_9 and event.key - pygame.K_1 < len(PALETTE):
                        editor.palette_index = event.key - pygame.K_1
                    elif event.key == pygame.K_g:
                        editor.set_ego_start(*mouse)
                    elif event.key == pygame.K_h:
                        editor.set_destination(*mouse)
                    elif event.key == pygame.K_t:
                        editor.add_trigger(*mouse)
                    elif event.key == pygame.K_DELETE:
                        editor.delete_selected()
                    elif event.key == pygame.K_COMMA:
                        editor.move_selected(0, 0, -ROTATE_DEGREES)
                    elif event.key == pygame.K_PERIOD:
                        editor.move_selected(0, 0, ROTATE_DEGREES)
                    elif event.key == pygame.K_UP:
                        editor.move_selected(NUDGE_METERS, 0, 0)
                    elif event.key == pygame.K_DOWN:
                        editor.move_selected(-NUDGE_METERS, 0, 0)
                    elif event.key == pygame.K_LEFT:
                        editor.move_selected(0, -NUDGE_METERS, 0)
                    elif event.key == pygame.K_RIGHT:
                        editor.move_selected(0, NUDGE_METERS, 0)
            keys = pygame.key.get_pressed()
            pan = CAMERA_PAN_SPEED * editor.meters_per_pixel(screen.get_width()) * screen.get_width() * dt
            editor.camera_x += pan * (keys[pygame.K_w] - keys[pygame.K_s])
            editor.camera_y += pan * (keys[pygame.K_d] - keys[pygame.K_a])
            zoom = keys[pygame.K_q] - keys[pygame.K_e]
            editor.camera_height = min(CAMERA_MAX_HEIGHT, max(CAMERA_MIN_HEIGHT, editor.camera_height * (1 + 0.8 * dt * zoom)))
            editor.draw(screen, font)
            pygame.display.flip()
    finally:
        editor.close()
        pygame.quit()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--map", default=None, help="map to load, e.g. Town10HD_Opt")
    parser.add_argument("--scene", default=None, help="scene JSON file to open (and save to)")
    parser.add_argument("--host", default=settings.CARLA_HOST)
    parser.add_argument("--port", type=int, default=settings.CARLA_PORT)
    options = parser.parse_args()
    if carla is None:
        raise SystemExit("The 'carla' package is not installed (pip install carla==0.9.15).")
    client = carla.Client(options.host, options.port)
    client.set_timeout(settings.CARLA_TIMEOUT_SECONDS)
    run(client, options.map, options.scene)


if __name__ == "__main__":
    main()
