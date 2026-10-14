"""Scene editor logic against the fake carla module (no server, no screen)."""

import os
import sys

import pygame

import fake_carla
from autosim.scene import load_scene

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))
import scene_editor  # noqa: E402


def test_place_edit_save_and_reload(monkeypatch, tmp_path):
    monkeypatch.setattr(scene_editor, "carla", fake_carla)
    path = str(tmp_path / "my_scene.json")
    editor = scene_editor.SceneEditor(fake_carla.Client("x", 1), None, path)
    size = (800, 600)
    editor._place_camera(size)
    x, y = editor.screen_to_world((400, 300), size)
    assert (round(x, 3), round(y, 3)) == (round(editor.camera_x, 3), round(editor.camera_y, 3))
    assert editor.world_to_screen(x, y, size) == (400, 300)

    editor.place(30.0, 0.5)
    editor.palette_index = 5           # police car
    editor.place(40.0, -3.5)
    editor.select_nearest(30.2, 0.4)
    editor.move_selected(0.25, 0.0, 15.0)
    editor.select_nearest(40.0, -3.5)
    editor.delete_selected()
    editor.set_ego_start(0.0, 0.2)
    editor.set_destination(150.0, 0.0)
    editor.add_trigger(20.0, 0.0)
    pygame.init()
    screen = pygame.Surface(size)
    editor.draw(screen, pygame.font.Font(None, 20))
    editor.save()
    editor.close()

    scene = load_scene(path)
    assert scene.map_name == "Town10HD_Opt"
    assert len(scene.actors) == 1 and scene.actors[0].at.x == 30.25 and scene.actors[0].at.yaw == 15.0
    assert scene.ego_start is not None and scene.route.destination.x == 150.0
    assert "trigger_1" in scene.triggers
