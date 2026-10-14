"""Exercises simulation_carla.py against tests/fake_carla.py (no CARLA server).

These tests check AUTOSIM's own logic in the CARLA backend (spawning, route
placement, obstacle stops, lane change, pull over, E-STOP, cleanup).  They
cannot check CARLA's real physics or rendering.
"""

import types

import pytest

import autosim.simulation_carla as backend
import fake_carla
from autosim.scene import Placement, Scene
from autosim.simulation import CameraRequest, DrivingMode


class _Planner:
    def __init__(self, map_inst, resolution):
        pass

    def trace_route(self, start, end):
        return []


@pytest.fixture
def simulation(monkeypatch):
    monkeypatch.setattr(backend, "carla", fake_carla)
    road_option = types.SimpleNamespace(LANEFOLLOW="LANEFOLLOW")
    monkeypatch.setattr(backend, "_import_agents", lambda: (fake_carla.FakeAgent, _Planner, road_option))
    sim = backend.CarlaSimulation()
    sim.connect()
    yield sim
    sim.close()


def load(sim, scene_data, start_stopped=False):
    scene = Scene(scene_data, "test")
    sim.load_scenario(scene, "day", "clear", "auto", [CameraRequest("front", 64, 32)], 20.0, start_stopped)
    return scene


def run(sim, seconds):
    for _ in range(int(seconds * 30)):
        state = sim.tick(1 / 30)
    return state


def test_map_check(simulation):
    assert simulation.missing_maps(["Town10HD_Opt", "Town01", "Town99"]) == ["Town99"]


def test_route_relative_actor_lands_on_route(simulation):
    load(simulation, {"route": {"length_m": 200},
                      "actors": [{"blueprint": "static.prop.constructioncone", "at": {"route_m": 60, "lane": 1, "lateral_m": 0.5}}]})
    cone = [a for a in simulation.spawned if a.type_id == "static.prop.constructioncone"][0]
    assert abs(cone.get_location().x - 60) < 2.1
    assert abs(cone.get_location().y - 4.0) < 0.01          # right lane (3.5) + 0.5 m


def test_unknown_blueprint_is_skipped(simulation, capsys):
    load(simulation, {"actors": [{"blueprint": "static.prop.nonexistent", "at": {"route_m": 10}}]})
    assert "does not exist" in capsys.readouterr().out


def test_stops_for_prop_then_lane_change(simulation):
    load(simulation, {"route": {"length_m": 300},
                      "actors": [{"blueprint": "vehicle.carlamotors.carlacola", "at": {"route_m": 60}}]})
    state = run(simulation, 15)
    assert state.mode == DrivingMode.HOLDING and "Obstacle" in state.stop_reason
    assert state.route_traveled_m < 56
    assert simulation.change_lane("left")
    simulation.resume_autonomy()
    state = run(simulation, 12)
    assert state.mode == DrivingMode.AUTONOMOUS
    assert state.route_traveled_m > 70
    assert state.collision_count == 0


def test_emergency_stop_brakes_and_holds(simulation):
    load(simulation, {"route": {"length_m": 300}})
    run(simulation, 5)
    simulation.emergency_stop()
    state = run(simulation, 4)
    assert state.mode == DrivingMode.EMERGENCY_STOP and state.speed_mph < 0.5
    assert simulation.ego.last_control.throttle == 0.0 and simulation.ego.last_control.hand_brake


def test_pull_over(simulation):
    load(simulation, {"route": {"length_m": 300}})
    run(simulation, 5)
    simulation.pull_over()
    state = run(simulation, 12)
    assert state.mode == DrivingMode.PULLED_OVER and state.hazard_lights_on
    assert simulation.ego.get_location().y > 0.5             # moved toward the road edge


def test_route_completion(simulation):
    load(simulation, {"route": {"length_m": 60}})
    state = run(simulation, 30)
    assert state.mode == DrivingMode.ROUTE_COMPLETE


def test_manual_control_and_walker_and_cleanup(simulation):
    load(simulation, {"route": {"length_m": 200}}, start_stopped=True)
    assert run(simulation, 1).mode == DrivingMode.HOLDING
    simulation.set_manual_control(True)
    simulation.apply_manual_input(0.3, 0.0, 0.0, False)
    state = run(simulation, 3)
    assert state.mode == DrivingMode.MANUAL and state.speed_mps > 0.5
    walker = simulation.spawn_walker(Placement({"route_m": 30, "lateral_m": -6}), Placement({"route_m": 30, "lateral_m": 6}))
    run(simulation, 2)
    assert walker.get_location().y > -6                     # it is walking
    assert "front" in simulation.camera_frames()
    world = simulation.world
    simulation.cleanup_scenario()
    assert len(world.actors) == 0
