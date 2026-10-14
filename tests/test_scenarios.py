import os
import random

import pytest

from autosim.experiment import build_schedule
from autosim.scenario import ScenarioFileError, load_scenarios
from conftest import EXAMPLE_BATTERY


def write_csv(folder, text):
    path = os.path.join(str(folder), "battery.csv")
    with open(path, "w") as handle:
        handle.write(text)
    return path


def test_example_battery_loads():
    scenarios = load_scenarios(EXAMPLE_BATTERY)
    assert len(scenarios) >= 10
    blocked = [s for s in scenarios if s.scenario_id == "blocked_lane"][0]
    assert blocked.task.options[0] == ("Yes", "change_lane_left")
    assert blocked.scene.actors[0].is_vehicle


def test_minimal_file_uses_defaults(tmp_path):
    path = write_csv(tmp_path, "scenario_id\nonly_id\n")
    scenario = load_scenarios(path)[0]
    values = scenario.resolve_status(random.Random(0))
    for key in ("tire_fl_psi", "tire_fr_psi", "tire_rl_psi", "tire_rr_psi"):
        assert 30 <= values[key] <= 34
    assert 3 <= values["route_total_miles"] <= 15
    assert 0 <= values["route_progress_percent"] <= 75
    assert values["software_status"] == "Good"
    assert scenario.scene.map_name is None


def test_cells_override_and_accept_distributions(tmp_path):
    path = write_csv(tmp_path, "scenario_id,battery_percent,motor_temp_f,my_column\n"
                               "a,12,uniform:200;210,hello\n")
    scenario = load_scenarios(path)[0]
    values = scenario.resolve_status(random.Random(0))
    assert values["battery_percent"] == 12
    assert 200 <= values["motor_temp_f"] <= 210
    assert scenario.extra["my_column"] == "hello"


def test_all_problems_are_reported_together(tmp_path):
    path = write_csv(tmp_path, "scenario_id,weather,task_options,scene\n"
                               "a,snowy,Yes=fly,\n"
                               ",clear,,\n"
                               "a,clear,,missing.json\n")
    with pytest.raises(ScenarioFileError) as error:
        load_scenarios(path)
    text = str(error.value)
    assert "snowy" in text and "fly" in text and "scenario_id is empty" in text
    assert "duplicate" in text and "missing.json" in text


def test_correct_responses_are_only_copied(tmp_path):
    path = write_csv(tmp_path, "scenario_id,task_prompt,task_options,correct_responses\n"
                               "a,Go?,Yes=continue|No=stay_stopped,Yes (unless pedestrians)\n")
    scenario = load_scenarios(path)[0]
    assert scenario.correct_responses == "Yes (unless pedestrians)"


def test_schedule_orders_and_never_loops_forever():
    items = list(range(5))
    rng = random.Random(4)
    assert build_schedule(items, "in_order", 0, False, rng) == items
    assert build_schedule(items, "in_order", 7, False, rng) == [0, 1, 2, 3, 4, 0, 1]
    shuffled = build_schedule(items, "random", 10, False, rng)
    assert sorted(shuffled[:5]) == items and sorted(shuffled[5:]) == items
    with_replacement = build_schedule(items, "random", 50, True, rng)
    assert len(with_replacement) == 50 and set(with_replacement) <= set(items)
