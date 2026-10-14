"""Runs the real main loop (python -m autosim --mock) for two short scenarios."""

import os

import autosim.session
from autosim.experiment import main
from conftest import EXAMPLE_BATTERY


def test_main_loop_with_mock(monkeypatch, tmp_path):
    original_update = autosim.session.ScenarioSession.update

    def update_then_end(self, dt, manual_keys=None):
        original_update(self, dt, manual_keys)
        self.frames = getattr(self, "frames", 0) + 1
        if self.frames == 10:
            self.end_requested = True     # as if "End supervision" was pressed
    monkeypatch.setattr(autosim.session.ScenarioSession, "update", update_then_end)

    exit_code = main(["--mock", "--scenarios", EXAMPLE_BATTERY, "--count", "2", "--order", "random",
                      "--seed", "3", "--wait-time", "0.1", "--no-audio", "--window-size", "800x450",
                      "--output", str(tmp_path), "--participant", "TEST"])
    assert exit_code == 0
    folder = os.path.join(str(tmp_path), os.listdir(str(tmp_path))[0])
    assert sorted(os.listdir(folder)) == ["events.csv", "example_battery.csv", "run_settings.json", "scenarios.csv"]
    with open(os.path.join(folder, "scenarios.csv")) as handle:
        assert len(handle.read().strip().splitlines()) == 3   # header + 2 scenarios


def test_bad_scenario_file_reports_and_exits(tmp_path, capsys):
    bad = tmp_path / "bad.csv"
    bad.write_text("scenario_id,weather\na,snowy\n")
    assert main(["--mock", "--scenarios", str(bad), "--no-record"]) == 2
    assert "snowy" in capsys.readouterr().out
