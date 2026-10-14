"""
datalog.py -- writes what happened during a session to CSV files.

Each session gets its own folder:  <output>/<participant>_<YYYYmmdd_HHMMSS>/

  events.csv        one row per event (button press, answer, mode change,
                    alert, collision, scenario start/end, ...), written as it
                    happens so nothing is lost if the program crashes.
  scenarios.csv     one row per completed scenario (durations, decision
                    answer and correctness, counts, values shown...).
  run_settings.json the command-line options and settings used.
  <scenario file>   a copy of the scenario CSV that was run.

If recording is disabled (--no-record) the logger accepts the same calls
and does nothing.
"""

import csv
import datetime
import json
import os
import shutil
import time
from typing import Any, Dict, List, Optional

EVENT_COLUMNS: List[str] = [
    "wall_time", "session_time_s", "scenario_index", "scenario_id", "scenario_time_s",
    "event", "details", "speed_mph", "x", "y", "mode", "route_traveled_m",
]


class DataLogger:
    """Session data writer.  See module docstring."""

    def __init__(self, output_folder: str, participant_id: str, enabled: bool = True) -> None:
        self.enabled = enabled
        self.folder = ""
        self.session_start = time.monotonic()
        self._event_file = None
        self._event_writer: Optional[csv.DictWriter] = None
        self._summary_rows: List[Dict[str, Any]] = []
        if not enabled:
            return
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.folder = os.path.join(output_folder, "{}_{}".format(participant_id, stamp))
        os.makedirs(self.folder, exist_ok=True)
        self._event_file = open(os.path.join(self.folder, "events.csv"), "w", newline="", encoding="utf-8")
        self._event_writer = csv.DictWriter(self._event_file, fieldnames=EVENT_COLUMNS)
        self._event_writer.writeheader()
        print("[data] Recording to {}".format(self.folder))

    # ------------------------------------------------------------------ API

    def save_run_settings(self, options: Dict[str, Any], scenario_file: str) -> None:
        if not self.enabled:
            return
        with open(os.path.join(self.folder, "run_settings.json"), "w", encoding="utf-8") as handle:
            json.dump(options, handle, indent=2, default=str)
        if os.path.isfile(scenario_file):
            shutil.copy(scenario_file, os.path.join(self.folder, os.path.basename(scenario_file)))

    def event(self, event: str, scenario_index: Any = "", scenario_id: str = "",
              scenario_time: Optional[float] = None, details: Optional[Dict[str, Any]] = None,
              vehicle: Optional[Dict[str, Any]] = None) -> None:
        """Append one event row (flushed immediately)."""
        if not self.enabled or self._event_writer is None:
            return
        row: Dict[str, Any] = {
            "wall_time": datetime.datetime.now().isoformat(timespec="milliseconds"),
            "session_time_s": round(time.monotonic() - self.session_start, 3),
            "scenario_index": scenario_index,
            "scenario_id": scenario_id,
            "scenario_time_s": "" if scenario_time is None else round(scenario_time, 3),
            "event": event,
            "details": json.dumps(details, default=str) if details else "",
        }
        row.update(vehicle or {})
        self._event_writer.writerow({key: row.get(key, "") for key in EVENT_COLUMNS})
        self._event_file.flush()

    def scenario_summary(self, row: Dict[str, Any]) -> None:
        """Add one completed scenario.  The file is rewritten so new columns
        (e.g. from task scripts) always get a header."""
        if not self.enabled:
            return
        self._summary_rows.append(dict(row))
        columns: List[str] = []
        for summary in self._summary_rows:
            for key in summary:
                if key not in columns:
                    columns.append(key)
        path = os.path.join(self.folder, "scenarios.csv")
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=columns)
            writer.writeheader()
            for summary in self._summary_rows:
                writer.writerow({key: summary.get(key, "") for key in columns})

    def close(self) -> None:
        if self._event_file is not None:
            self._event_file.close()
            self._event_file = None
            print("[data] Saved {} scenario summaries in {}".format(len(self._summary_rows), self.folder))
