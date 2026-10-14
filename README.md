# AUTOSIM - AUtonomous TeleOperation SIMulator

AUTOSIM is an open-source platform for **remote-assistance (teleoperation)
experiments with automated vehicles**, built on the
[CARLA](https://carla.org) simulator's Python API. It contains:

* an **operator interface** (cameras, map, status, alerts, decisions, actions),
* an **experiment manager** that runs a battery of scenarios from a CSV file,
  with configurable timing, simulated video/input delay and data logging, and
* tools to **author scenarios** (scene JSON files, task scripts).

> Work in progress. Full documentation follows in `docs/`.

## Quick start

```bash
pip install -r requirements.txt
python -m autosim --mock                 # try it without CARLA
python -m autosim --scenarios scenarios/example_battery.csv   # with a running CARLA server
python -m autosim --help                 # every option
```
