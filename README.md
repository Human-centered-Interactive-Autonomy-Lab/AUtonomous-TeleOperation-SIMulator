# AUTOSIM - AUtonomous TeleOperation SIMulator

AUTOSIM is an open-source testbed for **remote-assistance (teleoperation)
experiments with automated vehicles**, built on the Python API of the
[CARLA](https://carla.org) driving simulator.X 

A participant acts as a **remote operator** (RO) supervising a self-driving
car. The car sometimes asks for help: *"Is the left lane open?"*, *"Heavy
pedestrian traffic, please advise"*. Other times something goes wrong: a
software fault, low battery, a carsick passenger. The operator answers
questions and gives high-level commands (pull over, emergency stop, call
emergency or fleet services, talk to the passenger), or drives slowly by hand.
AUTOSIM records what the operator did and when.

## Features

* **Operator interface**: five camera views (front, left, right, rear,
  top-down), speed, alerts, notification bell, decision prompts with answer
  buttons, vehicle status with detailed sensor view, map with route, GPS and
  routing tools, route progress, and action buttons (E-STOP, Pull Over,
  Manual Control, Resume AV, emergency and fleet services, talk to the
  passenger or to people outside, honk, hazard lights).
* **Scenario batteries in a CSV file.** Each row is one scenario. Any value
  you leave out (battery %, tire pressure, weather, ...) gets a sensible,
  optionally random, default.
* **Scenes in JSON files.** Props, parked vehicles, police cars and
  pedestrians can be placed in map coordinates or relative to the car's
  route ("a cone 60 m ahead in the right lane"). A click-to-place
  **scene editor** is included.
* **Task logic without programming.** For example: stop at 55 m, ask a
  question, and change lanes on "Yes". Optional **Python task scripts**
  handle anything else, such as pedestrians crossing with timed gaps or a
  software fault.
* **Experiment control** from the command line: scenario order (in order,
  random with or without replacement), number of scenarios, and the time
  between scenarios (deterministic, uniform, normal, exponential or clipped
  exponential).
* **Simulated network link**: video delay, input (command) delay, video
  frame rate and its variability, and video resolution, each with a
  configurable distribution.
* **Data logging**: an event log of every action, answer, alert and
  collision with timestamps, plus a per-scenario summary CSV. The simulator
  records facts only. Which responses count as correct is decided by the
  experimenter.

## Installation

1. Install CARLA (tested with the packaged **CARLA 0.9.15**) and start it
   (`CarlaUE4.exe` or `./CarlaUE4.sh`).
2. Install Python 3.7 or newer and the dependencies:
   ```bash
   pip install -r requirements.txt
   pip install carla==0.9.15        # or the wheel in <CARLA>/PythonAPI/carla/dist/
   ```
   The CARLA client package must match the server version. For 0.9.15 on
   Windows, the wheel shipped with CARLA targets Python 3.7.

## Running

```bash
# Run the example battery (start CARLA first):
python -m autosim --scenarios scenarios/example_battery.csv --participant P001

# A randomized battery of 12 scenarios with exponential waits and degraded video:
python -m autosim --order random --count 12 \
    --wait-time 20 --wait-distribution exponential \
    --video-delay 0.3 --video-delay-distribution normal --video-delay-spread 0.05 \
    --video-fps 15 --input-delay 0.2

python -m autosim --help        # every option with its default
```

If CARLA cannot be reached, AUTOSIM prints a warning and opens the
interface anyway, with black camera views and inactive buttons, so the
layout can be inspected. Nothing is recorded in that case.

Each scenario ends when the operator presses **End supervision**. Close the
window (or press Shift+Esc) to stop early. Data is saved either way, in
`data/<participant>_<date>_<time>/`.

## Writing your own experiment

| You want to...                               | Read                                           |
|----------------------------------------------|------------------------------------------------|
| define scenarios (CSV columns, defaults)      | [docs/SCENARIOS.md](docs/SCENARIOS.md)         |
| place objects in the world (scene files)      | [docs/SCENES.md](docs/SCENES.md)               |
| add custom task behaviour in Python           | [docs/TASK_SCRIPTS.md](docs/TASK_SCRIPTS.md)   |
| change buttons, panels, colors, defaults      | [docs/CUSTOMIZING.md](docs/CUSTOMIZING.md)     |
| understand the data files                     | [docs/DATA.md](docs/DATA.md)                   |
| understand how the code fits together         | [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)   |

The `scenarios/` folder has a working example of each piece:
`example_battery.csv` (13 scenarios covering the task types below),
`scenes/*.json`, `scripts/*.py` and `audio/`.

### Example task types included

| Scenario                  | What happens                                                               |
|---------------------------|----------------------------------------------------------------------------|
| `construction_closed/open`| Construction ahead; the AV asks whether the road is closed (go / no-go)     |
| `police_blockage`         | Police cars block the road; drive through, detour or wait                  |
| `blocked_lane`            | Parked van; "Is the left lane open?" (lane change)                          |
| `debris_unclear`          | Unclear object on the road; continue, go around or wait                     |
| `pedestrian_gap`          | Pedestrians cross with 3 s / 6 s gaps; operator says when it is clear      |
| `software_fault`          | Software fault mid-drive; pull over, then restart                           |
| `low_battery`, `low_tire` | Maintenance alerts; fleet services                                          |
| `passenger_discomfort`    | Passenger asks to talk ("I feel carsick")                                    |
| `weather_sensors`         | Storm, degraded sensors; continue or pull over                              |
| `lane_markings`           | Night, unreadable markings; "How many lanes?"                               |
| `angry_bystander`         | Upset pedestrian next to the stopped car                                    |

## Project layout

```
autosim/                 the program
  experiment.py          command line, scenario order, main loop    (start here)
  settings.py            every default value, grouped and commented
  scenario.py, scene.py  reading scenario CSV and scene JSON files
  session.py             runs one scenario (alerts, decisions, logging)
  tasks.py               question tasks and the task-script API
  interface.py           the operator window (pygame)
  actions.py             all buttons and dialogs (add yours here)
  link.py                simulated video/input delay and frame rate
  simulation.py          the simulator interface ...
  simulation_carla.py    ... implemented with CARLA
  datalog.py, audio.py, messages.py, distributions.py
scenarios/               example battery, scenes, task scripts, audio
tools/scene_editor.py    click-to-place scene editor (needs CARLA)
tests/                   tests of the file formats and timing logic (pytest)
docs/                    documentation
```

## Tests

```bash
pip install pytest
python -m pytest
```

The tests cover the parts that do not need the simulator: reading
scenario files, distributions, scenario order, and the simulated video and
input delays.

## Status and limitations

AUTOSIM is under active development. Known limitations:

* The autonomous driving uses CARLA's `BasicAgent`. It follows the route,
  stops for vehicles and red lights, and AUTOSIM adds stops for props and
  pedestrians in the lane. It is not a production-grade planner.
* Hazard lights use CARLA's left and right blinker lights.
* "Talk to passenger / externally" is push-to-talk with scripted replies
  (text and/or pre-recorded audio). There is no live voice link.

## License

MIT. See [LICENSE](LICENSE). `autosim/third_party/carla_agents` contains
CARLA's MIT-licensed navigation agents (see its README).
