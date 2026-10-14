# Scenario files

A **battery** of scenarios is one CSV file. Each row is one scenario. Edit
it in Excel or any text editor (save as CSV UTF-8). Start from
`scenarios/example_battery.csv`.

```bash
python -m autosim --scenarios path/to/my_battery.csv
```

## Rules

* Only `scenario_id` is required. Every other column is optional, and any
  cell may be left blank.
* A blank cell or missing column uses the default from
  `autosim/settings.py`, either a fixed value or a random draw (see below).
* Columns AUTOSIM does not recognise are kept and passed to task scripts as
  `ctx.scenario.extra["column_name"]`.
* When the file is read, **every** problem is reported at once (unknown
  preset, missing scene file, bad distribution, ...), and the program does
  not start until they are fixed.
* Lists inside one cell are separated by `|`.

## Columns

### Identification

| Column        | Meaning                                                         |
|---------------|-----------------------------------------------------------------|
| `scenario_id` | Unique name. Used in the data files.                            |
| `group`       | Any label. `--group NAME` runs only rows of that group.         |
| `description` | Notes for the experimenter (never shown to the participant).    |

### World

| Column             | Meaning                                                          | Default            |
|--------------------|------------------------------------------------------------------|--------------------|
| `scene`            | Scene JSON file (next to the CSV, or in its `scenes/` folder)    | default scene (*)  |
| `time_of_day`      | `day`, `sunset`, `night` (presets in settings `SUN_PRESETS`)     | `day`              |
| `weather`          | `clear`, `overcast`, `rain`, `storm`, `heavy_fog`, `dust_storm`  | `clear`            |
| `lights`           | street lights: `auto` (on at night), `off`, `low`, `high`        | `auto`             |
| `target_speed_mph` | cruising speed of the automated vehicle                          | 20                 |
| `start_stopped`    | `yes` = the car stands still at the start                        | no                 |
| `route_end`        | `customer` or `depot`: text shown when the route is completed    | `customer`         |

(*) The default scene keeps the loaded map, starts at spawn point 0 and drives
400 m along the road.

### What the operator sees

| Column           | Meaning                                                                      |
|------------------|------------------------------------------------------------------------------|
| `alerts`         | Alerts at start, separated by `|`. Prefix `critical:`, `warning:` or `info:`. |
| `hidden_buttons` | Action buttons to hide, e.g. `honk|talk_externally` (ids in `actions.py`)    |
| status columns   | See the next table                                                           |

**Status columns.** A cell can hold a number, text, or a distribution.

| Column                                                                   | Default             |
|--------------------------------------------------------------------------|---------------------|
| `route_total_miles`                                                      | `uniform:3,15`      |
| `route_progress_percent`                                                 | `uniform:0,75`      |
| `battery_percent`                                                        | `uniform:40,95`     |
| `battery_temp_f`                                                         | `uniform:70,95`     |
| `motor_temp_f`                                                           | `uniform:120,170`   |
| `efficiency_percent`                                                     | `uniform:80,95`     |
| `tire_fl_psi`, `tire_fr_psi`, `tire_rl_psi`, `tire_rr_psi`               | `uniform:30,34`     |
| `cabin_temp_f`                                                           | `uniform:68,74`     |
| `outdoor_temp_f`                                                         | `uniform:40,80`     |
| `windows`, `seatbelts`, `passengers`                                     | `All up`, `All fastened`, `1` |
| `software_status`, `detection_status`, `route_planner_status`, `control_module_status` | `Good` |
| `camera_front_status`, `camera_rear_status`, `camera_left_status`, `camera_right_status` | `Good` |
| `lidar_status`, `radar_status`, `gps_status`, `connection_status`        | `Good`              |

Numbers outside `STATUS_NORMAL_RANGES` in settings, and status words other
than "Good"/"OK", are highlighted. Random values are drawn when the scenario
starts and are written to the data (`shown_*` columns), so you always know
what the participant saw.

### Task (no programming)

| Column         | Meaning                                                                                 |
|----------------|-----------------------------------------------------------------------------------------|
| `task_trigger` | When the task fires: `start`, a distance in metres along the route (`55`), or the name of a trigger point in the scene. |
| `task_prompt`  | The question shown in the Decision panel.                                              |
| `task_options` | Answer buttons and what each does: `Yes=continue | No=change_lane_left`.               |
| `task_alert`   | Alert added when the task fires (prefix with a severity if you like).                    |
| `task_script`  | Python task script (next to the CSV or in its `scripts/` folder). See TASK_SCRIPTS.md.   |

When a task with a question fires, the car brakes to a stop, shows the
alert, and asks the question. If the car already stopped by itself (for
example for an obstacle) shortly before the trigger point, the question
appears then.

Outcomes an answer can have:

| Outcome             | Effect                                                         |
|---------------------|----------------------------------------------------------------|
| `continue`          | resume driving (obstacles the car waited for count as checked) |
| `stay_stopped`      | keep waiting                                                   |
| `change_lane_left`  | move one lane left, then continue                              |
| `change_lane_right` | move one lane right, then continue                             |
| `pull_over`         | pull over to the road edge and stop                            |
| `manual_control`    | ask the operator to drive manually                             |
| `reroute`           | follow the scene's first alternate route (`reroute:2` = second) |
| `none`              | only record the answer                                          |

Answers are operator commands, so they obey the simulated input delay.

### Responses (for your analysis)

| Column                | Meaning                                                                 |
|-----------------------|-------------------------------------------------------------------------|
| `correct_responses`   | Free text: what you consider correct (an answer, a button, a sequence). |
| `incorrect_responses` | Free text: what you consider incorrect.                                  |

These are **copied unchanged** into the scenario summary. AUTOSIM never
judges responses. It records answers, button presses and their times
(`action_sequence`), and you decide what was right.

### Communication

| Column              | Meaning                                                            |
|---------------------|--------------------------------------------------------------------|
| `passenger_message` | A passenger-talk notification appears. On Accept, this is said.    |
| `passenger_reply`   | Said after the operator talks to the passenger (push-to-talk).     |
| `external_reply`    | Said after the operator talks to people outside.                   |

Each can be plain text or a **file in `scenarios/audio/`**. An audio file
(`.wav`, `.ogg`, `.mp3`) is played, and a same-named `.txt` is shown as its
transcript. A `.txt` file is shown as text. The `.txt` files included are
placeholders describing what each recording should say.

### Link quality per scenario

| Column        | Meaning                                                                 |
|---------------|-------------------------------------------------------------------------|
| `video_delay` | Overrides the command-line video delay, e.g. `normal:0.3,0.05`           |
| `input_delay` | Overrides the command-line input delay                                   |

## Distributions

Wherever a number is expected you can write a distribution instead:

| Written as                          | Meaning                                  |
|-------------------------------------|------------------------------------------|
| `31`                                | always 31                                |
| `uniform:30,34`                     | uniform between 30 and 34                |
| `normal:10,2`                       | normal, mean 10, standard deviation 2    |
| `exponential:20`                    | exponential, mean 20                     |
| `clipped_exponential:20,5,60`       | exponential, mean 20, clipped to 5..60   |
| `choice:Good,Degraded,Fault`        | one of the values, equally likely        |

Named arguments also work (`uniform:low=30,high=34`). Use `;` instead of
`,` (`uniform:30;34`) if you edit the CSV by hand. Excel adds the needed
quotes automatically.

## Order, count and timing (command line)

| Option                                       | Meaning                                                   |
|----------------------------------------------|-----------------------------------------------------------|
| `--order in_order` / `random`                | file order or shuffled                                    |
| `--with-replacement`                         | random picks may repeat (otherwise each row once per pass)|
| `--count N`                                  | run N scenarios (loops through the file if needed)        |
| `--seed N`                                   | same order and same random values every time              |
| `--wait-time`, `--wait-distribution`, `--wait-spread`, `--wait-min`, `--wait-max` | time between scenarios |

The scenario's loading time counts toward the wait, and both the drawn and
actual waits are logged.
