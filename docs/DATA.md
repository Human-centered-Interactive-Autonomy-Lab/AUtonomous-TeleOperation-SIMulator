# Data files

Each session writes a folder `data/<participant>_<YYYYmmdd_HHMMSS>/`
(change it with `--output` and `--participant`, or disable it with
`--no-record`).

| File                 | Content                                                        |
|----------------------|----------------------------------------------------------------|
| `events.csv`         | every event, one per row, written as it happens               |
| `scenarios.csv`      | one row per scenario                                           |
| `run_settings.json`  | the command-line options and the scenario order                |
| `<scenario file>.csv`| a copy of the scenario file that was run                       |

AUTOSIM records **what happened and when**. It does not grade responses.
Use the `correct_responses` / `incorrect_responses` columns (copied into
`scenarios.csv`) to score the data however your study requires.

## events.csv

| Column                          | Meaning                                                    |
|---------------------------------|------------------------------------------------------------|
| `wall_time`                     | clock time (ISO format, milliseconds)                       |
| `session_time_s`                | seconds since the session started                           |
| `scenario_index`, `scenario_id` | which scenario                                              |
| `scenario_time_s`               | seconds since the scenario started                          |
| `event`                         | event type (below)                                          |
| `details`                       | JSON with event-specific values                             |
| `speed_mph`, `x`, `y`, `mode`, `route_traveled_m` | vehicle state at that moment              |

Event types include: `scenario_start` (with every status value shown),
`wait_before_scenario`, `operator_action` (every button and answer, with
details such as call partner or fleet request), `command_executed` (when a
delayed command actually reached the vehicle), `decision_shown`,
`alert_shown`, `notification`, `notification_opened`, `mode_change`,
`task_triggered`, `speech`, `talk_start`/`talk_end`, `collision`,
`manual_control_refused`, `map_zoom`, `status_changed`, `route_complete`,
`task_error`, `scenario_end`, plus anything task scripts log.

## scenarios.csv

| Column                                   | Meaning                                               |
|------------------------------------------|-------------------------------------------------------|
| `scenario_index`, `order_index`, `scenario_id`, `group`, `scene` | identification               |
| `wait_drawn_s`, `wait_actual_s`          | inter-scenario wait before this scenario              |
| `duration_s`                             | start to "End supervision"                            |
| `first_action`, `time_to_first_action_s` | the operator's first action                           |
| `action_sequence`                        | every action as `id@seconds`, in order                |
| `decision_prompt`, `decision_shown_at_s` | the first question asked                              |
| `decision_answer`                        | the answer, or `(unanswered)`                         |
| `decision_latency_s`                     | time from question to answer                          |
| `decisions_total`, `decisions_unanswered`|                                                       |
| `commands_total`, `emergency_stops`, `pull_overs`, `manual_control_s` |                          |
| `collisions`, `route_complete`, `final_mode`, `ended_by` |                                         |
| `video_delay`, `input_delay`             | the link settings used                                |
| `correct_responses`, `incorrect_responses` | copied from the scenario file                       |
| `shown_<status>`                         | every status value the participant saw                |
| *(task script columns)*                  | anything recorded with `ctx.record(...)`              |
