# Task scripts

Most tasks need no programming (see the `task_*` columns in SCENARIOS.md).
For anything else, write a small **task script**: a `.py` file in
`scenarios/scripts/` named in the scenario's `task_script` column.

```python
from autosim.tasks import TaskScript


class Task(TaskScript):
    def on_start(self, ctx):
        ctx.add_alert("GPS signal weak", "warning")

    def on_tick(self, ctx, dt):
        if ctx.elapsed > 10 and not self.asked:
            self.asked = True
            ctx.stop_vehicle("Needs help")
            ctx.ask("Is it safe to continue?", ["Yes=continue", "No=pull_over"])

    def on_answer(self, ctx, label):
        ctx.log("my_answer", label=label)
```

The file must define one subclass of `TaskScript`. Override only the hooks
you need. If a script raises an error, the error is logged (`task_error`)
and the experiment keeps running.

## Hooks

| Hook                                   | Called when                                           |
|----------------------------------------|-------------------------------------------------------|
| `on_start(ctx)`                        | the scenario starts                                   |
| `on_tick(ctx, dt)`                     | every frame (about 30 per second)                     |
| `on_action(ctx, action_id)`            | the operator presses a button (ids in `actions.py`)   |
| `on_answer(ctx, label)`                | the operator answers a question                       |
| `on_mode_change(ctx, old, new)`        | the driving mode changes (`autosim.simulation.DrivingMode`) |
| `on_end(ctx)`                          | the scenario ends                                     |

## The `ctx` API

**Information**

| Name                       | Meaning                                                         |
|----------------------------|-----------------------------------------------------------------|
| `ctx.scenario`             | the scenario row; `ctx.scenario.extra` holds your own columns   |
| `ctx.state`                | speed (`speed_mph`), position, `mode`, `route_traveled_m`, ...  |
| `ctx.status`               | the values shown in the Status panel                            |
| `ctx.elapsed`              | seconds since the scenario started                              |
| `ctx.rng`                  | random generator (reproducible with `--seed`)                   |
| `ctx.distance_to(route_m)` | metres along the route from the car to that point               |
| `ctx.trigger_route_m(name)`| route position of a trigger in the scene                        |

**The automated vehicle.** These act immediately, as the AV's own decisions:

`ctx.stop_vehicle(reason)`, `ctx.resume_vehicle()`, `ctx.pull_over()`,
`ctx.change_lane("left"|"right")`, `ctx.set_target_speed(mph)`,
`ctx.perform_outcome("continue")` (any outcome from SCENARIOS.md).

**The world**

| Call                                                       | Effect                                     |
|------------------------------------------------------------|--------------------------------------------|
| `ctx.spawn(blueprint, route_m, lane=0, lateral_m=0, yaw=0)`| place an object; returns a handle          |
| `ctx.spawn_walker(start, goal, speed_mps=1.4)`             | pedestrian walking between two positions (dicts as in scene files) |
| `ctx.destroy(handle)`                                      | remove it                                  |

**The interface**

| Call                                              | Effect                                              |
|---------------------------------------------------|-----------------------------------------------------|
| `ctx.ask(prompt, options, on_answer=None)`        | question with buttons; options are `"Label=outcome"` |
| `ctx.add_alert(text, severity)` / `remove_alert`  | Alerts panel (`info`, `warning`, `critical`)        |
| `ctx.notify(title, body, severity, actions)`      | bell + banner; `actions=[("Accept", function)]`     |
| `ctx.set_status(key, value)`                      | change a Status value (e.g. `software_status`)      |
| `ctx.say(speaker, message)`                       | someone speaks: text or a file in `scenarios/audio` |
| `ctx.feedback(text)`                              | short fading text in the Actions panel              |

**Data**

| Call                         | Effect                                                    |
|------------------------------|-----------------------------------------------------------|
| `ctx.log(event, **details)`  | a row in `events.csv`                                     |
| `ctx.record(name, value)`    | a column in this scenario's row of `scenarios.csv`        |

Record **measurements**, not verdicts. For example, `pedestrian_gap.py`
records the gap that was available when "Clear to go" was pressed. Whether
that was a good decision is left to the experimenter.

## Examples

* `scenarios/scripts/pedestrian_gap.py`: spawns pedestrians crossing in
  front of the stopped car with configurable gaps (`crossing_gaps_s`
  column), and records the gap available at the operator's "Clear to go".
* `scenarios/scripts/software_fault.py`: after `fault_after_s` seconds the
  software status turns to "Restart needed" and the car slows. Once pulled
  over, a notification offers a restart.
