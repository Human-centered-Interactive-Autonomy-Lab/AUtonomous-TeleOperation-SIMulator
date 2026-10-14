# Architecture

```
                         python -m autosim
                                 |
                          experiment.py            command line, scenario order,
                                 |                 waiting between scenarios
          +----------------------+----------------------+
          |                      |                      |
   scenario.py / scene.py   session.py  <------>   interface.py + actions.py
   (CSV row + JSON world)   one scenario:          pygame window: draws the
                            alerts, decisions,     session state, turns clicks
                            notifications,         into session calls
                            status, logging
                              |        |
                    tasks.py  |        |  link.py
              (question task,|        |  CommandLink: input delay
               task scripts) |        |  VideoLink: video delay, frame rate
                              v        v
                         simulation.py  (interface)
                           /                 \
              simulation_carla.py        simulation_mock.py
              CARLA + BasicAgent          2-D stand-in (--mock)
```

## One frame (about 30 per second)

1. `interface.handle_events()`: mouse clicks call handlers from
   `actions.py`. A handler records the action and sends vehicle commands
   through `session.send_vehicle_command()`, which queues them in the
   `CommandLink` (input delay).
2. `session.update()`: runs commands that are due, applies the delayed
   manual-driving input, calls `simulation.tick()` (which returns a
   `VehicleState`), runs the task hooks, and passes camera frames to the
   `VideoLink`.
3. `interface.draw()`: paints every panel from the session state, using the
   delayed camera frames from the `VideoLink`.

## Design decisions

* **The ego vehicle is driven by AUTOSIM on every tick.** It never uses
  Traffic Manager autopilot, so nothing overrides an emergency stop.
  Autonomous driving uses CARLA's `BasicAgent`, and AUTOSIM adds stopping
  for props and pedestrians.
* **Scenes are spawned directly from JSON.** Recorder `.log` replays are not
  used: they also drive the ego vehicle, which conflicts with operator
  commands, they cannot be edited by hand, and they depend on paths on the
  server machine.
* **Immediate-mode interface.** Panels are redrawn from the session state
  every frame, and clickable areas are recorded while drawing. No widget
  state needs to be kept in sync.
* **One simulator interface.** The GUI, scenarios and logging never touch
  CARLA directly, which makes the mock mode and the tests possible.
* **Facts, not grades.** The data records what the operator did and when.
  Scoring is left to the experimenter.
