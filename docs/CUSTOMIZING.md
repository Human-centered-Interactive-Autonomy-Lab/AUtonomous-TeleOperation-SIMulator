# Customizing AUTOSIM

## Defaults: `autosim/settings.py`

Every default lives in this one file, grouped in commented sections:
connection, files, experiment flow, link quality, **default vehicle status**,
environment presets, driving behaviour, manual control, camera mounts,
interface layout and colors, and audio. Command-line options and scenario
columns override these defaults. They do not replace the file.

Examples:

```python
DEFAULT_VEHICLE_STATUS["battery_percent"] = "normal:70,10"   # draw battery around 70 %
DEFAULT_WEATHER = "overcast"
WEATHER_PRESETS["drizzle"] = {"cloudiness": 90, "precipitation": 20, "wetness": 40}
MANUAL_CONTROL_MAX_ENTRY_SPEED_MPH = 3.0
EMERGENCY_STOP_IGNORES_INPUT_DELAY = False      # E-STOP obeys the input delay (default)
```

## Layout: `settings.PANEL_LAYOUT`

Each panel is a rectangle given as fractions of the window
`(left, top, width, height)`, so the layout scales to any window size.
Move a panel by editing its numbers, and remove one by deleting its line.
Camera images are created at the size of their panel, so they are never
stretched.

## Buttons and dialogs: `autosim/actions.py`

All buttons are defined in the `ACTIONS` list, and all dialogs in `DIALOGS`.
To add a button:

```python
def _flash_headlights(session):
    session.record_action("flash_headlights")          # logged + task hook
    session.send_vehicle_command("flash_headlights",   # goes through the input delay
                                 lambda: session.simulation.set_hazard_lights(True))
    session.feedback("Headlights flashed.")

ACTIONS.append(Action("flash_headlights", "Flash headlights", "actions", _flash_headlights, "link"))
```

`group` decides where it appears: `actions` (command list), `drive` (big
buttons), `end`, `map` or `status`. A handler can return the name of a dialog
to open one. To hide a button in a single scenario, put its id in that
scenario's `hidden_buttons` column.

## Camera mounts: `settings.CAMERA_MOUNTS`

Position (metres, relative to the car's centre) and field of view of each
camera. The top-down camera is 14 m above the car by default: a close
"surround view" rather than a satellite view.

## Another simulator

The program talks to the simulator only through the methods of
`autosim/simulation.py`. Implementing those methods for another simulator
leaves the interface, scenario files and logging unchanged.
`simulation_carla.py` is the reference implementation.
