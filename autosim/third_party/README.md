# Third-party code

`carla_agents/agents/` is an unmodified copy of the navigation agents that ship
with CARLA 0.9.15 (`PythonAPI/carla/agents`: BasicAgent, LocalPlanner,
GlobalRoutePlanner, PID controllers), copyright (c) 2018-2020 CVC, released
under the MIT license (see the header of each file).

AUTOSIM uses the `agents` package from your CARLA installation when it can be
found (the `CARLA_AGENTS_PATH` setting, or the `CARLA_ROOT` environment
variable), and falls back to this copy otherwise.
