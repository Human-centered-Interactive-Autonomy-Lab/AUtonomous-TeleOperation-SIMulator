"""
settings.py -- every tunable default in AUTOSIM, in one place.

How to use this file
--------------------
* Change a value here and it takes effect everywhere.
* Values that a researcher typically changes per session (scenario file,
  wait times, video delay, ...) can ALSO be overridden from the command line
  (see ``python -m autosim --help``); the values below are then only the
  defaults shown by ``--help``.
* Values that change per scenario (battery %, weather, ...) can be
  overridden by columns in the scenario CSV file (see docs/SCENARIOS.md).
  The values below are used when a column is missing or a cell is blank.
* Anywhere a *distribution* is expected you may write a fixed number
  (``31.0``) or a distribution spec string such as ``"uniform:30,34"``.
  See autosim/distributions.py for the full syntax.

Sections
--------
  1. CARLA connection
  2. Files and folders
  3. Experiment flow (order, count, waiting time)
  4. Simulated communication link (video delay, input delay, frame rate)
  5. Default vehicle status shown to the operator
  6. Environment presets (sun, weather, lights)
  7. Ego vehicle and autonomous driving
  8. Manual control (operator teleoperation)
  9. Cameras
 10. Operator interface: window, layout, colors, fonts, timing
 11. Audio
"""

from typing import Dict, List, Tuple

# =============================================================================
# 1. CARLA CONNECTION
# =============================================================================

CARLA_HOST: str = "127.0.0.1"          # machine running the CARLA server
CARLA_PORT: int = 2000                 # CARLA's default RPC port
CARLA_TIMEOUT_SECONDS: float = 30.0    # give up on a server call after this

# Path to the CARLA "PythonAPI/carla" folder that contains the ``agents``
# package (BasicAgent, route planner).  Leave empty to use the CARLA_ROOT
# environment variable or the copy bundled in autosim/third_party.
CARLA_AGENTS_PATH: str = ""


# =============================================================================
# 2. FILES AND FOLDERS
# =============================================================================

# Scenario battery used when --scenarios is not given (relative to the
# current working directory or to the repository root).
DEFAULT_SCENARIO_FILE: str = "scenarios/example_battery.csv"

# Folder where session data (event log, scenario summary) is written.
DEFAULT_OUTPUT_FOLDER: str = "data"

# Participant identifier used in output folder names when none is given.
DEFAULT_PARTICIPANT_ID: str = "P000"


# =============================================================================
# 3. EXPERIMENT FLOW
# =============================================================================

# "in_order"  -> rows in file order
# "random"    -> shuffled (see DEFAULT_WITH_REPLACEMENT)
DEFAULT_SCENARIO_ORDER: str = "in_order"

# Only used when the order is "random".  False = each scenario at most once
# per pass through the file; True = every pick is independent.
DEFAULT_WITH_REPLACEMENT: bool = False

# Number of scenarios to run.  0 = run every row once.  A number larger than
# the number of rows loops through the file again.
DEFAULT_SCENARIO_COUNT: int = 0

# Waiting time between the end of one scenario ("End supervision") and the
# start of the next.  Kinds: deterministic, uniform, normal, exponential,
# clipped_exponential.  For uniform, SPREAD is the half-width; for normal,
# SPREAD is the standard deviation.  MIN/MAX are used by clipped_exponential
# (and clip every other kind as well).
DEFAULT_WAIT_SECONDS: float = 5.0
DEFAULT_WAIT_DISTRIBUTION: str = "deterministic"
DEFAULT_WAIT_SPREAD_SECONDS: float = 0.0
DEFAULT_WAIT_MIN_SECONDS: float = 0.0
DEFAULT_WAIT_MAX_SECONDS: float = 600.0



# =============================================================================
# 4. SIMULATED COMMUNICATION LINK
# =============================================================================
# The operator sees the world through a (simulated) network link.  Delays are
# drawn from a distribution for every frame / every command, using the same
# distribution kinds as the waiting time above.

DEFAULT_VIDEO_DELAY_SECONDS: float = 0.0
DEFAULT_VIDEO_DELAY_DISTRIBUTION: str = "deterministic"
DEFAULT_VIDEO_DELAY_SPREAD_SECONDS: float = 0.0

DEFAULT_INPUT_DELAY_SECONDS: float = 0.0
DEFAULT_INPUT_DELAY_DISTRIBUTION: str = "deterministic"
DEFAULT_INPUT_DELAY_SPREAD_SECONDS: float = 0.0

# The emergency stop is delayed like every other command (realistic).  Set to
# True to let it bypass the simulated input delay.
EMERGENCY_STOP_IGNORES_INPUT_DELAY: bool = False

# Frames per second delivered to the operator, and the standard deviation of
# the frame rate (frames per second) to simulate an unsteady stream.
DEFAULT_VIDEO_FPS: float = 20.0
DEFAULT_VIDEO_FPS_VARIABILITY: float = 0.0

# Camera image resolution as a fraction of the on-screen panel size.
# 1.0 = sharp; 0.5 = half resolution (looks blurry when scaled up).
DEFAULT_VIDEO_RESOLUTION_SCALE: float = 1.0


# =============================================================================
# 5. DEFAULT VEHICLE STATUS SHOWN TO THE OPERATOR
# =============================================================================
# Used for every value a scenario does not set.  Numbers may be fixed values
# or distribution specs; strings are shown as-is.  Typical EV values were
# chosen (tire pressures per the lab notes; temperatures from common EV
# operating ranges).  The column name in the scenario CSV is the key.

DEFAULT_VEHICLE_STATUS: Dict[str, object] = {
    # --- route --------------------------------------------------------------
    "route_total_miles":      "uniform:3,15",
    "route_progress_percent": "uniform:0,75",
    # --- powertrain ---------------------------------------------------------
    "battery_percent":        "uniform:40,95",
    "battery_temp_f":         "uniform:70,95",
    "motor_temp_f":           "uniform:120,170",
    "efficiency_percent":     "uniform:80,95",
    # --- tires (psi): front-left, front-right, rear-left, rear-right ----------
    "tire_fl_psi":            "uniform:30,34",
    "tire_fr_psi":            "uniform:30,34",
    "tire_rl_psi":            "uniform:30,34",
    "tire_rr_psi":            "uniform:30,34",
    # --- climate ------------------------------------------------------------
    "cabin_temp_f":           "uniform:68,74",
    "outdoor_temp_f":         "uniform:40,80",
    # --- body ---------------------------------------------------------------
    "windows":                "All up",
    "seatbelts":              "All fastened",
    "passengers":             "1",
    # --- subsystems and sensors ---------------------------------------------
    "software_status":        "Good",
    "detection_status":       "Good",
    "route_planner_status":   "Good",
    "control_module_status":  "Good",
    "camera_front_status":    "Good",
    "camera_rear_status":     "Good",
    "camera_left_status":     "Good",
    "camera_right_status":    "Good",
    "lidar_status":           "Good",
    "radar_status":           "Good",
    "gps_status":             "Good",
    "connection_status":      "Good",
}

# Values outside these ranges are highlighted in the Status panel.
# (minimum, maximum); None means "no limit on that side".
STATUS_NORMAL_RANGES: Dict[str, Tuple[object, object]] = {
    "battery_percent": (20, None),
    "battery_temp_f":  (None, 110),
    "motor_temp_f":    (None, 190),
    "tire_fl_psi":     (29, 36),
    "tire_fr_psi":     (29, 36),
    "tire_rl_psi":     (29, 36),
    "tire_rr_psi":     (29, 36),
}

# Status strings treated as "fine".  Anything else is highlighted.
STATUS_GOOD_WORDS: List[str] = ["good", "ok", "normal", "online", "all up", "all fastened"]


# =============================================================================
# 6. ENVIRONMENT PRESETS
# =============================================================================
# Names used in the scenario CSV columns ``time_of_day``, ``weather`` and
# ``lights``.  Add a preset here and it can be used immediately.

DEFAULT_TIME_OF_DAY: str = "day"
DEFAULT_WEATHER: str = "clear"
DEFAULT_LIGHTS: str = "auto"      # "auto" = street lights on only at night

# Sun position: (altitude in degrees, azimuth in degrees).
SUN_PRESETS: Dict[str, Tuple[float, float]] = {
    "day":    (45.0, 0.0),
    "sunset": (2.0, 270.0),
    "night":  (-90.0, 0.0),
}

# Weather: any subset of carla.WeatherParameters attribute names.
WEATHER_PRESETS: Dict[str, Dict[str, float]] = {
    "clear":     {"cloudiness": 10, "precipitation": 0, "precipitation_deposits": 0,
                  "wind_intensity": 5, "fog_density": 0, "wetness": 0},
    "overcast":  {"cloudiness": 80, "precipitation": 0, "precipitation_deposits": 0,
                  "wind_intensity": 50, "fog_density": 2, "wetness": 10},
    "rain":      {"cloudiness": 100, "precipitation": 80, "precipitation_deposits": 90,
                  "wind_intensity": 100, "fog_density": 7, "wetness": 100},
    "storm":     {"cloudiness": 100, "precipitation": 100, "precipitation_deposits": 100,
                  "wind_intensity": 100, "fog_density": 20, "wetness": 100},
    "heavy_fog": {"cloudiness": 90, "precipitation": 0, "precipitation_deposits": 0,
                  "wind_intensity": 0, "fog_density": 100, "fog_distance": 10,
                  "fog_falloff": 0.7, "wetness": 0},
    "dust_storm": {"cloudiness": 10, "precipitation": 0, "wind_intensity": 5,
                   "fog_density": 0, "wetness": 0, "dust_storm": 100},
}

# Street-light presets: (lights on?, intensity in lumens or None for default).
LIGHT_PRESETS: Dict[str, Tuple[bool, object]] = {
    "off":  (False, None),
    "low":  (True, 20000),
    "high": (True, 100000),
}


# =============================================================================
# 7. EGO VEHICLE AND AUTONOMOUS DRIVING
# =============================================================================

DEFAULT_EGO_BLUEPRINT: str = "vehicle.tesla.model3"
EGO_ROLE_NAME: str = "hero"

# Cruising speed of the automated vehicle.
AUTONOMOUS_TARGET_SPEED_MPH: float = 20.0

# Default route length when a scene gives neither destination nor length.
DEFAULT_ROUTE_LENGTH_METERS: float = 400.0

# Stop for static props (cones, barriers, boxes) in the ego lane that are
# closer than this.  CARLA's BasicAgent only stops for vehicles by itself.
OBSTACLE_DETECTION_DISTANCE_METERS: float = 12.0
OBSTACLE_LANE_HALF_WIDTH_METERS: float = 1.6

# When the operator tells a stopped AV to continue, obstacles within this
# distance ahead count as "checked by the operator" and are no longer avoided.
RESUME_CLEARS_OBSTACLES_WITHIN_METERS: float = 30.0

# Controlled ("soft") stop used by task logic, and the emergency stop.
SOFT_STOP_BRAKE: float = 0.5
EMERGENCY_STOP_BRAKE: float = 1.0

# Pull over: drive slowly this far ahead while shifting to the road edge.
PULL_OVER_SPEED_MPH: float = 6.0
PULL_OVER_DISTANCE_METERS: float = 18.0
PULL_OVER_EDGE_MARGIN_METERS: float = 0.4    # gap kept to the lane edge

# Time budget for a commanded lane change (seconds driving in the old lane
# first, then the maneuver).
LANE_CHANGE_SAME_LANE_SECONDS: float = 0.5
LANE_CHANGE_DURATION_SECONDS: float = 2.0

# Speed below which the vehicle is considered stopped.
STOPPED_SPEED_MPH: float = 0.5

# Spawn a pedestrian beside the car when the route is completed.
SPAWN_PEDESTRIAN_AT_ROUTE_END: bool = True

# Text shown in the Route panel after the route is completed.
ROUTE_COMPLETE_NEXT_CUSTOMER_TEXT: str = "Route complete. Waiting for next customer."
ROUTE_COMPLETE_DEPOT_TEXT: str = "Route complete. Returning to depot."


# =============================================================================
# 8. MANUAL CONTROL (TELEOPERATION)
# =============================================================================

# Manual control can only be taken below this speed.
MANUAL_CONTROL_MAX_ENTRY_SPEED_MPH: float = 5.0

# "Inching": throttle and steering limits while driving manually.
MANUAL_MAX_THROTTLE: float = 0.30
MANUAL_MAX_STEER: float = 0.50
MANUAL_COAST_BRAKE: float = 0.35       # applied when no throttle key is held
MANUAL_SPEED_LIMIT_MPH: float = 10.0   # throttle is cut above this speed


# =============================================================================
# 9. CAMERAS
# =============================================================================
# Mount pose relative to the vehicle centre (metres / degrees, CARLA axes:
# x forward, y right, z up) and horizontal field of view.  The image size is
# computed from the on-screen panel size (times the resolution scale) so the
# picture is never stretched.

CAMERA_MOUNTS: Dict[str, Dict[str, float]] = {
    "front":    {"x": 1.5,  "y": 0.0,  "z": 1.6, "pitch": -5.0, "yaw": 0.0,   "fov": 100.0},
    "left":     {"x": 0.5,  "y": -0.9, "z": 1.4, "pitch": -5.0, "yaw": -90.0, "fov": 90.0},
    "right":    {"x": 0.5,  "y": 0.9,  "z": 1.4, "pitch": -5.0, "yaw": 90.0,  "fov": 90.0},
    "rear":     {"x": -2.2, "y": 0.0,  "z": 1.4, "pitch": -8.0, "yaw": 180.0, "fov": 100.0},
    # Low "reconstructed surround view" rather than a satellite view.
    "top_down": {"x": 0.0,  "y": 0.0,  "z": 14.0, "pitch": -90.0, "yaw": 0.0, "fov": 90.0},
}

# Upper limit on camera image width (pixels) to protect GPU memory.
CAMERA_MAX_IMAGE_WIDTH: int = 1600


# =============================================================================
# 10. OPERATOR INTERFACE
# =============================================================================

WINDOW_TITLE: str = "AUTOSIM - Remote Operator Station"
DEFAULT_WINDOW_SIZE: Tuple[int, int] = (1600, 900)
GUI_FRAMES_PER_SECOND: int = 30

# --- Layout ------------------------------------------------------------------
# Panel rectangles as fractions of the window: (left, top, width, height).
# Move or resize a panel by editing its numbers.  Remove a panel by deleting
# its line.  Based on the lab's interface mockup.
PANEL_LAYOUT: Dict[str, Tuple[float, float, float, float]] = {
    "alerts":         (0.004, 0.008, 0.325, 0.215),
    "decision":       (0.004, 0.231, 0.325, 0.137),
    "speed":          (0.337, 0.008, 0.326, 0.080),
    "camera_top_down": (0.337, 0.096, 0.326, 0.272),
    "status":         (0.671, 0.008, 0.325, 0.360),
    "camera_left":    (0.004, 0.376, 0.253, 0.240),
    "camera_front":   (0.261, 0.376, 0.478, 0.240),
    "camera_right":   (0.743, 0.376, 0.253, 0.240),
    "map":            (0.004, 0.624, 0.325, 0.368),
    "camera_rear":    (0.337, 0.624, 0.326, 0.210),
    "route":          (0.337, 0.842, 0.326, 0.150),
    "actions":        (0.671, 0.624, 0.325, 0.368),
}

# Panel titles (shown in each panel's header).
PANEL_TITLES: Dict[str, str] = {
    "alerts": "Alerts",
    "decision": "Decision",
    "status": "Status",
    "map": "Map",
    "route": "Route Progress",
    "actions": "Actions",
    "camera_front": "Front",
    "camera_left": "Left",
    "camera_right": "Right",
    "camera_rear": "Rear",
    "camera_top_down": "Surround (top-down)",
}

# --- Colors (R, G, B) -------------------------------------------------------
COLORS: Dict[str, Tuple[int, int, int]] = {
    "background":        (18, 24, 33),
    "panel":             (40, 74, 122),
    "panel_border":      (22, 40, 66),
    "panel_title":       (235, 241, 250),
    "text":              (235, 241, 250),
    "text_dim":          (170, 186, 208),
    "text_warning":      (255, 205, 80),
    "text_critical":     (255, 110, 100),
    "camera_placeholder": (28, 36, 48),
    "button":            (62, 102, 156),
    "button_hover":      (82, 126, 186),
    "button_text":       (245, 248, 252),
    "button_disabled":   (60, 70, 84),
    "button_toggled":    (40, 150, 110),
    "button_danger":     (210, 35, 35),
    "button_danger_hover": (240, 60, 60),
    "button_primary":    (235, 125, 30),
    "button_primary_hover": (250, 150, 60),
    "button_yes":        (30, 160, 70),
    "button_no":         (205, 40, 40),
    "progress_back":     (190, 196, 204),
    "progress_fill":     (110, 200, 80),
    "severity_info":     (150, 200, 255),
    "severity_warning":  (255, 205, 80),
    "severity_critical": (255, 110, 100),
    "map_background":    (30, 44, 62),
    "map_road":          (120, 134, 152),
    "map_route":         (80, 200, 255),
    "map_ego":           (255, 200, 40),
    "map_hazard":        (255, 90, 80),
    "banner":            (250, 220, 120),
    "banner_text":       (30, 30, 30),
    "overlay_shade":     (0, 0, 0),
    "dialog":            (32, 58, 98),
    "waiting_text":      (200, 210, 225),
}

# --- Fonts (as a fraction of window height, so text scales) -----------------
FONT_SCALE: Dict[str, float] = {
    "small":  0.017,
    "normal": 0.021,
    "title":  0.025,
    "big":    0.060,     # speed read-out
}

# --- Timing -----------------------------------------------------------------
NOTIFICATION_BANNER_SECONDS: float = 4.0     # 3-5 s per requirements
ACTION_FEEDBACK_SECONDS: float = 3.0         # fading text in the Actions panel
MAP_DEFAULT_METERS_ACROSS: float = 300.0     # initial map zoom
MAP_ZOOM_STEP: float = 1.5                   # factor per zoom click
MAP_MIN_METERS_ACROSS: float = 40.0
MAP_MAX_METERS_ACROSS: float = 3000.0

# Fixed sizes of the GPS read-out (decimal places).
GPS_DECIMALS: int = 4


# =============================================================================
# 11. AUDIO
# =============================================================================

AUDIO_ENABLED: bool = True
AUDIO_SAMPLE_RATE: int = 44100
HORN_DURATION_SECONDS: float = 0.6
HORN_FREQUENCIES_HZ: Tuple[float, float] = (420.0, 500.0)   # dual-tone car horn
CHIME_FREQUENCIES_HZ: Tuple[float, float] = (880.0, 1320.0)
AUDIO_VOLUME: float = 0.5
# Name of the folder (next to the scenario CSV) holding pre-recorded messages.
# Also searched relative to the working directory.
AUDIO_FOLDER: str = "audio"
AUDIO_FILE_EXTENSIONS = (".wav", ".ogg", ".mp3", ".flac")

