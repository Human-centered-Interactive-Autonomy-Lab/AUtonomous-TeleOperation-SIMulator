"""Test setup: run pygame without a screen or sound card."""

import os
import sys

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

REPOSITORY = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLE_BATTERY = os.path.join(REPOSITORY, "scenarios", "example_battery.csv")
