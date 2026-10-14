"""Shared test setup."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

REPOSITORY = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLE_BATTERY = os.path.join(REPOSITORY, "scenarios", "example_battery.csv")
