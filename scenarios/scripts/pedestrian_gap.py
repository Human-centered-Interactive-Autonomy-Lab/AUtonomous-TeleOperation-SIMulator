"""
pedestrian_gap.py -- "Heavy pedestrian traffic, please advise" (gap acceptance).

The AV stops before a crosswalk.  Pedestrians cross in front of it, one after
another, leaving gaps of a few seconds between them.  The operator presses
"Clear to go" when they judge there is a big enough gap.  The task records
how much time was actually available at that moment (whether that counts as
a good decision is up to the experimenter).

Optional columns in the scenario CSV:
    crossing_gaps_s     gaps between pedestrians, e.g. "3|3|6|3|6"  (default)
    crossing_speed_mps  walking speed (default 1.4)
The crosswalk position is the scene trigger "crosswalk" (or 50 m).
"""

from autosim.tasks import TaskScript

SIDEWALK_OFFSET_M = 9.0       # pedestrians start/end this far left/right of the ego lane centre
LANE_HALF_WIDTH_M = 2.5       # part of the road the car needs (ego lane plus margin)
STOP_BEFORE_CROSSWALK_M = 9.0 # the car's centre stops this far before the crosswalk


class Task(TaskScript):
    def on_start(self, ctx):
        extra = ctx.scenario.extra
        self.gaps = [float(g) for g in extra.get("crossing_gaps_s", "3|3|6|3|6").split("|") if g.strip()]
        self.speed = float(extra.get("crossing_speed_mps", 1.4))
        self.crosswalk_m = ctx.trigger_route_m("crosswalk") or 50.0
        # Time a pedestrian needs to walk from the sidewalk into the lane, and across it.
        self.time_to_lane = (SIDEWALK_OFFSET_M - LANE_HALF_WIDTH_M) / self.speed
        self.time_in_lane = 2 * LANE_HALF_WIDTH_M / self.speed
        self.walkers = []            # (time the walker enters the lane, leaves the lane)
        self.next_spawn_time = 0.0
        self.gap_index = 0
        self.stopped = False
        self.answered = False

    def on_tick(self, ctx, dt):
        if not self.stopped and ctx.distance_to(self.crosswalk_m - STOP_BEFORE_CROSSWALK_M) <= ctx.state.speed_mps ** 2 / 8 + 1:
            self.stopped = True
            ctx.stop_vehicle("Heavy pedestrian traffic")
            ctx.add_alert("Heavy pedestrian traffic, please advise", "warning")
            ctx.ask("Heavy pedestrian traffic. Tell me when it is clear to go.",
                    ["Clear to go=continue"])
            self.next_spawn_time = ctx.elapsed
        if self.stopped and not self.answered and ctx.elapsed >= self.next_spawn_time:
            self._spawn_pedestrian(ctx)

    def _spawn_pedestrian(self, ctx):
        side = 1 if self.gap_index % 2 == 0 else -1          # alternate directions
        ctx.spawn_walker({"route_m": self.crosswalk_m, "lateral_m": -side * SIDEWALK_OFFSET_M},
                         {"route_m": self.crosswalk_m, "lateral_m": side * SIDEWALK_OFFSET_M},
                         speed_mps=self.speed)
        enters = ctx.elapsed + self.time_to_lane
        self.walkers.append((enters, enters + self.time_in_lane))
        gap = self.gaps[self.gap_index % len(self.gaps)]
        self.gap_index += 1
        # The next pedestrian enters the lane `gap` seconds after this one leaves it.
        self.next_spawn_time = ctx.elapsed + self.time_in_lane + gap

    def on_answer(self, ctx, label):
        self.answered = True
        now = ctx.elapsed
        occupied = any(enter <= now <= leave for enter, leave in self.walkers)
        upcoming = [enter - now for enter, _ in self.walkers if enter > now]
        # The next pedestrian would have been spawned at next_spawn_time.
        upcoming.append(self.next_spawn_time + self.time_to_lane - now)
        available = 0.0 if occupied else min(upcoming)
        ctx.record("gap_available_s", round(available, 2))
        ctx.record("lane_occupied_at_go", occupied)
        ctx.record("pedestrians_before_go", self.gap_index)
        ctx.log("gap_decision", available_s=round(available, 2), lane_occupied=occupied)
