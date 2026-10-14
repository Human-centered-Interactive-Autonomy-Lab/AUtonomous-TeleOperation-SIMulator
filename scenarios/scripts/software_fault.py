"""
software_fault.py -- "Unspecified software fault detected" (most frequent
disengagement cause in the literature).

After a few seconds the software status turns to "Restart needed", a
critical alert appears and the AV slows down.  Once the vehicle has pulled
over, a notification offers a software restart.

Optional columns:
    fault_after_s    seconds after the start when the fault appears (default 8)
    fault_text       alert text (default "Unspecified software fault detected")
"""

from autosim.simulation import DrivingMode
from autosim.tasks import TaskScript


class Task(TaskScript):
    def on_start(self, ctx):
        self.fault_after = float(ctx.scenario.extra.get("fault_after_s", 8))
        self.fault_text = ctx.scenario.extra.get("fault_text", "Unspecified software fault detected")
        self.fault_time = None
        self.restart_offered = False

    def on_tick(self, ctx, dt):
        if self.fault_time is None and ctx.elapsed >= self.fault_after:
            self.fault_time = ctx.elapsed
            ctx.set_status("software_status", "Restart needed")
            ctx.set_status("control_module_status", "Fault detected")
            ctx.add_alert(self.fault_text, "critical")
            ctx.set_target_speed(8)
            ctx.log("software_fault_started")

            ctx.record("fault_started_s", round(ctx.elapsed, 2))

    def on_mode_change(self, ctx, old_mode, new_mode):
        if new_mode == DrivingMode.PULLED_OVER and self.fault_time is not None and not self.restart_offered:
            self.restart_offered = True
            ctx.notify("Software restart available", "The vehicle is stopped and can be restarted safely.",
                       "warning", [("Restart software", lambda: self._restart(ctx))])

    def _restart(self, ctx):
        ctx.set_status("software_status", "Good")
        ctx.set_status("control_module_status", "Good")
        ctx.remove_alert(self.fault_text)
        ctx.set_target_speed(ctx.scenario.target_speed_mph)
        ctx.feedback("Software restarted. Press Resume AV to continue.")
        ctx.record("software_restarted_s", round(ctx.elapsed, 2))
