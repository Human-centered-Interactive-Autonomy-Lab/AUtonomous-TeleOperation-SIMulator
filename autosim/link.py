"""
link.py -- simulated network link between the vehicle and the operator.

Real remote operators see delayed, sometimes choppy video, and their
commands reach the vehicle late.  This module reproduces that:

  VideoLink    Holds camera frames back by a (random) delay and thins them
               out to a target frame rate with optional jitter.
  CommandLink  Holds operator commands back by a (random) input delay
               before they are executed on the vehicle.

Both work on plain timestamps (seconds, ``time.monotonic()`` style), so they
can be unit-tested without CARLA or pygame.
"""

import random
from collections import deque
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

from autosim.distributions import Distribution, deterministic


class _CameraChannel:
    """Delay buffer for one camera."""

    def __init__(self) -> None:
        # (time the frame becomes visible, frame)
        self.pending: Deque[Tuple[float, Any]] = deque()
        self.visible_frame: Any = None
        self.last_release_time: float = float("-inf")
        self.next_accept_time: float = float("-inf")


class VideoLink:
    """Delays camera frames and limits their rate.

    Parameters
    ----------
    delay        Distribution of the video delay in seconds (sampled per frame).
    fps          Target frames per second delivered to the operator.
    fps_sd       Standard deviation of the frame rate (frames per second).
    """

    MAX_BUFFERED_FRAMES = 200   # safety cap (about 10 s at 20 fps)

    def __init__(
        self,
        delay: Optional[Distribution] = None,
        fps: float = 20.0,
        fps_sd: float = 0.0,
        rng: Optional[random.Random] = None,
    ) -> None:
        self.delay = delay or deterministic(0.0)
        self.fps = max(0.1, float(fps))
        self.fps_sd = max(0.0, float(fps_sd))
        self.rng = rng or random.Random()
        self._channels: Dict[str, _CameraChannel] = {}

    def reset(self) -> None:
        """Forget all frames (call between scenarios)."""
        self._channels.clear()

    def push(self, camera: str, capture_time: float, frame: Any) -> None:
        """Offer a new frame captured at ``capture_time``.

        Frames arriving faster than the (jittered) frame rate are dropped.
        Each kept frame gets its own delay, but frames are never shown out of
        order: a frame is released no earlier than the one before it.
        """
        channel = self._channels.setdefault(camera, _CameraChannel())
        if capture_time < channel.next_accept_time:
            return
        channel.next_accept_time = capture_time + self._next_frame_interval()
        release_time = max(capture_time + self.delay.sample(self.rng), channel.last_release_time)
        channel.last_release_time = release_time
        channel.pending.append((release_time, frame))
        while len(channel.pending) > self.MAX_BUFFERED_FRAMES:
            channel.pending.popleft()

    def frame(self, camera: str, now: float) -> Any:
        """The frame the operator should be seeing at time ``now`` (or None)."""
        channel = self._channels.get(camera)
        if channel is None:
            return None
        while channel.pending and channel.pending[0][0] <= now:
            channel.visible_frame = channel.pending.popleft()[1]
        return channel.visible_frame

    def _next_frame_interval(self) -> float:
        fps = self.fps if self.fps_sd == 0 else self.rng.gauss(self.fps, self.fps_sd)
        fps = max(0.5, fps)   # never wait more than 2 s for a frame
        # Accept slightly early so a camera running at exactly fps is not
        # dropped by floating-point noise.
        return 0.98 / fps


class CommandLink:
    """Delays operator commands before they reach the vehicle.

    ``send(name, function)`` schedules ``function()`` to run after a delay
    drawn from the input-delay distribution.  ``update(now)`` must be called
    every frame; it runs every command whose time has come, in order.

    Continuous inputs (manual driving keys) use ``send_continuous``, which
    keeps only the newest value per channel once it is due.
    """

    def __init__(self, delay: Optional[Distribution] = None, rng: Optional[random.Random] = None) -> None:
        self.delay = delay or deterministic(0.0)
        self.rng = rng or random.Random()
        self._queue: List[Tuple[float, int, str, Callable[[], None]]] = []
        self._counter = 0
        self._continuous: Dict[str, Deque[Tuple[float, Any]]] = {}
        self._continuous_current: Dict[str, Any] = {}
        self._last_release = float("-inf")

    def reset(self) -> None:
        self._queue = []
        self._continuous = {}
        self._continuous_current = {}
        self._last_release = float("-inf")

    def send(self, name: str, function: Callable[[], None], now: float, immediate: bool = False) -> float:
        """Schedule a command.  Returns the time it will execute."""
        if immediate:
            release = now
        else:
            # Commands keep their order even if a later one draws a shorter delay.
            release = max(now + self.delay.sample(self.rng), self._last_release)
            self._last_release = release
        self._counter += 1
        self._queue.append((release, self._counter, name, function))
        self._queue.sort(key=lambda item: (item[0], item[1]))
        return release

    def send_continuous(self, channel: str, value: Any, now: float) -> None:
        """Queue the newest value of a continuous input (e.g. throttle/steer)."""
        buffer = self._continuous.setdefault(channel, deque())
        release = now + self.delay.sample(self.rng)
        if buffer and release < buffer[-1][0]:
            release = buffer[-1][0]
        buffer.append((release, value))

    def update(self, now: float) -> List[str]:
        """Run every due command.  Returns the names of commands that ran."""
        ran = []
        while self._queue and self._queue[0][0] <= now:
            _, _, name, function = self._queue.pop(0)
            function()
            ran.append(name)
        return ran

    def continuous_value(self, channel: str, now: float) -> Any:
        """Newest due value of a continuous input (or None if nothing is due yet)."""
        buffer = self._continuous.get(channel)
        while buffer and buffer[0][0] <= now:
            self._continuous_current[channel] = buffer.popleft()[1]
        return self._continuous_current.get(channel)

    @property
    def pending_count(self) -> int:
        return len(self._queue)
