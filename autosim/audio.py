"""
audio.py -- sounds: car horn, notification chime, and audio files.

The horn and chime are synthesised with numpy, so no sound files are needed.
Pre-recorded messages are found by messages.py and played with play_file().
If the computer has no audio device, every call silently does nothing.
"""

import os
from typing import Dict, Tuple

import numpy
import pygame

from autosim import settings


class Audio:
    def __init__(self, enabled: bool = settings.AUDIO_ENABLED) -> None:
        self.enabled = False
        self._sounds: Dict[str, "pygame.mixer.Sound"] = {}
        if not enabled:
            return
        try:
            pygame.mixer.init(frequency=settings.AUDIO_SAMPLE_RATE, size=-16, channels=2, buffer=512)
            self._sounds["horn"] = self._tone(settings.HORN_FREQUENCIES_HZ, settings.HORN_DURATION_SECONDS, harmonics=True)
            self._sounds["chime"] = self._chime(settings.CHIME_FREQUENCIES_HZ)
            self.enabled = True
        except (pygame.error, NotImplementedError) as error:
            print("[audio] Sound disabled ({}).".format(error))

    def play(self, name: str) -> None:
        """Play a built-in sound: 'horn' or 'chime'."""
        sound = self._sounds.get(name)
        if self.enabled and sound is not None:
            sound.play()

    def play_file(self, path: str) -> bool:
        """Play an audio file (full path, e.g. from resolve_message)."""
        if not self.enabled or not path:
            return False
        if not os.path.isfile(path):
            print("[audio] File not found: {}".format(path))
            return False
        try:
            pygame.mixer.Sound(path).play()
            return True
        except pygame.error as error:
            print("[audio] Cannot play {}: {}".format(path, error))
            return False

    # ------------------------------------------------------------ synthesis

    def _tone(self, frequencies: Tuple[float, ...], seconds: float, harmonics: bool = False) -> "pygame.mixer.Sound":
        """Sum of tones with a short attack/release envelope (a dual-tone horn)."""
        rate = settings.AUDIO_SAMPLE_RATE
        t = numpy.linspace(0, seconds, int(rate * seconds), endpoint=False)
        wave = numpy.zeros_like(t)
        for frequency in frequencies:
            wave += numpy.sign(numpy.sin(2 * numpy.pi * frequency * t)) * 0.3   # square-ish = brassy
            if harmonics:
                wave += 0.25 * numpy.sin(2 * numpy.pi * 2 * frequency * t)
        envelope = numpy.minimum(1.0, numpy.minimum(t / 0.02, (seconds - t) / 0.05))
        return self._to_sound(wave * envelope)

    def _chime(self, frequencies: Tuple[float, float]) -> "pygame.mixer.Sound":
        rate = settings.AUDIO_SAMPLE_RATE
        parts = []
        for frequency in frequencies:
            t = numpy.linspace(0, 0.18, int(rate * 0.18), endpoint=False)
            parts.append(numpy.sin(2 * numpy.pi * frequency * t) * numpy.exp(-t * 14))
        return self._to_sound(numpy.concatenate(parts))

    def _to_sound(self, wave: "numpy.ndarray") -> "pygame.mixer.Sound":
        peak = max(1e-9, float(numpy.max(numpy.abs(wave))))
        samples = (wave / peak * settings.AUDIO_VOLUME * 32767).astype(numpy.int16)
        stereo = numpy.ascontiguousarray(numpy.column_stack([samples, samples]))
        return pygame.sndarray.make_sound(stereo)

