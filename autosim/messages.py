"""
messages.py -- pre-recorded messages (passenger requests, replies, ...).

Messages live in the audio folder next to the scenario CSV (scenarios/audio).
A message cell in the scenario file is resolved like this:

    "I feel carsick.wav"   an audio file  -> played (its transcript is shown if
                                             a .txt with the same name exists)
    "I feel carsick.txt"   a text file    -> its text is shown
    "Can we stop, please?" anything else  -> shown as typed
"""

import os
from typing import Optional, Tuple

from autosim import settings

MESSAGE_FILE_EXTENSIONS = tuple(settings.AUDIO_FILE_EXTENSIONS) + (".txt",)


def looks_like_file(message: str) -> bool:
    """True if the cell names a message file (by its extension)."""
    return os.path.splitext(message.strip())[1].lower() in MESSAGE_FILE_EXTENSIONS


def find_message_file(name: str, scenario_folder: str = "") -> Optional[str]:
    """Look for ``name`` in the audio folder next to the scenario file, the
    audio folder in the working directory, and as a path."""
    if not name or len(name) > 255:
        return None
    candidates = [
        os.path.join(scenario_folder, settings.AUDIO_FOLDER, name) if scenario_folder else "",
        os.path.join("scenarios", settings.AUDIO_FOLDER, name),
        os.path.join(settings.AUDIO_FOLDER, name),
        name,
    ]
    for candidate in candidates:
        if candidate and os.path.isfile(candidate):
            return os.path.abspath(candidate)
    return None


def resolve_message(message: str, scenario_folder: str = "") -> Tuple[str, str]:
    """Return (text to show, audio file to play or "") for a message cell."""
    path = find_message_file(message.strip(), scenario_folder)
    if path is None:
        return message, ""
    base, extension = os.path.splitext(path)
    if extension.lower() == ".txt":
        return _read_text(path), ""
    if extension.lower() in settings.AUDIO_FILE_EXTENSIONS:
        transcript = base + ".txt"
        return (_read_text(transcript) if os.path.isfile(transcript) else ""), path
    return message, ""


def _read_text(path: str) -> str:
    with open(path, "r", encoding="utf-8-sig") as handle:
        return " ".join(handle.read().split())
