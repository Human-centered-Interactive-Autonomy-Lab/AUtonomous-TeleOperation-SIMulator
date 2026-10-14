"""
distributions.py -- random distributions used for timing and default values.

A distribution can be written as a short text "spec", which is how it
appears in settings.py and in scenario CSV cells:

    "5"                                   always 5 (deterministic)
    "deterministic:5"   or "fixed:5"      always 5
    "uniform:3,15"                        uniform between 3 and 15
    "normal:10,2"                         normal, mean 10, standard deviation 2
    "exponential:20"                      exponential with mean 20
    "clipped_exponential:20,5,60"         exponential, mean 20, clipped to [5, 60]
    "choice:Good,Degraded,Fault"          one of the listed values, equally likely

Arguments can also be named, in any order:  "uniform:low=3,high=15",
"normal:mean=10,sd=2", "clipped_exponential:mean=20,min=5,max=60".
Arguments may be separated by ";" instead of "," (handy in hand-written CSV
files, where a comma would start a new column): "uniform:3;15".

Command-line options use ``make_distribution(kind, mean, spread, min, max)``
instead, so users can type ``--wait-time 20 --wait-distribution exponential``.
"""

import math
import random
from typing import Any, Dict, List, Optional, Sequence

DISTRIBUTION_KINDS = (
    "deterministic",
    "uniform",
    "normal",
    "exponential",
    "clipped_exponential",
)

# Positional argument names for each kind, so "uniform:3,15" == "uniform:low=3,high=15".
_ARGUMENT_NAMES: Dict[str, Sequence[str]] = {
    "deterministic": ("value",),
    "fixed": ("value",),
    "uniform": ("low", "high"),
    "normal": ("mean", "sd"),
    "exponential": ("mean",),
    "clipped_exponential": ("mean", "min", "max"),
}


class Distribution:
    """A random number source.  ``sample(rng)`` returns one value.

    ``minimum`` / ``maximum`` (optional) clip every sample; they are also how
    the normal distribution is kept non-negative for times.
    """

    def __init__(
        self,
        kind: str,
        parameters: Dict[str, float],
        minimum: Optional[float] = None,
        maximum: Optional[float] = None,
    ) -> None:
        if kind == "fixed":
            kind = "deterministic"
        if kind not in DISTRIBUTION_KINDS:
            raise ValueError(
                "Unknown distribution '{}'. Use one of: {}".format(kind, ", ".join(DISTRIBUTION_KINDS))
            )
        self.kind = kind
        self.parameters = dict(parameters)
        self.minimum = minimum
        self.maximum = maximum
        self._validate()

    # --------------------------------------------------------------------- API

    def sample(self, rng: Optional[random.Random] = None) -> float:
        """Draw one value."""
        rng = rng or random
        p = self.parameters
        if self.kind == "deterministic":
            value = p["value"]
        elif self.kind == "uniform":
            value = rng.uniform(p["low"], p["high"])
        elif self.kind == "normal":
            value = rng.gauss(p["mean"], p["sd"])
        elif self.kind in ("exponential", "clipped_exponential"):
            value = rng.expovariate(1.0 / p["mean"]) if p["mean"] > 0 else 0.0
        else:  # pragma: no cover - guarded by _validate
            raise AssertionError(self.kind)
        return self._clip(value)

    @property
    def mean(self) -> float:
        """Nominal mean (before clipping) -- used for display and logging."""
        p = self.parameters
        if self.kind == "deterministic":
            return p["value"]
        if self.kind == "uniform":
            return (p["low"] + p["high"]) / 2.0
        return p["mean"]

    def describe(self) -> str:
        """Human-readable spec, e.g. 'uniform(low=3, high=15)'."""
        arguments = ", ".join("{}={:g}".format(k, v) for k, v in self.parameters.items())
        text = "{}({})".format(self.kind, arguments)
        if self.kind != "clipped_exponential" and (self.minimum is not None or self.maximum is not None):
            text += " clipped to [{}, {}]".format(
                "-inf" if self.minimum is None else "{:g}".format(self.minimum),
                "inf" if self.maximum is None else "{:g}".format(self.maximum),
            )
        return text

    def __repr__(self) -> str:
        return "Distribution<{}>".format(self.describe())

    # ---------------------------------------------------------------- helpers

    def _validate(self) -> None:
        required = _ARGUMENT_NAMES[self.kind]
        missing = [name for name in required if name not in self.parameters]
        if missing:
            raise ValueError("Distribution '{}' needs: {}".format(self.kind, ", ".join(missing)))
        if self.kind == "uniform" and self.parameters["low"] > self.parameters["high"]:
            raise ValueError("uniform: low must not exceed high")
        if self.kind == "normal" and self.parameters["sd"] < 0:
            raise ValueError("normal: sd must be >= 0")
        if self.kind in ("exponential", "clipped_exponential") and self.parameters["mean"] < 0:
            raise ValueError("exponential: mean must be >= 0")
        if self.kind == "clipped_exponential":
            self.minimum = self.parameters["min"]
            self.maximum = self.parameters["max"]
            if self.minimum > self.maximum:
                raise ValueError("clipped_exponential: min must not exceed max")

    def _clip(self, value: float) -> float:
        if self.minimum is not None:
            value = max(self.minimum, value)
        if self.maximum is not None:
            value = min(self.maximum, value)
        return value


class Choice:
    """Pick one of several values (numbers or text) with equal probability."""

    def __init__(self, options: List[Any]) -> None:
        if not options:
            raise ValueError("choice: needs at least one option")
        self.options = list(options)

    def sample(self, rng: Optional[random.Random] = None) -> Any:
        return (rng or random).choice(self.options)

    def describe(self) -> str:
        return "choice({})".format(", ".join(str(o) for o in self.options))


def deterministic(value: float) -> Distribution:
    """Shortcut for a distribution that always returns ``value``."""
    return Distribution("deterministic", {"value": float(value)})


def make_distribution(
    kind: str,
    mean: float,
    spread: float = 0.0,
    minimum: Optional[float] = 0.0,
    maximum: Optional[float] = None,
) -> Distribution:
    """Build a distribution from the command-line style arguments.

    ``spread`` is the half-width for uniform and the standard deviation for
    normal; it is ignored by the other kinds.  Every sample is clipped to
    [minimum, maximum] (times must not be negative).
    """
    kind = kind.lower().strip()
    if kind in ("deterministic", "fixed", "constant"):
        return Distribution("deterministic", {"value": mean}, minimum, maximum)
    if kind == "uniform":
        return Distribution("uniform", {"low": mean - spread, "high": mean + spread}, minimum, maximum)
    if kind == "normal":
        return Distribution("normal", {"mean": mean, "sd": spread}, minimum, maximum)
    if kind == "exponential":
        return Distribution("exponential", {"mean": mean}, minimum, maximum)
    if kind in ("clipped_exponential", "clipped-exponential"):
        low = 0.0 if minimum is None else minimum
        high = math.inf if maximum is None else maximum
        return Distribution("clipped_exponential", {"mean": mean, "min": low, "max": high})
    raise ValueError("Unknown distribution '{}'. Use one of: {}".format(kind, ", ".join(DISTRIBUTION_KINDS)))


def parse_spec(text: Any):
    """Turn a spec (see module docstring) into a Distribution or Choice.

    Plain numbers become deterministic distributions.  Raises ValueError on a
    malformed spec.  Non-string, non-number input is returned unchanged.
    """
    if isinstance(text, (int, float)) and not isinstance(text, bool):
        return deterministic(float(text))
    if not isinstance(text, str):
        raise ValueError("Cannot parse distribution from {!r}".format(text))
    text = text.strip()
    try:
        return deterministic(float(text))
    except ValueError:
        pass
    if ":" not in text:
        raise ValueError("Not a number or distribution spec: '{}'".format(text))
    kind, _, argument_text = text.partition(":")
    kind = kind.strip().lower()
    pieces = [piece.strip() for piece in argument_text.replace(";", ",").split(",") if piece.strip()]
    if kind == "choice":
        return Choice([_number_or_text(piece) for piece in pieces])
    if kind not in _ARGUMENT_NAMES:
        raise ValueError("Unknown distribution '{}' in '{}'".format(kind, text))
    names = _ARGUMENT_NAMES[kind]
    parameters: Dict[str, float] = {}
    for position, piece in enumerate(pieces):
        if "=" in piece:
            name, _, value = piece.partition("=")
            name = name.strip().lower()
        else:
            if position >= len(names):
                raise ValueError("Too many arguments in '{}'".format(text))
            name, value = names[position], piece
        aliases = {"sd": "sd", "std": "sd", "stdev": "sd", "minimum": "min", "maximum": "max"}
        name = aliases.get(name, name)
        try:
            parameters[name] = float(value)
        except ValueError:
            raise ValueError("Argument '{}' in '{}' is not a number".format(piece, text))
    return Distribution(kind, parameters)


def is_spec(text: Any) -> bool:
    """True if ``text`` looks like a distribution spec ('kind:...')."""
    if not isinstance(text, str) or ":" not in text:
        return False
    kind = text.split(":", 1)[0].strip().lower()
    return kind in _ARGUMENT_NAMES or kind == "choice"


def sample_value(value: Any, rng: Optional[random.Random] = None) -> Any:
    """Resolve a settings/CSV value: specs are sampled, everything else is returned.

    Numeric strings become floats.  Other strings are returned as text.
    """
    if isinstance(value, (Distribution, Choice)):
        return value.sample(rng)
    if is_spec(value):
        return parse_spec(value).sample(rng)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            return value
    return value


def _number_or_text(piece: str) -> Any:
    try:
        return float(piece)
    except ValueError:
        return piece
