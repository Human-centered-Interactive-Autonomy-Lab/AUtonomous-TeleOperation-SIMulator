import random

import pytest

from autosim.distributions import Choice, make_distribution, parse_spec, sample_value


def test_plain_number_is_deterministic():
    assert parse_spec("5").sample() == 5.0
    assert parse_spec(2.5).sample() == 2.5


@pytest.mark.parametrize("spec", ["uniform:3,15", "uniform:low=3,high=15", "uniform:3;15"])
def test_uniform_bounds(spec):
    rng = random.Random(0)
    values = [parse_spec(spec).sample(rng) for _ in range(500)]
    assert min(values) >= 3 and max(values) <= 15


def test_clipped_exponential_is_clipped():
    rng = random.Random(1)
    distribution = parse_spec("clipped_exponential:mean=20,min=5,max=30")
    values = [distribution.sample(rng) for _ in range(1000)]
    assert min(values) == 5 and max(values) == 30


def test_normal_from_command_line_never_negative():
    rng = random.Random(2)
    distribution = make_distribution("normal", mean=1.0, spread=5.0, minimum=0.0)
    assert min(distribution.sample(rng) for _ in range(500)) >= 0.0


def test_exponential_mean_roughly_right():
    rng = random.Random(3)
    distribution = make_distribution("exponential", mean=10.0)
    values = [distribution.sample(rng) for _ in range(20000)]
    assert 9.5 < sum(values) / len(values) < 10.5


def test_choice_and_text_values():
    assert isinstance(parse_spec("choice:Good,Degraded"), Choice)
    assert sample_value("Good") == "Good"
    assert sample_value("31") == 31.0


@pytest.mark.parametrize("bad", ["uniform:15,3", "bogus:1", "normal:1", "uniform:a,b", "hello"])
def test_bad_specs_raise(bad):
    with pytest.raises(ValueError):
        parse_spec(bad)
