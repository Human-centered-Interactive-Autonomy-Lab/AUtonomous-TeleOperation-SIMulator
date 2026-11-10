"""Route panel text: "2 of 13 min" and "0.7 of 5.4 mi"."""

from autosim.interface import miles_text, minutes_text


def test_minutes():
    assert minutes_text(2.2, 12.8) == "2 of 13 min"
    assert minutes_text(0.2, 1.1) == "0 of 1 min"
    assert minutes_text(1.6, 1.4) == "1 of 1 min"       # never more done than the total


def test_miles():
    assert miles_text(0.71, 5.38) == "0.7 of 5.4 mi"
    assert miles_text(0.249, 0.249) == "0.2 of 0.2 mi"  # trip complete: both the same
    assert miles_text(0.26, 0.24) == "0.2 of 0.2 mi"
    assert miles_text(-0.01, 0.25) == "0.0 of 0.2 mi"
