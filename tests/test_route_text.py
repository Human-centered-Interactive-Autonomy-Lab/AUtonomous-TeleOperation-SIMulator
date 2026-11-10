"""Plain-words remaining time and distance in the Route panel."""

from autosim.interface import distance_left_text, time_left_text


def test_time_left():
    assert time_left_text(5.4) == "About 5 min left"
    assert time_left_text(1.6) == "About 2 min left"
    assert time_left_text(0.5) == "Under 1 min left"
    assert time_left_text(0.1) == "Arriving"
    assert time_left_text(-1.0) == "Arriving"


def test_distance_left():
    assert distance_left_text(0.16) == "850 ft to go"         # 845 ft, to the nearest 50
    assert distance_left_text(0.05) == "250 ft to go"
    assert distance_left_text(0.0) == "0 ft to go"
    assert distance_left_text(0.24) == "0.2 mi to go"
    assert distance_left_text(1.26) == "1.3 mi to go"
