from autosim.distributions import deterministic, parse_spec
from autosim.link import CommandLink, VideoLink


def test_video_delay_holds_frames_back():
    link = VideoLink(delay=deterministic(0.5), fps=100)
    link.push("front", 10.0, "frame A")
    assert link.frame("front", 10.4) is None
    assert link.frame("front", 10.5) == "frame A"


def test_video_frames_never_go_backwards_with_jitter():
    import random
    link = VideoLink(delay=parse_spec("uniform:0,1"), fps=1000, rng=random.Random(0))
    for index in range(50):
        link.push("front", index * 0.01, index)
    shown = [link.frame("front", t / 10.0) for t in range(0, 30)]
    numbers = [n for n in shown if n is not None]
    assert numbers == sorted(numbers)


def test_video_frame_rate_limit():
    link = VideoLink(fps=10)
    for index in range(100):          # 100 frames in one second
        link.push("front", index / 100.0, index)
    kept = len(link._channels["front"].pending)
    assert 9 <= kept <= 11


def test_commands_wait_for_input_delay_and_keep_order():
    link = CommandLink(delay=deterministic(0.3))
    ran = []
    link.send("a", lambda: ran.append("a"), now=0.0)
    link.send("b", lambda: ran.append("b"), now=0.1)
    assert link.update(0.29) == []
    assert link.update(0.31) == ["a"]
    assert link.update(1.0) == ["b"] and ran == ["a", "b"]


def test_immediate_command_skips_delay():
    link = CommandLink(delay=deterministic(5.0))
    link.send("stop", lambda: None, now=0.0, immediate=True)
    assert link.update(0.0) == ["stop"]


def test_continuous_input_is_delayed():
    link = CommandLink(delay=deterministic(0.2))
    link.send_continuous("manual", (1, 0), now=0.0)
    assert link.continuous_value("manual", 0.1) is None
    assert link.continuous_value("manual", 0.25) == (1, 0)
    assert link.continuous_value("manual", 0.3) == (1, 0)   # held until a newer value is due
