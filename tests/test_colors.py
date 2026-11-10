"""Every color theme must let every operator see every button and read every
label: WCAG 2.1 AA contrast (4.5:1 for text, 3:1 for the edges of buttons,
switches and other graphics)."""

import pytest

from autosim import settings

TEXT, GRAPHIC = 4.5, 3.0

# (what, foreground, background, minimum ratio)
PAIRS = [
    ("panel text", "text", "panel", TEXT),
    ("secondary text", "text_dim", "panel", TEXT),
    ("panel titles", "panel_title", "panel", TEXT),
    ("warning text", "text_warning", "panel", TEXT),
    ("critical text", "text_critical", "panel", TEXT),
    ("alert row text", "text", "row_hover", TEXT),
    ("button labels", "button_text", "button", TEXT),
    ("button labels on hover", "button_text", "button_hover", TEXT),
    ("dialog text", "text", "dialog", TEXT),
    ("button outline", "button_border", "panel", GRAPHIC),
    ("panel outline", "panel_border", "background", 1.5),   # panels also differ from it in fill
    ("switch off", "switch_off", "button", GRAPHIC),
    ("switch on", "switch_on", "button", GRAPHIC),
    ("route on map", "map_route", "map_background", GRAPHIC),
    ("roads on map", "map_road", "map_background", GRAPHIC),
    ("progress bar", "progress_fill", "progress_back", GRAPHIC),
    ("alert stripe (info)", "severity_info", "row_hover", GRAPHIC),
    ("alert stripe (warning)", "severity_warning", "row_hover", GRAPHIC),
    ("alert stripe (critical)", "severity_critical", "row_hover", GRAPHIC),
    ("notification banner", "banner_text", "banner", TEXT),
    ("feedback messages", "toast_text", "toast", TEXT),
] + [("white on " + key, "text_on_color", key, TEXT) for key in (
    "button_danger", "button_danger_hover", "button_primary", "button_primary_hover", "button_toggled",
    "button_yes", "button_no", "badge_normal", "badge_warning", "badge_critical", "badge_neutral")]


def luminance(color):
    def channel(value):
        value /= 255.0
        return value / 12.92 if value <= 0.03928 else ((value + 0.055) / 1.055) ** 2.4
    red, green, blue = color
    return 0.2126 * channel(red) + 0.7152 * channel(green) + 0.0722 * channel(blue)


def contrast(first, second):
    lighter, darker = sorted((luminance(first), luminance(second)), reverse=True)
    return (lighter + 0.05) / (darker + 0.05)


def test_contrast_formula():
    assert contrast((0, 0, 0), (255, 255, 255)) == pytest.approx(21.0)
    assert contrast((118, 118, 118), (255, 255, 255)) == pytest.approx(4.54, abs=0.02)


@pytest.mark.parametrize("theme", sorted(settings._THEMES))
def test_theme_meets_wcag_aa(theme):
    palette = dict(settings._COMMON_COLORS, **settings._THEMES[theme])
    failures = ["{} {:.1f}:1 (needs {}:1)".format(what, contrast(palette[fg], palette[bg]), need)
                for what, fg, bg, need in PAIRS if contrast(palette[fg], palette[bg]) < need]
    assert not failures, "{}: {}".format(theme, "; ".join(failures))


def test_default_theme_is_high_contrast():
    assert settings.THEME == "high_contrast"
    assert settings.COLORS["panel"] == settings._THEMES["high_contrast"]["panel"]
