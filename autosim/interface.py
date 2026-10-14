"""
interface.py -- the operator interface (pygame window).

How it is organised
-------------------
* Panel positions come from settings.PANEL_LAYOUT (fractions of the window),
  so the layout follows any window size and is changed by editing numbers.
* Buttons come from actions.py.  This file only decides how they look.
* "Immediate mode": every frame, ``draw()`` paints all panels from the
  current ScenarioSession state and records the clickable areas
  ("hotspots") it painted.  Mouse events are matched against those
  hotspots.  There is no hidden widget state to keep in sync.

Panels: speed, alerts (+ notification bell), decision, status, five camera
views, map, route progress, actions.  Overlays: notification banner,
dialogs, notification list, detailed status.
"""

import math
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy
import pygame

from autosim import actions, settings
from autosim.link import VideoLink
from autosim.session import Notification, ScenarioSession
from autosim.simulation import CameraRequest, DrivingMode

Rect = pygame.Rect
Color = Tuple[int, int, int]
C = settings.COLORS

CAMERA_PANELS: Dict[str, str] = {
    "camera_front": "front",
    "camera_left": "left",
    "camera_right": "right",
    "camera_rear": "rear",
    "camera_top_down": "top_down",
}

MANUAL_KEYS: Dict[str, Tuple[int, ...]] = {
    "forward": (pygame.K_w, pygame.K_UP),
    "backward": (pygame.K_s, pygame.K_DOWN),
    "left": (pygame.K_a, pygame.K_LEFT),
    "right": (pygame.K_d, pygame.K_RIGHT),
}


class Hotspot:
    """A clickable rectangle painted this frame."""

    def __init__(self, rect: Rect, on_press: Callable[[], None],
                 on_release: Optional[Callable[[], None]] = None, layer: int = 0) -> None:
        self.rect = rect
        self.on_press = on_press
        self.on_release = on_release
        self.layer = layer          # higher layers (dialogs) get clicks first


class OperatorInterface:
    """Owns the window; draws a session and turns clicks into session calls."""

    def __init__(self, screen: pygame.Surface) -> None:
        self.screen = screen
        self.fonts: Dict[str, pygame.font.Font] = {}
        self.panels: Dict[str, Rect] = {}
        self._text_cache: Dict[Tuple[str, str, Color], pygame.Surface] = {}
        self._hotspots: List[Hotspot] = []
        self._held: Optional[Hotspot] = None
        self._hover = (0, 0)
        self._camera_cache: Dict[str, Tuple[int, pygame.Surface]] = {}
        self.open_dialog: Optional[str] = None
        self.open_notification: Optional[Notification] = None
        self.show_notification_list = False
        self.alert_scroll = 0
        self.map_meters_across = settings.MAP_DEFAULT_METERS_ACROSS
        self._road_cache: List[List[Tuple[float, float]]] = []
        self.resize(screen.get_size())

    # =====================================================================
    # Window and layout
    # =====================================================================

    def resize(self, size: Tuple[int, int]) -> None:
        width, height = size
        self.panels = {
            name: Rect(int(x * width), int(y * height), int(w * width), int(h * height))
            for name, (x, y, w, h) in settings.PANEL_LAYOUT.items()
        }
        self.fonts = {
            name: pygame.font.Font(None, max(10, int(scale * height * 1.35)))
            for name, scale in settings.FONT_SCALE.items()
        }
        self._text_cache.clear()
        self._camera_cache.clear()

    def camera_requests(self, resolution_scale: float) -> List[CameraRequest]:
        """Camera image sizes matching the panels (so images are never stretched)."""
        requests = []
        for panel, camera in CAMERA_PANELS.items():
            if panel not in self.panels:
                continue
            area = self._camera_area(self.panels[panel])
            scale = min(resolution_scale, settings.CAMERA_MAX_IMAGE_WIDTH / max(1, area.width))
            requests.append(CameraRequest(camera, area.width * scale, area.height * scale))
        return requests

    def new_scenario(self, session: ScenarioSession) -> None:
        """Reset per-scenario interface state."""
        self.open_dialog = None
        self.open_notification = None
        self.show_notification_list = False
        self.alert_scroll = 0
        self._camera_cache.clear()
        self._road_cache = session.simulation.road_polylines()

    # =====================================================================
    # Input
    # =====================================================================

    def handle_events(self, session: Optional[ScenarioSession]) -> bool:
        """Process pending events.  Returns False when the window was closed."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE and (pygame.key.get_mods() & pygame.KMOD_SHIFT):
                return False   # Shift+Esc quits (plain Esc is too easy to hit by accident)
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                self._close_overlays()
            if event.type == pygame.VIDEORESIZE:
                self.screen = pygame.display.set_mode((max(640, event.w), max(400, event.h)), pygame.RESIZABLE)
                self.resize(self.screen.get_size())
            if event.type == pygame.MOUSEMOTION:
                self._hover = event.pos
            if session is None:
                continue
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                for hotspot in sorted(self._hotspots, key=lambda h: -h.layer):
                    if hotspot.rect.collidepoint(event.pos):
                        hotspot.on_press()
                        if hotspot.on_release is not None:
                            self._held = hotspot
                        break
            if event.type == pygame.MOUSEBUTTONUP and event.button == 1 and self._held is not None:
                held, self._held = self._held, None
                if held.on_release is not None:
                    held.on_release()
            if event.type == pygame.MOUSEWHEEL and "alerts" in self.panels:
                if self.panels["alerts"].collidepoint(pygame.mouse.get_pos()):
                    self.alert_scroll = max(0, self.alert_scroll - event.y)
        return True

    def manual_keys(self) -> Dict[str, bool]:
        pressed = pygame.key.get_pressed()
        return {name: any(pressed[key] for key in keys) for name, keys in MANUAL_KEYS.items()}

    def _close_overlays(self) -> None:
        self.open_dialog = None
        self.open_notification = None
        self.show_notification_list = False

    # =====================================================================
    # Drawing -- entry points
    # =====================================================================

    def draw(self, session: ScenarioSession, video_link: VideoLink) -> None:
        self._hotspots = []
        self.screen.fill(C["background"])
        now = time.monotonic()
        draw_functions = {
            "speed": self._draw_speed,
            "alerts": self._draw_alerts,
            "decision": self._draw_decision,
            "status": self._draw_status,
            "map": self._draw_map,
            "route": self._draw_route,
            "actions": self._draw_actions,
        }
        for name, rect in self.panels.items():
            if name in CAMERA_PANELS:
                self._draw_camera(name, rect, video_link.frame(CAMERA_PANELS[name], now), session)
            elif name in draw_functions:
                draw_functions[name](rect, session)
        self._draw_banner(session)
        if self.show_notification_list:
            self._draw_notification_list(session)
        if self.open_notification is not None:
            self._draw_notification_detail(session, self.open_notification)
        if self.open_dialog == "detailed_status":
            self._draw_detailed_status(session)
        elif self.open_dialog is not None:
            self._draw_dialog(session, self.open_dialog)
        pygame.display.flip()

    def draw_waiting_screen(self, message: str) -> None:
        """Neutral screen between scenarios."""
        self._hotspots = []
        self.screen.fill(C["background"])
        width, height = self.screen.get_size()
        self._blit_text(message, "title", C["waiting_text"], (width // 2, height // 2), anchor="center")
        pygame.display.flip()

    # =====================================================================
    # Panels
    # =====================================================================

    def _panel(self, rect: Rect, title: Optional[str] = None) -> Rect:
        """Draw a panel background and title; returns the content area."""
        pygame.draw.rect(self.screen, C["panel"], rect, border_radius=6)
        pygame.draw.rect(self.screen, C["panel_border"], rect, 2, border_radius=6)
        if not title:
            return rect.inflate(-12, -12)
        title_height = self.fonts["title"].get_height() + 6
        self._blit_text(title, "title", C["panel_title"], (rect.centerx, rect.y + 4), anchor="midtop")
        return Rect(rect.x + 8, rect.y + title_height + 2, rect.width - 16, rect.height - title_height - 8)

    def _draw_speed(self, rect: Rect, session: ScenarioSession) -> None:
        self._panel(rect)
        state = session.vehicle_state
        self._blit_text("{:.0f} MPH".format(state.speed_mph), "big", C["text"],
                        (rect.centerx - rect.width // 8, rect.centery), anchor="center")
        mode_color = C["text_critical"] if state.mode == DrivingMode.EMERGENCY_STOP else (
            C["text_warning"] if state.mode in (DrivingMode.HOLDING, DrivingMode.STOPPING, DrivingMode.MANUAL)
            else C["text_dim"])
        label = DrivingMode.LABELS.get(state.mode, state.mode)
        self._blit_text(label, "small", mode_color, (rect.right - 10, rect.centery - 8), anchor="midright")
        if session.hazard_lights_on and int(time.monotonic() * 2) % 2 == 0:
            self._blit_text("HAZARDS", "small", C["text_warning"], (rect.right - 10, rect.centery + 10),
                            anchor="midright")

    def _draw_alerts(self, rect: Rect, session: ScenarioSession) -> None:
        content = self._panel(rect, settings.PANEL_TITLES.get("alerts"))
        self._draw_bell(rect, session)
        line_height = self.fonts["normal"].get_height() + 4
        visible_lines = max(1, content.height // line_height)
        alerts = session.alerts
        self.alert_scroll = min(self.alert_scroll, max(0, len(alerts) - visible_lines))
        if not alerts:
            self._blit_text("No active alerts", "normal", C["text_dim"], content.topleft)
        for row, alert in enumerate(alerts[self.alert_scroll:self.alert_scroll + visible_lines]):
            y = content.y + row * line_height
            color = C["severity_" + alert.severity]
            pygame.draw.circle(self.screen, color, (content.x + 6, y + line_height // 2 - 1), 5)
            text = self._fit_text(alert.text, "normal", content.width - 34)
            self._blit_text(text, "normal", C["text"], (content.x + 18, y))
        if len(alerts) > visible_lines:   # scroll bar
            bar = Rect(content.right - 8, content.y, 6, content.height)
            pygame.draw.rect(self.screen, C["panel_border"], bar, border_radius=3)
            knob_height = max(12, bar.height * visible_lines // len(alerts))
            knob_y = bar.y + (bar.height - knob_height) * self.alert_scroll // max(1, len(alerts) - visible_lines)
            pygame.draw.rect(self.screen, C["text_dim"], (bar.x, knob_y, bar.width, knob_height), border_radius=3)

    def _draw_bell(self, alerts_rect: Rect, session: ScenarioSession) -> None:
        """Notification bell with unread count (top-right of the Alerts panel)."""
        size = self.fonts["title"].get_height()
        bell = Rect(alerts_rect.right - size - 14, alerts_rect.y + 5, size + 6, size)
        center_x = bell.centerx
        body = [(center_x - size * 0.32, bell.y + size * 0.72), (center_x - size * 0.26, bell.y + size * 0.3),
                (center_x, bell.y + size * 0.12), (center_x + size * 0.26, bell.y + size * 0.3),
                (center_x + size * 0.32, bell.y + size * 0.72)]
        pygame.draw.polygon(self.screen, C["text"], body)
        pygame.draw.circle(self.screen, C["text"], (int(center_x), int(bell.y + size * 0.82)), max(2, size // 10))
        unread = session.unread_notifications()
        if unread:
            badge_center = (bell.right - 2, bell.y + 4)
            pygame.draw.circle(self.screen, C["button_danger"], badge_center, max(7, size // 3))
            self._blit_text(str(unread), "small", C["text"], badge_center, anchor="center")

        def toggle_list() -> None:
            self.show_notification_list = not self.show_notification_list
            self.open_notification = None
            session.log("notifications_opened" if self.show_notification_list else "notifications_closed")
        self._hotspots.append(Hotspot(bell.inflate(8, 8), toggle_list, layer=1))

    def _draw_decision(self, rect: Rect, session: ScenarioSession) -> None:
        decision = session.decision
        content = self._panel(rect, None if decision else settings.PANEL_TITLES.get("decision"))
        if decision is None:
            self._blit_text("No pending requests from the vehicle", "normal", C["text_dim"], content.topleft)
            return
        lines = self._wrap(decision.prompt, "title", content.width)
        y = content.y
        for line in lines[:2]:
            self._blit_text(line, "title", C["text"], (content.centerx, y), anchor="midtop")
            y += self.fonts["title"].get_height()
        options = decision.options
        count = max(1, len(options))
        columns = min(count, 3)
        rows = math.ceil(count / columns)
        gap = 10
        button_area = Rect(content.x, y + 6, content.width, content.bottom - y - 6)
        button_width = (button_area.width - gap * (columns - 1)) // columns
        button_height = min(int(self.fonts["normal"].get_height() * 2.2), (button_area.height - gap * (rows - 1)) // rows)
        for index, (label, _) in enumerate(options):
            column, row = index % columns, index // columns
            button = Rect(button_area.x + column * (button_width + gap),
                          button_area.y + row * (button_height + gap), button_width, button_height)
            style = {"yes": "yes", "no": "no"}.get(label.strip().lower(), "normal")
            self._button(button, label, style, lambda label=label: session.answer(label))

    def _draw_status(self, rect: Rect, session: ScenarioSession) -> None:
        content = self._panel(rect, settings.PANEL_TITLES.get("status"))
        values = session.status_values
        line = self.fonts["normal"].get_height() + 3
        rows = [
            ("Software", values.get("software_status", ""), "software_status", ""),
            None,
            ("Battery", values.get("battery_percent", ""), "battery_percent", "%"),
            ("Battery temp", values.get("battery_temp_f", ""), "battery_temp_f", "°F"),
            ("Motor temp", values.get("motor_temp_f", ""), "motor_temp_f", "°F"),
            None,
        ]
        y = content.y
        for row in rows:
            if row is None:
                y += line // 2
                continue
            label, value, key, unit = row
            self._status_line(content.x, y, label, value, key, unit)
            y += line
        # Tire pressures in a 2 x 2 grid: front-left/right, rear-left/right.
        self._blit_text("Tire pressure", "normal", C["text"], (content.x, y))
        grid_x = content.x + self.fonts["normal"].size("Tire pressure  ")[0]
        for index, key in enumerate(("tire_fl_psi", "tire_fr_psi", "tire_rl_psi", "tire_rr_psi")):
            column, row_index = index % 2, index // 2
            value = values.get(key, "")
            self._blit_text(self._format(value), "normal", self._value_color(key, value),
                            (grid_x + column * self.fonts["normal"].size("000.0  ")[0], y + row_index * line))
        button_height = int(self.fonts["normal"].get_height() * 1.6)
        for index, action in enumerate(actions.actions_in_group("status", session.scenario.hidden_buttons)):
            width = self.fonts["normal"].size(action.label)[0] + 24
            button = Rect(content.right - width, content.bottom - button_height, width, button_height)
            self._action_button(button, action, session)

    def _status_line(self, x: int, y: int, label: str, value: Any, key: str, unit: str) -> None:
        self._blit_text("{}:".format(label), "normal", C["text"], (x, y))
        offset = self.fonts["normal"].size("Battery temp:  ")[0]
        self._blit_text(self._format(value) + unit, "normal", self._value_color(key, value), (x + offset, y))

    def _draw_camera(self, panel_name: str, rect: Rect, frame: Any, session: ScenarioSession) -> None:
        pygame.draw.rect(self.screen, C["panel_border"], rect, border_radius=4)
        area = self._camera_area(rect)
        camera = CAMERA_PANELS[panel_name]
        if frame is None:
            pygame.draw.rect(self.screen, C["camera_placeholder"], area)
            self._blit_text("No video", "normal", C["text_dim"], area.center, anchor="center")
        else:
            cached = self._camera_cache.get(camera)
            if cached is None or cached[0] != id(frame):
                surface = pygame.surfarray.make_surface(numpy.ascontiguousarray(frame).swapaxes(0, 1))
                surface = self._cover(surface, area.size)
                self._camera_cache[camera] = (id(frame), surface)
            self.screen.blit(self._camera_cache[camera][1], area.topleft)
        label = settings.PANEL_TITLES.get(panel_name, camera)
        label_surface = self._text(label, "small", C["text"])
        background = Rect(area.x + 4, area.y + 4, label_surface.get_width() + 10, label_surface.get_height() + 4)
        shade = pygame.Surface(background.size, pygame.SRCALPHA)
        shade.fill((0, 0, 0, 140))
        self.screen.blit(shade, background.topleft)
        self.screen.blit(label_surface, (background.x + 5, background.y + 2))
        if panel_name == "camera_front" and session.manual_control_active:
            self._blit_text("MANUAL CONTROL - W A S D / arrow keys", "normal", C["text_warning"],
                            (area.centerx, area.bottom - 6), anchor="midbottom")
        if panel_name == "camera_front" and session.talking_to:
            self._blit_text("Talking to {}...".format("passenger" if session.talking_to == "passenger" else "people outside"),
                            "normal", C["text_warning"], (area.centerx, area.y + 6), anchor="midtop")

    def _draw_map(self, rect: Rect, session: ScenarioSession) -> None:
        content = self._panel(rect, settings.PANEL_TITLES.get("map"))
        pygame.draw.rect(self.screen, C["map_background"], content)
        state = session.vehicle_state
        scale = content.width / self.map_meters_across

        def to_screen(point: Tuple[float, float]) -> Tuple[float, float]:
            return (content.centerx + (point[0] - state.x) * scale, content.centery + (point[1] - state.y) * scale)

        previous_clip = self.screen.get_clip()
        self.screen.set_clip(content)
        road_width = max(2, int(7 * scale))
        for polyline in self._road_cache:
            if len(polyline) >= 2:
                pygame.draw.lines(self.screen, C["map_road"], False, [to_screen(p) for p in polyline], road_width)
        route = session.simulation.route_polyline()
        if len(route) >= 2:
            pygame.draw.lines(self.screen, C["map_route"], False, [to_screen(p) for p in route], max(2, road_width // 2))
            end_x, end_y = to_screen(route[-1])
            pygame.draw.circle(self.screen, C["map_route"], (int(end_x), int(end_y)), 6)
        for point in session.simulation.hazard_points():
            x, y = to_screen(point)
            pygame.draw.polygon(self.screen, C["map_hazard"], [(x, y - 6), (x - 5, y + 4), (x + 5, y + 4)])
        heading = math.radians(state.yaw_degrees)
        tip = (content.centerx + math.cos(heading) * 11, content.centery + math.sin(heading) * 11)
        left = (content.centerx + math.cos(heading + 2.5) * 8, content.centery + math.sin(heading + 2.5) * 8)
        right = (content.centerx + math.cos(heading - 2.5) * 8, content.centery + math.sin(heading - 2.5) * 8)
        pygame.draw.polygon(self.screen, C["map_ego"], [tip, left, right])
        self.screen.set_clip(previous_clip)

        # Zoom buttons (top right) and tool buttons (top left).
        size = int(self.fonts["title"].get_height() * 1.1)
        zoom_in = Rect(content.right - 2 * size - 6, content.y + 4, size, size)
        zoom_out = Rect(content.right - size - 2, content.y + 4, size, size)
        self._button(zoom_in, "+", "normal", lambda: self._zoom(1 / settings.MAP_ZOOM_STEP, session))
        self._button(zoom_out, "-", "normal", lambda: self._zoom(settings.MAP_ZOOM_STEP, session))
        x = content.x + 4
        button_height = int(self.fonts["small"].get_height() * 1.7)
        for action in actions.actions_in_group("map", session.scenario.hidden_buttons):
            width = self.fonts["small"].size(action.label)[0] + 14
            self._action_button(Rect(x, content.y + 4, width, button_height), action, session, font="small")
            x += width + 4
        gps = "{:.{d}f}°, {:.{d}f}°".format(state.latitude, state.longitude, d=settings.GPS_DECIMALS)
        gps_surface = self._text(gps, "small", C["text"])
        box = Rect(content.x + 4, content.bottom - gps_surface.get_height() - 8, gps_surface.get_width() + 10,
                   gps_surface.get_height() + 4)
        pygame.draw.rect(self.screen, C["panel"], box, border_radius=3)
        self.screen.blit(gps_surface, (box.x + 5, box.y + 2))
        self._blit_text("{:.0f} m".format(self.map_meters_across), "small", C["text_dim"],
                        (content.right - 4, content.bottom - 4), anchor="bottomright")

    def _zoom(self, factor: float, session: ScenarioSession) -> None:
        self.map_meters_across = min(settings.MAP_MAX_METERS_ACROSS,
                                     max(settings.MAP_MIN_METERS_ACROSS, self.map_meters_across * factor))
        session.log("map_zoom", meters_across=round(self.map_meters_across))

    def _draw_route(self, rect: Rect, session: ScenarioSession) -> None:
        content = self._panel(rect, settings.PANEL_TITLES.get("route"))
        if session.route_complete:
            for index, line in enumerate(self._wrap(session.route_complete_text(), "title", content.width)):
                self._blit_text(line, "title", C["text"],
                                (content.centerx, content.y + 8 + index * self.fonts["title"].get_height()), anchor="midtop")
            return
        traveled, total = session.route_progress()
        minutes_done, minutes_total = session.route_minutes()
        fraction = 0.0 if total <= 0 else traveled / total
        bar = Rect(content.x + 6, content.y + 4, content.width - 12, max(10, content.height // 4))
        pygame.draw.rect(self.screen, C["progress_back"], bar, border_radius=bar.height // 2)
        fill = Rect(bar.x, bar.y, int(bar.width * fraction), bar.height)
        if fill.width > 0:
            pygame.draw.rect(self.screen, C["progress_fill"], fill, border_radius=bar.height // 2)
        y = bar.bottom + 6
        self._blit_text("{:.0f} of {:.0f} min".format(minutes_done, minutes_total), "normal", C["text"],
                        (bar.right, y), anchor="topright")
        self._blit_text("{:.1f} of {:.1f} mi".format(traveled, total), "normal", C["text"],
                        (bar.right, y + self.fonts["normal"].get_height() + 2), anchor="topright")
        self._blit_text("{:.0f}%".format(fraction * 100), "normal", C["text"], (bar.x, y))

    def _draw_actions(self, rect: Rect, session: ScenarioSession) -> None:
        content = self._panel(rect, settings.PANEL_TITLES.get("actions"))
        hidden = session.scenario.hidden_buttons
        left = Rect(content.x, content.y, int(content.width * 0.55), content.height)
        right = Rect(left.right + 8, content.y, content.right - left.right - 8, content.height)

        # Command list (left).
        line = int(self.fonts["normal"].get_height() * 1.45)
        for index, action in enumerate(actions.actions_in_group("actions", hidden)):
            self._action_button(Rect(left.x, left.y + index * line, left.width, line - 3), action, session)

        # Driving buttons (right).
        drive = actions.actions_in_group("drive", hidden)
        end_buttons = actions.actions_in_group("end", hidden)
        end_height = int(self.fonts["normal"].get_height() * 1.7)
        available = right.height - (end_height + 10 if end_buttons else 0)
        gap = 8
        height = min(int(self.fonts["title"].get_height() * 2.0), (available - gap * (len(drive) - 1)) // max(1, len(drive)))
        for index, action in enumerate(drive):
            self._action_button(Rect(right.x, right.y + index * (height + gap), right.width, height), action, session,
                                font="title")
        for action in end_buttons:
            width = self.fonts["normal"].size(action.label)[0] + 24
            self._action_button(Rect(content.right - width, content.bottom - end_height, width, end_height),
                                action, session)

        # Fading feedback text (bottom left) -- replaces popups for every click.
        y = content.bottom - 4
        for text, opacity in reversed(session.active_feedback()):
            for wrapped in reversed(self._wrap(text, "small", left.width)):
                surface = self._text(wrapped, "small", C["text_warning"]).copy()
                surface.set_alpha(int(255 * opacity))
                y -= surface.get_height()
                self.screen.blit(surface, (content.x, y))
            y -= 4

    # =====================================================================
    # Overlays
    # =====================================================================

    def _draw_banner(self, session: ScenarioSession) -> None:
        """Newest notification as a banner for a few seconds."""
        if not session.notifications:
            return
        newest = session.notifications[0]
        if session.elapsed - newest.created_at > settings.NOTIFICATION_BANNER_SECONDS or newest.handled:
            return
        width = self.screen.get_width()
        text = newest.title + ("  -  " + newest.body if newest.body else "")
        surface = self._text(self._fit_text(text, "normal", int(width * 0.45)), "normal", C["banner_text"])
        banner = Rect(0, 0, surface.get_width() + 30, surface.get_height() + 14)
        # Below the speed read-out, so the speed is never hidden.
        anchor = self.panels.get("camera_top_down", self.screen.get_rect())
        banner.midtop = (anchor.centerx, anchor.y + 4)
        pygame.draw.rect(self.screen, C["banner"], banner, border_radius=8)
        self.screen.blit(surface, (banner.x + 15, banner.y + 7))

        def open_it() -> None:
            self._open_notification(newest, session)
        self._hotspots.append(Hotspot(banner, open_it, layer=3))

    def _overlay_box(self, width_fraction: float, height_lines: int, anchor_panel: str = "status") -> Rect:
        """Rectangle for a dialog, placed over the given panel column (not the front camera)."""
        anchor = self.panels.get(anchor_panel, self.screen.get_rect())
        width = int(self.screen.get_width() * width_fraction)
        height = min(int(self.screen.get_height() * 0.8), height_lines * (self.fonts["normal"].get_height() + 14) + 40)
        box = Rect(0, 0, width, height)
        box.topright = (anchor.right, anchor.y)
        return box

    def _dialog_frame(self, box: Rect, title: str, on_close: Callable[[], None]) -> Rect:
        shadow = box.move(4, 4)
        pygame.draw.rect(self.screen, (0, 0, 0), shadow, border_radius=8)
        pygame.draw.rect(self.screen, C["dialog"], box, border_radius=8)
        pygame.draw.rect(self.screen, C["text_dim"], box, 2, border_radius=8)
        self._blit_text(title, "title", C["panel_title"], (box.centerx, box.y + 8), anchor="midtop")
        size = self.fonts["title"].get_height()
        close = Rect(box.right - size - 8, box.y + 6, size, size)
        self._button(close, "X", "normal", on_close, layer=2)
        self._hotspots.append(Hotspot(box, lambda: None, layer=1))   # clicks inside do not fall through
        return Rect(box.x + 12, box.y + size + 16, box.width - 24, box.height - size - 24)

    def _draw_dialog(self, session: ScenarioSession, name: str) -> None:
        dialog = actions.DIALOGS[name]
        options = actions.dialog_options(name, session)
        body_lines = dialog.text(session).split("\n") if dialog.text else []
        box = self._overlay_box(0.24, len(options) + len(body_lines) + 1, "status")

        def close() -> None:
            self.open_dialog = None
        content = self._dialog_frame(box, dialog.title, close)
        y = content.y
        for line in body_lines:
            self._blit_text(line, "normal", C["text"], (content.x, y))
            y += self.fonts["normal"].get_height() + 4
        height = self.fonts["normal"].get_height() + 10
        for option in options:
            def choose(option=option) -> None:
                result = option.handler(session)
                self.open_dialog = result if result in actions.DIALOGS else None
            self._button(Rect(content.x, y, content.width, height), option.label, option.style, choose, layer=2)
            y += height + 6

    def _draw_detailed_status(self, session: ScenarioSession) -> None:
        rows = actions.DETAILED_STATUS_ROWS
        box = self._overlay_box(0.24, len(rows) // 1 + 3, "status")
        box.height = min(box.height, int(self.screen.get_height() * 0.62))

        def close() -> None:
            self.open_dialog = None
        content = self._dialog_frame(box, "Detailed status", close)
        line = self.fonts["small"].get_height() + 3
        values = session.status_values
        column_width = content.width // 2
        per_column = max(1, (content.height - 4) // line)
        for index, (key, label, unit) in enumerate(rows):
            column, row = index // per_column, index % per_column
            x = content.x + column * column_width
            y = content.y + row * line
            value = values.get(key, "")
            self._blit_text(label + ":", "small", C["text"], (x, y))
            self._blit_text(self._format(value) + unit, "small", self._value_color(key, value),
                            (x + int(column_width * 0.55), y))

    def _draw_notification_list(self, session: ScenarioSession) -> None:
        box = self._overlay_box(0.30, min(10, len(session.notifications)) + 2, "alerts")
        box.topleft = self.panels["alerts"].topleft

        def close() -> None:
            self.show_notification_list = False
        content = self._dialog_frame(box, "Notifications", close)
        if not session.notifications:
            self._blit_text("No notifications", "normal", C["text_dim"], content.topleft)
            return
        height = self.fonts["normal"].get_height() + 10
        for index, notification in enumerate(session.notifications[:10]):
            row = Rect(content.x, content.y + index * (height + 4), content.width, height)
            stamp = "{:>4.0f}s  ".format(notification.created_at)
            label = ("" if notification.read else "* ") + stamp + notification.title
            self._button(row, self._fit_text(label, "normal", row.width - 12), "normal",
                         lambda n=notification: self._open_notification(n, session), layer=2, align="left")

    def _draw_notification_detail(self, session: ScenarioSession, notification: Notification) -> None:
        body = self._wrap(notification.body, "normal", int(self.screen.get_width() * 0.28)) if notification.body else []
        box = self._overlay_box(0.30, len(body) + len(notification.actions) + 2, "alerts")
        box.topleft = self.panels["alerts"].topleft

        def close() -> None:
            self.open_notification = None
        content = self._dialog_frame(box, self._fit_text(notification.title, "title", box.width - 60), close)
        y = content.y
        for line in body:
            self._blit_text(line, "normal", C["text"], (content.x, y))
            y += self.fonts["normal"].get_height() + 2
        height = self.fonts["normal"].get_height() + 10
        for label, function in notification.actions:
            def run(function=function) -> None:
                notification.handled = True
                self.open_notification = None
                function()
            self._button(Rect(content.x, y + 6, content.width, height), label, "primary", run, layer=2)
            y += height + 6

    def _open_notification(self, notification: Notification, session: ScenarioSession) -> None:
        notification.read = True
        self.show_notification_list = False
        self.open_notification = notification
        session.log("notification_opened", title=notification.title)

    # =====================================================================
    # Buttons
    # =====================================================================

    def _action_button(self, rect: Rect, action: actions.Action, session: ScenarioSession, font: str = "normal") -> None:
        enabled = action.is_enabled(session) if action.is_enabled else True
        toggled = action.is_on(session) if action.is_on else False
        label = action.label
        if action.kind == "toggle" and action.style == "link":
            label = ("[x] " if toggled else "[  ] ") + label
        if action.kind == "hold" and session.talking_to and action.action_id.endswith(session.talking_to.replace("external", "externally")):
            toggled = True

        def press() -> None:
            result = action.handler(session)
            if result in actions.DIALOGS:
                self.open_dialog = result
        release = (lambda: action.release(session)) if action.kind == "hold" and action.release else None
        style = "toggled" if toggled and action.style != "link" else action.style
        self._button(rect, label, style, press if enabled else (lambda: None), release, font=font,
                     enabled=enabled, highlighted=toggled and action.style == "link",
                     align="left" if action.style == "link" else "center")

    def _button(self, rect: Rect, label: str, style: str, on_press: Callable[[], None],
                on_release: Optional[Callable[[], None]] = None, font: str = "normal", layer: int = 0,
                enabled: bool = True, highlighted: bool = False, align: str = "center") -> None:
        hover = rect.collidepoint(self._hover)
        colors = {
            "normal": ("button", "button_hover"),
            "link": ("panel", "button_hover"),
            "danger": ("button_danger", "button_danger_hover"),
            "primary": ("button_primary", "button_primary_hover"),
            "toggled": ("button_toggled", "button_toggled"),
            "yes": ("button_yes", "button_yes"),
            "no": ("button_no", "button_no"),
        }.get(style, ("button", "button_hover"))
        color = C[colors[1] if hover else colors[0]] if enabled else C["button_disabled"]
        if style == "danger":
            pygame.draw.ellipse(self.screen, color, rect)
            pygame.draw.ellipse(self.screen, C["panel_border"], rect, 2)
        else:
            pygame.draw.rect(self.screen, color, rect, border_radius=5)
            if style != "link":
                pygame.draw.rect(self.screen, C["panel_border"], rect, 2, border_radius=5)
        text_color = C["text_warning"] if highlighted else C["button_text"]
        text = self._fit_text(label, font, rect.width - 10)
        if align == "left":
            self._blit_text(text, font, text_color, (rect.x + 6, rect.centery), anchor="midleft")
        else:
            self._blit_text(text, font, text_color, rect.center, anchor="center")
        self._hotspots.append(Hotspot(rect, on_press, on_release, layer))

    # =====================================================================
    # Text and image helpers
    # =====================================================================

    def _text(self, text: str, font: str, color: Color) -> pygame.Surface:
        key = (text, font, color)
        surface = self._text_cache.get(key)
        if surface is None:
            if len(self._text_cache) > 600:
                self._text_cache.clear()
            surface = self.fonts[font].render(text, True, color)
            self._text_cache[key] = surface
        return surface

    def _blit_text(self, text: str, font: str, color: Color, position: Tuple[float, float], anchor: str = "topleft") -> None:
        surface = self._text(text, font, color)
        rect = surface.get_rect(**{anchor: (int(position[0]), int(position[1]))})
        self.screen.blit(surface, rect)

    def _wrap(self, text: str, font: str, width: int) -> List[str]:
        words, lines, current = text.split(), [], ""
        for word in words:
            candidate = (current + " " + word).strip()
            if self.fonts[font].size(candidate)[0] <= width or not current:
                current = candidate
            else:
                lines.append(current)
                current = word
        if current:
            lines.append(current)
        return lines

    def _fit_text(self, text: str, font: str, width: int) -> str:
        if self.fonts[font].size(text)[0] <= width:
            return text
        while text and self.fonts[font].size(text + "...")[0] > width:
            text = text[:-1]
        return text + "..."

    def _format(self, value: Any) -> str:
        if isinstance(value, float):
            return "{:.0f}".format(value) if value == int(value) or abs(value) >= 100 else "{:.1f}".format(value)
        return str(value)

    def _value_color(self, key: str, value: Any) -> Color:
        """Highlight values outside their normal range or status words that are not 'good'."""
        if isinstance(value, (int, float)):
            low, high = settings.STATUS_NORMAL_RANGES.get(key, (None, None))
            if (low is not None and value < low) or (high is not None and value > high):
                return C["text_critical"]
            return C["text"]
        if isinstance(value, str) and value and value.strip().lower() not in settings.STATUS_GOOD_WORDS:
            return C["text_warning"]
        return C["text"]

    def _camera_area(self, rect: Rect) -> Rect:
        return rect.inflate(-4, -4)

    @staticmethod
    def _cover(surface: pygame.Surface, size: Tuple[int, int]) -> pygame.Surface:
        """Scale to fill ``size`` keeping the aspect ratio (crop the overflow)."""
        width, height = size
        source_width, source_height = surface.get_size()
        scale = max(width / source_width, height / source_height)
        scaled = pygame.transform.smoothscale(surface, (max(1, int(source_width * scale + 0.5)),
                                                        max(1, int(source_height * scale + 0.5))))
        crop = Rect(0, 0, width, height)
        crop.center = scaled.get_rect().center
        return scaled.subsurface(crop.clip(scaled.get_rect())).copy()
