"""
The top-left quadrant of the match video: a replica of 2429's own PyQt dashboard window
(gui/dashboard_2026.py + layout_2026.ui), drawn at the same relative positions with the same wording.

Only what the robot logged to NetworkTables is shown.  A widget whose data is not in the log
(buttons, camera view, NT tree, autonomous chooser, untracked indicators ...) is left blank.
"""
from __future__ import annotations

import math

import cv2
import numpy as np

import core
from dash import (F, FD, ASSETS, PW, PH, blit, fit_scale, hue_color, load_icon, put, put_mid, rotated, rrect)
from gui_layout import WIDGETS, WINDOW

WIN_W = 1273                  # window width after the re-arrangement below (the original is 1480)
S = PW / WIN_W                # window pixels -> video pixels
Y0 = 30                       # our title strip above the window
MENU = 22                     # QMenuBar height: the layout's y values start below it

# dark theme to match the rest of the frame (the GUI itself is light)
WIN_BG, PANE_BG, EDGE_C = (24, 24, 24), (34, 34, 34), (95, 95, 95)
TEXT_C, TAB_OFF = (225, 225, 225), (52, 52, 52)
BLACK, WHITE = (0, 0, 0), (255, 255, 255)

# the dashboard's own stylesheet colors, as BGR
ON_BG, OFF_BG, DISC_BG = (0, 235, 80), (0, 0, 220), (95, 95, 95)       # DISC_BG: the GUI's light gray, darkened
FLASH_ON_BG, FLASH_OFF_BG = (0, 0, 0), (250, 20, 0)
HUB_RED, HUB_BLUE = (0, 0, 225), (225, 0, 0)
GRAY_TXT = (200, 200, 200)

COMMANDS = ["IntakeStow", "IntakeDeploy", "IntakeShoot", "IntakeCalStow", "IntakeCalZero", "IntakeOn", "IntakeOff",
            "IntakeReverse", "IntakeBrake", "IntakeIdle", "HopperOn", "HopperOff", "HopperReverse", "IndexerOn",
            "IndexerOff", "IndexerReverse", "ShooterOn", "ShooterOff", "ShooterReverse", "CanStatus", "ResetFlex",
            "GyroReset"]


def _snake(name):
    out = ""
    for i, ch in enumerate(name):
        out += ("_" + ch.lower()) if (ch.isupper() and i) else ch.lower()
    return out


CMD_BY_WIDGET = {f"qlabel_{_snake(c)}_indicator": c for c in COMMANDS}

# static QLabels that are only shown when the data they describe is in the log: label -> signal key
LABEL_DATA = {"label_2": "intake_angle", "label_4": "intake_rpm", "label_10": "indexer_rpm", "label_17": "hopper_rpm",
              "label_12": "shooter_rpm", "label_11": "navx", "label_19": "q_dtap", "label_14": "pdh_v",
              "label_15": "pdh_a", "label_21": "q_battery", "label_18": "auto_delay", "label_13": "cam_right_frames",
              "label_16": "q_tracking"}
LCD_DATA = {"qlcd_intake_position": "intake_angle", "qlcd_intake_speed": "intake_rpm", "qlcd_hopper_speed": "hopper_rpm",
            "qlcd_indexer_speed": "indexer_rpm", "qlcd_shooter_speed": "shooter_rpm", "qlcd_navx_heading": "navx"}
QUEST_IND = {"qlabel_questnav_heartbeat_indicator": "q_connected", "qlabel_questnav_inbounds_indicator": "q_accepted",
             "qlabel_questnav_tracking_indicator": "q_tracking", "qlabel_questnav_dtap_indicator": "q_passthrough",
             "qlabel_questnav_sync_toggle_indicator": "q_synched", "qlabel_questnav_enabled_toggle_indicator": "q_in_use",
             "qlabel_questnav_reset_indicator": "q_reset_cmd"}
CAMERAS = [("ARDU RIGHT", "cam_right", "qlabel_cam0_indicator", "qlabel_cam0_target_indicator"),
           ("ARDU BACK", "cam_back", "qlabel_cam1_indicator", "qlabel_cam1_target_indicator")]
STATIC_LABELS = {"label_2", "label_4", "label_10", "label_17", "label_12", "label_11", "label_19", "label_13",
                 "label_14", "label_15", "label_21", "label_16", "label_18", "label_20"}


# Re-arrangement so the field can be big: the match clock / FMS / Alliance tiles, the Commands and Status
# group and the PDH / Quest battery monitors move into the strip under the field, the hub indicator and the
# delay box are tucked in under the right-hand groups, and the window is scaled up to the width that is left.
ROW = 380                              # window y of the strip under the field
OVERRIDES = {
    "tabWidget": (720, 0, 545, 481),
    "qlabel_matchtime": (4, ROW, 130, 71),
    "qlabel_fms_connected_indicator": (142, ROW + 12, 115, 20), "qlabel_alliance_indicator": (142, ROW + 38, 115, 20),
    "groupBox_3": (270, ROW, 181, 125), "label_20": (282, ROW + 18, 81, 16),
    "qlabel_intake_idle_indicator": (280, ROW + 34, 78, 24), "qlabel_intake_brake_indicator": (363, ROW + 34, 78, 24),
    "qlabel_intake_cal_zero_indicator": (280, ROW + 62, 160, 24),
    "qlabel_intake_cal_stow_indicator": (280, ROW + 90, 160, 24),
    "label_14": (468, ROW + 4, 70, 16), "qlabel_pdh_voltage_monitor": (465, ROW + 22, 76, 31),
    "label_15": (549, ROW + 4, 70, 16), "qlabel_pdh_current_monitor": (546, ROW + 22, 76, 31),
    "label_21": (630, ROW + 4, 70, 16), "qlabel_quest_battery_monitor": (627, ROW + 22, 76, 31),
    "qlabel_active_hub_indicator": (1040, 404, 221, 51),
    "label_18": (1040, 344, 71, 20), "qcombobox_auto_delay": (1040, 374, 61, 22),
}

# The GUI's Field tab is small and everything below it (the live camera view) is blank in a log replay,
# so the Field tab is enlarged to the full width of the left column (window x 4..713), where the GUI also has it.
FIELD_LEFT, FIELD_W = 4, 709           # window x / width of the enlarged field pane
FIELD_TAB_Y = -1                       # window y of the tab bar; the pane starts one tab bar (24) lower


def X(x):
    return int(round(x * S))


def Y(y):
    return int(round(Y0 + (MENU + y) * S))


def R(rect):
    x, y, w, h = rect
    return X(x), Y(y), max(2, int(round(w * S))), max(2, int(round(h * S)))


def fscale(pt):
    return 0.052 * pt * S


class GuiPanel:
    def __init__(self, tel, ml: core.MatchLog, field: core.FieldImage, red_left: bool, title: str = ""):
        self.tel, self.ml, self.title = tel, ml, title
        self.w = {r["name"]: dict(r, rect=OVERRIDES.get(r["name"], r["rect"])) for r in WIDGETS}
        self.cam_hist = {}
        fw = X(FIELD_W)
        st = core.RenderStyle(red_left=red_left, title="", show_status=False)
        self.fp = core.FieldPanel(field, fw, fw + 24, st)          # tall on purpose: the width limits the field
        inner = self.fp.base[self.fp.hdr:]
        used = np.any(inner != 24, axis=2)                         # crop the dark padding away
        ys, xs = np.nonzero(used)
        y0_, y1_, x0_, x1_ = ys.min(), ys.max() + 1, xs.min(), xs.max() + 1
        self.field_img = inner[y0_:y1_, x0_:x1_].copy()
        fx, fy = X(FIELD_LEFT) + 1, Y(FIELD_TAB_Y + 24)
        self.field_xy = (fx, fy)
        self.field_x0, self.field_y0 = fx - x0_, fy - self.fp.hdr - y0_   # FieldPanel coords -> video coords
        icon_px = int(1.15 * self.fp.sx)
        self.icon_bot = load_icon("blockhead.png", icon_px)
        self.icon_bot_blue = None if self.icon_bot is None else self.icon_bot[:, :, [2, 1, 0, 3]].copy()    # red <-> blue
        self.icon_quest = load_icon("quest.png", icon_px)
        b = np.zeros((357, 357, 4), np.uint8)
        rrect(b, 12, 12, 345, 345, (0, 255, 76, 160), 59, 24)
        self.icon_border = cv2.resize(b, (int(icon_px * 49 / 41),) * 2, interpolation=cv2.INTER_AREA)
        self._base_sig = self._data_sig()
        self.base = self._make_base()

    # ----------------------------------------------------------------- helpers
    def _data_sig(self):
        """Which of the data-dependent static labels have data (it changes while a live connection fills in)."""
        keys = tuple(sorted(set(LABEL_DATA.values())))
        return (tuple(self.has(k) for k in keys), self.cmd_sig("IntakeIdle") is not None,
                self.cmd_sig("IntakeBrake") is not None)

    def has(self, key, n=1):
        s = self.tel.by_key.get(key)
        return s is not None and len(s) >= n

    def cmd_sig(self, cmd):
        from telemetry import SD
        return self.tel.sig.get(SD + f"Command/{cmd}/running")

    def _text(self, img, s, x, y, w, h, pt, color=BLACK, bold=True, th=None):
        """Centred (possibly two-line) text in a rectangle; the degree sign is drawn as a small ring."""
        lines = s.split("\n")
        sc = min(fscale(pt), *(fit_scale(ln.replace("\xb0", " "), w - 4, fscale(pt), 1) for ln in lines))
        th = th or (2 if (bold and pt >= 14) else 1)
        lh = h / len(lines)
        for i, ln in enumerate(lines):
            deg = ln.endswith("\xb0")
            body = ln[:-1] if deg else ln
            (tw, thh), _ = cv2.getTextSize(body, F, sc, th)
            total = tw + (int(5 * sc / 0.4) if deg else 0)
            cx = x + w / 2 - total / 2
            cy = y + lh * i + lh / 2 + thh / 2
            cv2.putText(img, body, (int(cx), int(cy)), F, sc, color, th, cv2.LINE_AA)
            if deg:
                cv2.circle(img, (int(cx + tw + 3 * sc / 0.4), int(cy - thh * 0.8)), max(1, int(2 * sc / 0.4)), color, 1, cv2.LINE_AA)

    def _tile(self, img, name, text=None, bg=DISC_BG, fg=BLACK, border=None, pt=None, radius=7):
        r = self.w[name]
        x, y, w, h = R(r["rect"])
        rad = max(2, int(radius * S))
        if bg == DISC_BG and fg == BLACK:
            fg = (235, 235, 235)                                    # light text on the darkened gray tiles
        if border is not None:
            rrect(img, x, y, x + w, y + h, border, rad)
            rrect(img, x + 2, y + 2, x + w - 2, y + h - 2, bg, max(1, rad - 2))
        else:
            rrect(img, x, y, x + w, y + h, bg, rad)
        self._text(img, text if text is not None else r["text"], x, y, w, h, pt or r["pt"], fg)

    def _indicator(self, img, name, on, text=None, flash=False, t=0.0):
        """The dashboard's default indicator style: green when true, red when false."""
        if on is None:
            return
        if flash and on:
            fl = int(t * 4) % 2 == 0
            self._tile(img, name, text, FLASH_ON_BG if fl else FLASH_OFF_BG, WHITE)
        else:
            self._tile(img, name, text, ON_BG if on else OFF_BG, BLACK if on else GRAY_TXT)

    def _warning(self, img, name, value, lo, hi, red_high, text, t):
        r = self.w[name]
        x, y, w, h = R(r["rect"])
        col = hue_color(value, lo, hi, red_high)
        bad = (value > hi) if red_high else (value < lo)
        fg = (WHITE if int(t * 4) % 2 else BLACK) if bad else BLACK
        rrect(img, x, y, x + w, y + h, col, max(2, int(7 * S)))
        self._text(img, text, x, y, w, h, r["pt"], fg)

    def _lcd(self, img, name, value):
        x, y, w, h = R(self.w[name]["rect"])
        cv2.rectangle(img, (x, y), (x + w, y + h), (16, 16, 16), -1)
        cv2.rectangle(img, (x, y), (x + w, y + h), EDGE_C, 1)
        s = "" if value is None else f"{int(value)}"
        sc = fit_scale(s or "00000", w - 8, 1.0, 2, FD)
        sc = min(sc, h / 30.0)
        (tw, th), _ = cv2.getTextSize(s, FD, sc, 2)
        cv2.putText(img, s, (x + w - 5 - tw, y + (h + th) // 2), FD, sc, (235, 235, 235), 2, cv2.LINE_AA)

    @staticmethod
    def _pose_str(label, p, sign=False):
        x, y, th = p
        rot = math.degrees(th)
        s = "+" if sign else ""
        return f"{label}\n{x:>{s}5.2f}m {y:>{s}4.2f}m {rot:>{s}4.0f}\xb0"

    # ------------------------------------------------------------------- base
    def _make_base(self):
        im = np.full((PH, PW, 3), (24, 24, 24), np.uint8)
        put(im, self.title or self.ml.match_label, 8, 20, 0.55, (230, 230, 230))
        wy1 = Y(722 - MENU)
        cv2.rectangle(im, (0, Y0), (PW, min(PH, wy1)), WIN_BG, -1)
        # tab widgets
        for r in self.w.values():
            if r["cls"] != "QTabWidget":
                continue
            x, y, w, h = R(r["rect"])
            if r["name"] == "tabWidget_2":                           # enlarged field tab (see FIELD_* above)
                x, y = X(FIELD_LEFT), Y(FIELD_TAB_Y)
                w, h = self.field_img.shape[1] + 2, self.field_img.shape[0] + 2 + int(24 * S)
            bar = int(24 * S)
            cv2.rectangle(im, (x, y + bar), (x + w, y + h), PANE_BG, -1)
            cv2.rectangle(im, (x, y + bar), (x + w, y + h), EDGE_C, 1)
            tx = x
            sel = 1 if r["name"] == "tabWidget_2" else 0
            for i, (_pn, ttl) in enumerate(r["tabs"]):
                tw = int((len(ttl) * 7 + 20) * S)
                col = PANE_BG if i == sel else TAB_OFF
                cv2.rectangle(im, (tx, y + 2), (tx + tw, y + bar), col, -1)
                cv2.rectangle(im, (tx, y + 2), (tx + tw, y + bar), EDGE_C, 1)
                put_mid(im, ttl, tx + tw / 2, y + bar / 2 + 1, 0.34, TEXT_C)
                tx += tw
        # group boxes + static labels
        for r in self.w.values():
            if r["cls"] == "QGroupBox" and r["name"] != "qgroupbox_field" and r.get("title"):
                x, y, w, h = R(r["rect"])
                top = y + int(5 * S)
                rrect(im, x, top, x + w, y + h, EDGE_C, 3, 1)
                ttl = r["title"]
                sc = 0.36
                tw = cv2.getTextSize(ttl, F, sc, 1)[0][0]
                cv2.rectangle(im, (x + int(8 * S), y - 1), (x + int(8 * S) + tw + 6, y + int(10 * S)), PANE_BG, -1)
                put(im, ttl, x + int(8 * S) + 3, y + int(8 * S), sc, TEXT_C, 1)
        for name in STATIC_LABELS:
            r = self.w.get(name)
            if r is None:
                continue
            key = LABEL_DATA.get(name)
            if name == "label_20":
                if not any(self.cmd_sig(c) is not None for c in ("IntakeIdle", "IntakeBrake")):
                    continue
            elif key is not None and not self.has(key):
                continue
            x, y, w, h = R(r["rect"])
            put(im, r["text"], x, y + int(h * 0.8), min(0.34, fit_scale(r["text"], w + 6, 0.34)), TEXT_C, 1)
        im[self.field_xy[1]:self.field_xy[1] + self.field_img.shape[0],
           self.field_xy[0]:self.field_xy[0] + self.field_img.shape[1]] = self.field_img
        return im

    # ---------------------------------------------------------------- field
    def fpx(self, x, y):
        px, py = self.fp.to_px(x, y)
        return px + self.field_x0, py + self.field_y0

    def _pose_icon(self, img, pose, icon, alpha=1.0, border=None):
        if pose is None or icon is None:
            return
        x, y, th = pose
        if not (-1.0 <= x <= 17.5 and -1.0 <= y <= 9.0):
            return                                          # QuestNav publishes -10,-10 until it tracks
        p0, p1 = self.fpx(x, y), self.fpx(x + math.cos(th), y + math.sin(th))
        ang = -(math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0])) + 90.0)
        if border is not None:
            blit(img, rotated(border, ang), p0[0], p0[1])
        blit(img, rotated(icon, ang), p0[0], p0[1], alpha)

    # ----------------------------------------------------------------- render
    def render(self, t):
        tel, ml = self.tel, self.ml
        if tel.live:
            sig = self._data_sig()
            if sig != self._base_sig:                     # a topic appeared: its caption can be drawn now
                self._base_sig, self.base = sig, self._make_base()
        img = self.base.copy()
        g = lambda k, d=None: tel.get(k, t, d)
        # ---- field: Quest under the robot (the dashboard raises the robot label on top)
        self._pose_icon(img, g("quest"), self.icon_quest, 0.5, self.icon_border)
        # our robot is drawn in its alliance color (red while the alliance is unknown: shop practice, a log without a match)
        self._pose_icon(img, g("drive"), self.icon_bot_blue if self.ml.is_blue else self.icon_bot)
        # ---- header items in the middle column
        mt = tel.match_time(t)
        if "qlabel_matchtime" in self.w and mt is None:
            self._tile(img, "qlabel_matchtime", "--", DISC_BG, BLACK, radius=15)     # no match: default
        elif "qlabel_matchtime" in self.w:
            if mt < 30:
                fl = int(t * 4) % 2 == 0
                self._tile(img, "qlabel_matchtime", f"* {int(mt)} *", FLASH_ON_BG if fl else FLASH_OFF_BG, WHITE, radius=15)
            else:
                self._tile(img, "qlabel_matchtime", f"{int(mt)}", ON_BG, BLACK, radius=15)
        if self.has("fms"):
            self._indicator(img, "qlabel_fms_connected_indicator", g("fms"), t=t)
        red = g("red")
        if red is not None:
            self._tile(img, "qlabel_alliance_indicator", None, HUB_RED if red else HUB_BLUE, GRAY_TXT)
        # ---- LCDs
        for name, key in LCD_DATA.items():
            if self.has(key):
                self._lcd(img, name, g(key))
        # ---- command indicators (running topics)
        for name, cmd in CMD_BY_WIDGET.items():
            s = self.cmd_sig(cmd)
            if s is not None and name in self.w:
                self._indicator(img, name, bool(s.at(t, False)), t=t)
        # ---- poses
        for name, key, label, sign in (("qlabel_pose_indicator", "drive", "ROBOT POSE", False),
                                       ("qlabel_quest_pose_indicator", "quest", "QUEST POSE", False),
                                       ("qlabel_quest_error_pose_indicator", "delta", "DELTA POSE", True)):
            p = g(key)
            if p is not None:
                self._tile(img, name, self._pose_str(label, p, sign), DISC_BG, BLACK)
        # ---- shot distance / double taps / battery / PDH
        sd = g("shot_dist")
        if sd is not None:
            self._tile(img, "qlabel_shot_distance", f"SHOT DIST\n{sd:.2f}m", DISC_BG, BLACK)
        dt = g("q_dtap")
        if dt is not None:
            self._warning(img, "qlabel_questnav_dtap_count_monitor", dt, 0, 10, True, f"{int(dt)}", t)
        bat = g("q_battery")
        if bat is not None:
            self._warning(img, "qlabel_quest_battery_monitor", bat, 0, 100, False, f"{int(bat)}", t)
        v, a = g("pdh_v"), g("pdh_a")
        if v is not None:
            self._warning(img, "qlabel_pdh_voltage_monitor", v, 8, 12, False, f"{v:.1f}", t)
        if a is not None:
            self._warning(img, "qlabel_pdh_current_monitor", a, 60, 160, True, f"{int(a)}", t)
        # ---- QuestNav indicators
        for name, key in QUEST_IND.items():
            if self.has(key):
                self._indicator(img, name, bool(g(key)), flash=(name == "qlabel_questnav_dtap_indicator"), t=t)
        # ---- cameras: heartbeat counter + targets
        for nick, key, hb, tg in CAMERAS:
            fr = tel.by_key.get(f"{key}_frames")
            if fr is None or len(fr) == 0:
                continue
            age = fr.age(t)
            alive = age is not None and age < 2.5
            conn = self._cam_connections(key, t)
            self._tile(img, hb, f"{nick}: {conn:2d}", ON_BG if alive else OFF_BG, BLACK if alive else GRAY_TXT,
                       border=(0, 235, 80) if alive else DISC_BG, radius=7)
            if self.has(f"{key}_target"):
                on = bool(g(f"{key}_target"))
                self._tile(img, tg, nick.replace(" ", "\n"), ON_BG if on else OFF_BG, BLACK if on else GRAY_TXT,
                           border=(0, 235, 80) if alive else DISC_BG, radius=7)
        # ---- active hub
        self._hub(img, mt, g("game_data", ""))
        # ---- auto delay combobox
        if self.has("auto_delay"):
            r = self.w["qcombobox_auto_delay"]
            x, y, w, h = R(r["rect"])
            cv2.rectangle(img, (x, y), (x + w, y + h), (50, 50, 50), -1)
            cv2.rectangle(img, (x, y), (x + w, y + h), EDGE_C, 1)
            put(img, f"{int(g('auto_delay'))}", x + 4, y + int(h * 0.72), 0.34, TEXT_C, 1)
            cv2.line(img, (x + w - 9, y + h // 2 - 1), (x + w - 5, y + h // 2 + 2), (200, 200, 200), 1)
            cv2.line(img, (x + w - 5, y + h // 2 + 2), (x + w - 1, y + h // 2 - 1), (200, 200, 200), 1)
        return img

    def _cam_connections(self, key, t):
        s = self.tel.by_key.get(f"{key}_frames")
        c = self.tel.by_key.get(f"{key}_conn")
        if c is not None and len(c):
            return int(c.at(t, 0))
        if key not in self.cam_hist or len(self.cam_hist[key]) != len(s):      # (live: the series keeps growing)
            gap = np.diff(s.t) > 2.5
            self.cam_hist[key] = np.concatenate([[1], np.cumsum(gap) + 1])
        i = s.idx(t)
        return int(self.cam_hist[key][i]) if i >= 0 else 0

    def _hub(self, img, mt, game_data):
        name = "qlabel_active_hub_indicator"
        r = self.w[name]
        if mt is None:
            self._tile(img, name, "HUB WAITING", DISC_BG, BLACK)
            return
        active, cd = "BOTH", 0
        if mt > 140:
            cd = mt - 140
        elif mt > 130:
            cd = mt - 130
        elif mt > 30:
            if mt > 105:
                cd, shift = mt - 105, 1
            elif mt > 80:
                cd, shift = mt - 80, 2
            elif mt > 55:
                cd, shift = mt - 55, 3
            else:
                cd, shift = mt - 30, 4
            if game_data in ("R", "B"):
                first = game_data == "R"
                active = ("BLUE" if first else "RED") if shift % 2 else ("RED" if first else "BLUE")
        else:
            cd = mt
        if active in ("RED", "BLUE"):
            self._tile(img, name, f"{active} ACTIVE: {int(cd)}s", HUB_RED if active == "RED" else HUB_BLUE, WHITE)
        else:
            x, y, w, h = R(r["rect"])
            rad = max(2, int(7 * S))
            rrect(img, x, y, x + w, y + h, HUB_BLUE, rad)
            rrect(img, x, y, x + int(w * 0.8), y + h, HUB_RED, rad)
            cv2.rectangle(img, (x + int(w * 0.8) - rad, y), (x + int(w * 0.8) + 1, y + h), HUB_RED, -1)
            self._text(img, f"BOTH ACTIVE: {int(cd)}s", x, y, w, h, r["pt"], WHITE)
