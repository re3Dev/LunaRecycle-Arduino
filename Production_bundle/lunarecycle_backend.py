"""
LunaRecycle Backend
Copyright (C) re:3D, Inc. — All Rights Reserved

Runs a Flask REST API (default http://0.0.0.0:5055) that bridges:
  - Arduino over serial  (Shredder Gate, DC Motor, INA219 energy monitor)
  - Conair Dryer over Modbus RTU

Usage:
    pip install -r requirements.txt
    python lunarecycle_backend.py

Configuration is via the constants / environment variables in the section below.
All endpoints return JSON:  { "ok": true, ... }  or  { "ok": false, "error": "..." }
"""

from __future__ import annotations

import glob
import json
import os
import termios
import threading
import time
from datetime import datetime, timedelta, timezone
from typing import Optional

try:
    from zoneinfo import ZoneInfo
except ImportError:
    ZoneInfo = None

import serial
import websocket
from flask import Flask, jsonify, request, send_from_directory
from flask_cors import CORS
from pymodbus.client import ModbusSerialClient

# ─────────────────────────────────────────────────────────────────────────────
#  Configuration
# ─────────────────────────────────────────────────────────────────────────────

# All settings can be overridden with environment variables (see the systemd
# unit / .env file for the Raspberry Pi deployment). Defaults target a Pi:
#   Arduino Mega  -> /dev/ttyACM0   (native USB CDC)
#   Conair dryer  -> /dev/ttyUSB0   (USB-to-RS485 adapter)
# On Windows override with e.g. LUNA_ARDUINO_PORT=COM4.

ARDUINO_PORT     = os.environ.get("LUNA_ARDUINO_PORT", "/dev/ttyACM0")
ARDUINO_BAUDRATE = int(os.environ.get("LUNA_ARDUINO_BAUD", "115200"))
ARDUINO_TIMEOUT  = 2.0   # seconds for blocking read-until-response
BLASTGATE_TIMEOUT = 12.0  # blast gate moves are blocking and can take seconds
# How often the background supervisor retries a dropped Arduino connection.
RECONNECT_INTERVAL = float(os.environ.get("LUNA_RECONNECT_INTERVAL", "3.0"))

DRYER_PORT      = os.environ.get("LUNA_DRYER_PORT", "/dev/ttyUSB0")
DRYER_BAUDRATE  = int(os.environ.get("LUNA_DRYER_BAUD", "57600"))
DRYER_DEVICE_ID = int(os.environ.get("LUNA_DRYER_ID", "1"))
DRYER_PARITY    = os.environ.get("LUNA_DRYER_PARITY", "N")
DRYER_STOPBITS  = int(os.environ.get("LUNA_DRYER_STOPBITS", "1"))
DRYER_BYTESIZE  = int(os.environ.get("LUNA_DRYER_BYTESIZE", "8"))
DRYER_TIMEOUT   = 0.5    # seconds


def _parse_int_set_env(name: str, default: str) -> set[int]:
    raw = os.environ.get(name, default)
    out: set[int] = set()
    for token in raw.split(","):
        token = token.strip()
        if not token:
            continue
        try:
            out.add(int(token, 0))
        except Exception:
            continue
    return out


# Dryer "run state" can vary by controller revision. Keep explicit sets so
# status, UI chips, and event logging stay stable even when run_state is quirky.
DRYER_RUN_ON_VALUES = _parse_int_set_env("LUNA_DRYER_RUN_ON_VALUES", "100")
DRYER_RUN_OFF_VALUES = _parse_int_set_env("LUNA_DRYER_RUN_OFF_VALUES", "0")
DRYER_SSR_PREHEAT_DELAY_SEC = float(os.environ.get("LUNA_DRYER_SSR_PREHEAT_DELAY_SEC", "15.0"))

# Persistent event log path (JSON Lines). One event per line with CST timestamp.
EVENT_LOG_PATH = os.environ.get(
    "LUNA_EVENT_LOG_PATH",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "luna_actuator_events.jsonl"),
)


def _resolve_event_log_tz():
    if ZoneInfo is not None:
        try:
            return ZoneInfo("America/Chicago")
        except Exception:
            pass
    return timezone(timedelta(hours=-6), name="CST")


EVENT_LOG_TZ = _resolve_event_log_tz()

# Server bind. 0.0.0.0 makes the dashboard reachable from other devices on the
# LAN (e.g. http://<pi-ip>:5055). Set LUNA_HOST=127.0.0.1 to restrict to the Pi.
SERVER_HOST = os.environ.get("LUNA_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("LUNA_PORT", "5055"))

# Read-only viewer runtime config (served via /api/viewer/config).
VIEWER_DRYER_CAM_URL   = os.environ.get("LUNA_VIEWER_DRYER_CAM_URL", "")
VIEWER_PRINTER_CAM_URL = os.environ.get("LUNA_VIEWER_PRINTER_CAM_URL", "")
VIEWER_PRINTER_API_URL = os.environ.get("LUNA_VIEWER_PRINTER_API_URL", "")

# Moonraker (Klipper) live extrusion monitoring.
def _resolve_moonraker_ws_url() -> str:
    explicit = os.environ.get("LUNA_MOONRAKER_WS_URL", "").strip()
    if explicit:
        return explicit

    api_url = os.environ.get("LUNA_VIEWER_PRINTER_API_URL", "").strip()
    if api_url:
        from urllib.parse import urlparse

        parsed = urlparse(api_url)
        if parsed.scheme in ("http", "https") and parsed.netloc:
            scheme = "wss" if parsed.scheme == "https" else "ws"
            return f"{scheme}://{parsed.netloc}/websocket"

    # Default to the local Pi install convention from DEPLOY_PI.md so Moonraker
    # comes up automatically on the common same-host setup.
    return "ws://127.0.0.1:7125/websocket"


MOONRAKER_WS_URL = _resolve_moonraker_ws_url()
MOONRAKER_EXTRUDER_VELOCITY_MIN = float(os.environ.get("LUNA_MOONRAKER_EXTRUDER_VEL_MIN", "0.01"))
MOONRAKER_EXTRUDER_UNITS_PER_ROTATION = float(os.environ.get("LUNA_MOONRAKER_EXTRUDER_UNITS_PER_ROTATION", "1.0"))
RATIO_TEST_SPINS_PER_HOME = int(os.environ.get("LUNA_RATIO_TEST_SPINS_PER_HOME", "10"))
RATIO_TEST_POLL_SEC = float(os.environ.get("LUNA_RATIO_TEST_POLL_SEC", "0.5"))
RATIO_TEST_VACUUM_BOOST_SEC = float(os.environ.get("LUNA_RATIO_TEST_VACUUM_BOOST_SEC", "5.0"))
RATIO_TEST_VACUUM_IDLE_PCT = int(os.environ.get("LUNA_RATIO_TEST_VACUUM_IDLE_PCT", "15"))
RATIO_TEST_MIX_PWM = int(os.environ.get("LUNA_RATIO_TEST_MIX_PWM", "200"))
RATIO_TEST_BG_SETTLE_SEC = float(os.environ.get("LUNA_RATIO_TEST_BG_SETTLE_SEC", "5.0"))
RATIO_TEST_STOP_SETTLE_SEC = float(os.environ.get("LUNA_RATIO_TEST_STOP_SETTLE_SEC", "5.0"))
RATIO_TEST_REVERSE_SEC = float(os.environ.get("LUNA_RATIO_TEST_REVERSE_SEC", "15.0"))
RATIO_TEST_FORWARD_RECOVER_SEC = float(os.environ.get("LUNA_RATIO_TEST_FORWARD_RECOVER_SEC", "15.0"))
MOONRAKER_RECONNECT_SEC = float(os.environ.get("LUNA_MOONRAKER_RECONNECT_SEC", "2.0"))
MOONRAKER_FE_TEMP_C = float(os.environ.get("LUNA_MOONRAKER_FE_TEMP_C", "100.0"))
MOONRAKER_FE_RETRY_SEC = float(os.environ.get("LUNA_MOONRAKER_FE_RETRY_SEC", "5.0"))

# ─────────────────────────────────────────────────────────────────────────────
#  Flask app
# ─────────────────────────────────────────────────────────────────────────────

app = Flask(__name__)
CORS(app)

_BUNDLE_DIR = os.path.dirname(os.path.abspath(__file__))


class EventLogger:
    """Thread-safe, append-only logger for actuator edge and action events."""

    def __init__(self, path: str) -> None:
        self._path = path
        self._lock = threading.Lock()
        self._last_error = ""
        self._last_state: dict[str, bool] = {}
        self._last_file_size: Optional[int] = None
        # Ensure the destination directory exists so first-write cannot fail
        # silently when a custom path points to a missing folder.
        folder = os.path.dirname(os.path.abspath(self._path))
        if folder:
            os.makedirs(folder, exist_ok=True)
        self._archive_legacy_log_if_needed()
        self._ensure_log_file_exists()
        self._last_file_size = self._safe_file_size()

    def _safe_file_size(self) -> Optional[int]:
        try:
            if os.path.exists(self._path):
                return os.path.getsize(self._path)
        except Exception:
            return None
        return None

    def _sync_file_state_locked(self) -> None:
        """Detect log rotation/truncation and reset edge dedupe state."""
        self._ensure_log_file_exists()
        current_size = self._safe_file_size()
        prev_size = self._last_file_size
        if prev_size is not None and current_size is not None and current_size < prev_size:
            self._last_state = {}
        self._last_file_size = current_size

    def _ensure_log_file_exists(self) -> None:
        """Create the active log file at startup for easy deployment verification."""
        try:
            if not os.path.exists(self._path):
                with open(self._path, "a", encoding="utf-8"):
                    pass
        except Exception:
            self._last_error = "failed_to_create_event_log"

    def _archive_legacy_log_if_needed(self) -> None:
        """Archive old schema logs so only actuator-edge entries remain in active file."""
        if not os.path.exists(self._path):
            return
        try:
            is_legacy = False
            with open(self._path, "r", encoding="utf-8") as fp:
                for _ in range(30):
                    line = fp.readline()
                    if not line:
                        break
                    s = line.strip()
                    if not s:
                        continue
                    try:
                        obj = json.loads(s)
                    except Exception:
                        # Non-JSON lines are considered legacy/noise and should
                        # be archived out of the active actuator event log.
                        is_legacy = True
                        break

                    if not isinstance(obj, dict):
                        is_legacy = True
                        break

                    # Current schema uses an explicit "event" field with values
                    # like "state" or "action". Legacy rows used "event_type"
                    # and/or "source" instead.
                    if "event" in obj:
                        continue
                    if "event_type" in obj or "source" in obj:
                        is_legacy = True
                        break
            if not is_legacy:
                return
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            archived = f"{self._path}.legacy.{stamp}"
            os.replace(self._path, archived)
        except Exception:
            self._last_error = "failed_to_archive_legacy_log"

    def info(self) -> dict:
        exists = os.path.exists(self._path)
        size = os.path.getsize(self._path) if exists else 0
        return {
            "path": self._path,
            "mode": "actuator_edges_and_actions",
            "timezone": "CST",
            "exists": exists,
            "size_bytes": size,
            "last_error": self._last_error,
        }

    def _append_event(self, event: dict) -> None:
        with open(self._path, "a", encoding="utf-8") as fp:
            fp.write(json.dumps(event, separators=(",", ":"), ensure_ascii=True) + "\n")
        self._last_file_size = self._safe_file_size()

    def log_actuator_state(self, actuator: str, is_on: bool) -> None:
        try:
            with self._lock:
                self._sync_file_state_locked()
                prev = self._last_state.get(actuator)
                if prev is not None and prev == is_on:
                    return
                event = {
                    "ts": datetime.now(EVENT_LOG_TZ).strftime("%Y-%m-%dT%H:%M:%S"),
                    "event": "state",
                    "actuator": actuator,
                    "state": "ON" if is_on else "OFF",
                }
                self._append_event(event)
                self._last_state[actuator] = is_on
                self._last_error = ""
        except Exception:
            self._last_error = "failed_to_write_event_log"
            # Logging must never break control flow.
            pass

    def log_actuator_action(self, actuator: str, action: str, **details) -> None:
        try:
            with self._lock:
                self._sync_file_state_locked()
                event = {
                    "ts": datetime.now(EVENT_LOG_TZ).strftime("%Y-%m-%dT%H:%M:%S"),
                    "event": "action",
                    "actuator": actuator,
                    "action": action,
                }
                for key, value in details.items():
                    if value is None:
                        continue
                    event[str(key)] = value
                self._append_event(event)
                self._last_error = ""
        except Exception:
            self._last_error = "failed_to_write_event_log"
            pass

    def mark_all_off(self) -> None:
        for actuator in (
            "mixer_motor",
            "shredder",
            "tc_pump",
            "agitator",
            "vacuum",
            "dryer",
            "dryer_power",
            "fume_extractor",
        ):
            self.log_actuator_state(actuator, False)


event_logger = EventLogger(EVENT_LOG_PATH)


class MoonrakerMonitor:
    """Background websocket monitor for live extrusion state from Moonraker."""

    _DEFAULT_EXTRUDER_OBJECTS = ("extruder", "extruder1", "extruder2", "extruder3")

    def __init__(self, ws_url: str) -> None:
        self.ws_url = ws_url
        self.enabled = bool(ws_url)
        self._lock = threading.Lock()
        self.connected = False
        self.extruding = False
        self.live_velocity = 0.0
        self.live_extruder_velocity = 0.0
        self.print_state = ""
        self.filament_used = 0.0
        self.axis_map: list[str] = []
        self.extruder_axis_index = -1
        self.live_position: list[float] = []
        self.live_extruder_position = 0.0
        self.extruder_units_total = 0.0
        self.extruder_rotations_total = 0.0
        self._last_live_extruder_position: Optional[float] = None
        self.extruder_temps: dict[str, float] = {}
        self.extruder_targets: dict[str, float] = {}
        self._extruder_zone_phase: dict[str, str] = {}
        self._extruder_zone_target_reached_logged: dict[str, float] = {}
        self.max_extruder_temp_c = 0.0
        self.fe_auto_on: Optional[bool] = None
        self.fe_auto_last_error = ""
        self._next_fe_retry_at = 0.0
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if not self.enabled or self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "enabled": self.enabled,
                "connected": self.connected,
                "extruding": self.extruding,
                "live_velocity": self.live_velocity,
                "live_extruder_velocity": self.live_extruder_velocity,
                "axis_map": list(self.axis_map),
                "extruder_axis_index": self.extruder_axis_index,
                "live_position": list(self.live_position),
                "live_extruder_position": self.live_extruder_position,
                "extruder_units_total": self.extruder_units_total,
                "extruder_rotations_total": self.extruder_rotations_total,
                "extruder_units_per_rotation": MOONRAKER_EXTRUDER_UNITS_PER_ROTATION,
                "print_state": self.print_state,
                "filament_used": self.filament_used,
                "extruder_temps": dict(self.extruder_temps),
                "max_extruder_temp_c": self.max_extruder_temp_c,
                "fe_temp_threshold_c": MOONRAKER_FE_TEMP_C,
                "fe_auto_on": self.fe_auto_on,
                "fe_auto_last_error": self.fe_auto_last_error,
            }

    def _set_connected(self, connected: bool) -> None:
        with self._lock:
            self.connected = connected
            if not connected:
                self.extruding = False
                self.live_velocity = 0.0
                self.live_extruder_velocity = 0.0
                self.axis_map = []
                self.extruder_axis_index = -1
                self.live_position = []
                self.live_extruder_position = 0.0
                self.extruder_units_total = 0.0
                self.extruder_rotations_total = 0.0
                self._last_live_extruder_position = None
                self.extruder_temps = {}
                self.extruder_targets = {}
                self._extruder_zone_phase = {}
                self._extruder_zone_target_reached_logged = {}
                self.max_extruder_temp_c = 0.0

    def _apply_status(self, status: dict) -> None:
        motion = status.get("motion_report", {}) if isinstance(status, dict) else {}
        gmove = status.get("gcode_move", {}) if isinstance(status, dict) else {}
        stats = status.get("print_stats", {}) if isinstance(status, dict) else {}

        velocity_raw = motion.get("live_extruder_velocity")
        if velocity_raw is None:
            velocity_raw = motion.get("live_velocity")
        try:
            velocity = float(velocity_raw)
        except (TypeError, ValueError):
            velocity = None

        live_velocity_raw = motion.get("live_velocity")
        try:
            live_velocity = float(live_velocity_raw)
        except (TypeError, ValueError):
            live_velocity = None

        print_state = stats.get("state")
        filament_used_raw = stats.get("filament_used")
        try:
            filament_used = float(filament_used_raw)
        except (TypeError, ValueError):
            filament_used = None

        axis_map_raw = gmove.get("axis_map") if isinstance(gmove, dict) else None
        axis_map = None
        if isinstance(axis_map_raw, list):
            axis_map = [str(v).strip().lower() for v in axis_map_raw]

        live_pos_raw = motion.get("live_position") if isinstance(motion, dict) else None
        live_pos = None
        if isinstance(live_pos_raw, (list, tuple)):
            live_pos = []
            for v in live_pos_raw:
                try:
                    live_pos.append(float(v))
                except (TypeError, ValueError):
                    live_pos = None
                    break

        pending_zone_events: list[dict] = []

        with self._lock:
            if live_velocity is not None:
                self.live_velocity = live_velocity
            if velocity is not None:
                self.live_extruder_velocity = velocity
            if isinstance(print_state, str):
                self.print_state = print_state
            if filament_used is not None:
                self.filament_used = filament_used

            if axis_map is not None:
                self.axis_map = axis_map
                try:
                    self.extruder_axis_index = self.axis_map.index("e")
                except ValueError:
                    self.extruder_axis_index = -1

            if live_pos is not None:
                self.live_position = live_pos
                e_idx = self.extruder_axis_index
                e_live = None
                if 0 <= e_idx < len(live_pos):
                    e_live = live_pos[e_idx]
                elif len(live_pos) >= 4:
                    e_live = live_pos[3]

                if e_live is not None:
                    self.live_extruder_position = e_live
                    if self._last_live_extruder_position is not None:
                        delta = e_live - self._last_live_extruder_position
                        # Keep a monotonic total of forward extrusion only. Ignore
                        # retracts and giant resets from coordinate re-zeroing.
                        if 0.0 <= delta <= 1000.0:
                            self.extruder_units_total += delta
                    self._last_live_extruder_position = e_live
                    units_per_rot = max(1e-6, MOONRAKER_EXTRUDER_UNITS_PER_ROTATION)
                    self.extruder_rotations_total = self.extruder_units_total / units_per_rot

            # Collect any reported extruder heater temperatures / targets.
            for obj_name, obj_data in status.items():
                if not isinstance(obj_name, str) or not obj_name.startswith("extruder"):
                    continue
                if not isinstance(obj_data, dict):
                    continue
                temp_raw = obj_data.get("temperature")
                try:
                    temp_c = float(temp_raw)
                except (TypeError, ValueError):
                    continue

                prev_target = float(self.extruder_targets.get(obj_name, 0.0))
                prev_phase = self._extruder_zone_phase.get(obj_name, "OFF")

                target_raw = obj_data.get("target", None)
                target_present = "target" in obj_data
                if target_present:
                    try:
                        target_c = max(0.0, float(target_raw))
                    except (TypeError, ValueError):
                        target_c = prev_target
                else:
                    target_c = prev_target

                self.extruder_temps[obj_name] = temp_c
                self.extruder_targets[obj_name] = target_c

                is_on = target_c > 0.5
                at_target = is_on and temp_c >= (target_c - 1.0)

                if is_on and prev_target <= 0.5:
                    pending_zone_events.append({
                        "action": "HEATING_ON",
                        "zone": obj_name,
                        "target_c": round(target_c, 2),
                        "temp_c": round(temp_c, 2),
                    })

                if is_on:
                    phase = "AT_TARGET" if at_target else "HEATING"
                    if at_target:
                        last_logged_target = self._extruder_zone_target_reached_logged.get(obj_name)
                        if last_logged_target is None or abs(last_logged_target - target_c) > 0.25:
                            pending_zone_events.append({
                                "action": "TARGET_REACHED",
                                "zone": obj_name,
                                "target_c": round(target_c, 2),
                                "temp_c": round(temp_c, 2),
                            })
                            self._extruder_zone_target_reached_logged[obj_name] = target_c
                else:
                    if target_present and target_c <= 0.5 and prev_target > 0.5 and temp_c > 35.0:
                        phase = "COOLING"
                        if prev_phase != "COOLING":
                            pending_zone_events.append({
                                "action": "COOLING",
                                "zone": obj_name,
                                "temp_c": round(temp_c, 2),
                            })
                    else:
                        phase = "OFF"
                    self._extruder_zone_target_reached_logged.pop(obj_name, None)

                self._extruder_zone_phase[obj_name] = phase
            self.max_extruder_temp_c = max(self.extruder_temps.values()) if self.extruder_temps else 0.0

            # Extruding means a meaningful live extrusion velocity.
            vel = abs(self.live_extruder_velocity)
            self.extruding = vel >= MOONRAKER_EXTRUDER_VELOCITY_MIN

        for evt in pending_zone_events:
            event_logger.log_actuator_action("extruder_zone", evt.pop("action"), **evt)

        # Drive FE SSR from heater temperatures (ON >= threshold, OFF < threshold).
        self._sync_fume_extractor_from_temp()

    def _sync_fume_extractor_from_temp(self) -> None:
        with self._lock:
            if not self.extruder_temps:
                return
            target_on = self.max_extruder_temp_c >= MOONRAKER_FE_TEMP_C
            if self.fe_auto_on is not None and self.fe_auto_on == target_on:
                return
            if time.monotonic() < self._next_fe_retry_at:
                return

        try:
            set_fume_extractor_power(target_on)
            with self._lock:
                self.fe_auto_on = target_on
                self.fe_auto_last_error = ""
                self._next_fe_retry_at = 0.0
        except Exception as exc:
            with self._lock:
                self.fe_auto_last_error = str(exc)
                self._next_fe_retry_at = time.monotonic() + MOONRAKER_FE_RETRY_SEC

    @staticmethod
    def _subscribe_payload(extruder_objects: list[str]) -> str:
        objects = {
            "motion_report": ["live_extruder_velocity", "live_position"],
            "gcode_move": ["axis_map"],
            "print_stats": ["filament_used", "state"],
        }
        for name in extruder_objects:
            objects[name] = ["temperature", "target", "power"]

        payload = {
            "jsonrpc": "2.0",
            "method": "printer.objects.subscribe",
            "params": {"objects": objects},
            "id": 1,
        }
        return json.dumps(payload)

    @staticmethod
    def _extract_extruder_objects(msg: dict) -> list[str]:
        result = msg.get("result", {}) if isinstance(msg, dict) else {}
        objects = result.get("objects", []) if isinstance(result, dict) else []
        if not isinstance(objects, list):
            return list(MoonrakerMonitor._DEFAULT_EXTRUDER_OBJECTS)
        names = [str(name) for name in objects if isinstance(name, str) and name.startswith("extruder")]
        names.extend(MoonrakerMonitor._DEFAULT_EXTRUDER_OBJECTS)
        # Keep deterministic order and dedupe.
        names = sorted(set(names))
        return names or list(MoonrakerMonitor._DEFAULT_EXTRUDER_OBJECTS)

    def _run(self) -> None:
        while True:
            ws = None
            try:
                ws = websocket.create_connection(self.ws_url, timeout=5)

                # Discover available extruder heater objects first.
                ws.send(json.dumps({"jsonrpc": "2.0", "method": "printer.objects.list", "id": 2}))
                extruder_objects = list(self._DEFAULT_EXTRUDER_OBJECTS)
                list_deadline = time.monotonic() + 5.0
                while time.monotonic() < list_deadline:
                    raw = ws.recv()
                    if not raw:
                        raise RuntimeError("Moonraker websocket closed before objects list")
                    msg = json.loads(raw)
                    if isinstance(msg, dict) and msg.get("id") == 2:
                        extruder_objects = self._extract_extruder_objects(msg)
                        break

                ws.send(self._subscribe_payload(extruder_objects))
                self._set_connected(True)

                while True:
                    raw = ws.recv()
                    if not raw:
                        raise RuntimeError("Moonraker websocket closed")
                    msg = json.loads(raw)

                    # Initial subscribe response.
                    if isinstance(msg, dict) and msg.get("id") == 1:
                        result = msg.get("result", {})
                        status = result.get("status", {}) if isinstance(result, dict) else {}
                        if isinstance(status, dict):
                            self._apply_status(status)
                        continue

                    # Ongoing notify updates.
                    if isinstance(msg, dict) and msg.get("method") == "notify_status_update":
                        params = msg.get("params", [])
                        if isinstance(params, list) and params:
                            status = params[0]
                            if isinstance(status, dict):
                                self._apply_status(status)
            except Exception:
                self._set_connected(False)
                time.sleep(MOONRAKER_RECONNECT_SEC)
            finally:
                try:
                    if ws is not None:
                        ws.close()
                except Exception:
                    pass


moonraker = MoonrakerMonitor(MOONRAKER_WS_URL)


@app.route("/")
def serve_dashboard():
    """Serve the dashboard HTML so it runs on http://127.0.0.1:5055 (same-origin)."""
    return send_from_directory(_BUNDLE_DIR, "lunar_dashboard.html")

@app.route("/viewer")
def serve_viewer():
    """Serve the read-only viewer so it runs on http://127.0.0.1:5055/viewer (same-origin)."""
    return send_from_directory(_BUNDLE_DIR, "lunar_viewer.html")

@app.route("/model")
def serve_model():
    """Serve the spatial model so it runs on http://127.0.0.1:5055/model (same-origin)."""
    return send_from_directory(_BUNDLE_DIR, "lunar_model.html")

@app.route("/assets/<path:filename>")
def serve_bundle_asset(filename: str):
    """Serve static bundle assets (images, css, etc.) used by dashboard pages."""
    return send_from_directory(_BUNDLE_DIR, filename)

@app.route("/api/viewer/config", methods=["GET"])
def api_viewer_config():
    """Expose env-driven config for the read-only viewer page."""
    return jsonify({
        "ok": True,
        "data": {
            "dryer_cam_url": VIEWER_DRYER_CAM_URL,
            "printer_cam_url": VIEWER_PRINTER_CAM_URL,
            "printer_api_url": VIEWER_PRINTER_API_URL,
        },
    })

# ─────────────────────────────────────────────────────────────────────────────
#  Arduino serial bridge
# ─────────────────────────────────────────────────────────────────────────────

class ArduinoBridge:
    """Thread-safe serial bridge to the Arduino running LunaRecycle.ino."""

    SERIAL_IO_EXCEPTIONS = (serial.SerialException, OSError, termios.error)

    def __init__(self) -> None:
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self.connected = False
        self._response_lines: list[str] = []
        # Remember the last-used port so the supervisor can auto-reconnect.
        self._port = ARDUINO_PORT
        self._baud = ARDUINO_BAUDRATE
        # When True the background supervisor keeps trying to (re)connect.
        self._want_connected = True
        # Cache of the last good STATUS so the API can answer instantly even
        # while the serial line is busy with a blocking command (blast gate).
        self._last_status: dict = {}
        self._last_status_ts = 0.0

    # ── Connection ────────────────────────────────────────────────────────────

    @staticmethod
    def _port_candidates(port: str) -> list[str]:
        """Return connection candidates, resilient to ttyACM re-enumeration.

        On the Pi the Mega can disappear as /dev/ttyACM0 and come back as
        /dev/ttyACM1 after EMI / USB hiccups. If that happens, retrying only the
        stale path wedges the backend until a manual restart or env edit. Try
        the requested path first, then sibling ACM nodes and stable by-id names.
        """
        candidates: list[str] = []

        def add(path: str) -> None:
            if path and path not in candidates:
                candidates.append(path)

        add(port)

        base = os.path.basename(port)
        if base.startswith("ttyACM"):
            for path in sorted(glob.glob("/dev/ttyACM*")):
                add(path)

        for path in sorted(glob.glob("/dev/serial/by-id/*")):
            add(path)

        return candidates

    def connect(self, port: str = ARDUINO_PORT, baudrate: int = ARDUINO_BAUDRATE) -> bool:
        with self._lock:
            self._port = port
            self._baud = baudrate
            self._want_connected = True
            if self._ser and self._ser.is_open:
                self.connected = True
                return True

            last_exc: Exception | None = None
            for candidate in self._port_candidates(port):
                try:
                    self._ser = serial.Serial(candidate, baudrate, timeout=ARDUINO_TIMEOUT)
                    time.sleep(2.0)          # wait for Arduino reset after DTR pulse
                    self._ser.reset_input_buffer()
                    self.connected = True
                    self._port = candidate   # remember the working node for next reconnect
                    return True
                except (serial.SerialException, OSError) as exc:
                    self._ser = None
                    self.connected = False
                    last_exc = exc

            if last_exc is None:
                last_exc = RuntimeError(f"No serial candidates found for {port}")
            raise RuntimeError(str(last_exc)) from last_exc

    def disconnect(self) -> None:
        with self._lock:
            self._want_connected = False
            self._reset_serial_locked()

    def _track_state_change_from_command(self, normalized_cmd: str) -> None:
        parts = normalized_cmd.split()
        if not parts:
            return
        cmd = parts[0]

        if cmd == "MOTOR_STOP":
            event_logger.log_actuator_state("mixer_motor", False)
        elif cmd == "MOTOR_SET":
            pwm = 0
            if len(parts) > 1:
                try:
                    pwm = int(parts[1])
                except ValueError:
                    pwm = 0
            event_logger.log_actuator_state("mixer_motor", pwm > 0)
        elif cmd == "SHREDDER_ON":
            event_logger.log_actuator_state("shredder", True)
        elif cmd == "SHREDDER_OFF":
            event_logger.log_actuator_state("shredder", False)
        elif cmd == "TC_PUMP_ON":
            event_logger.log_actuator_state("tc_pump", True)
        elif cmd == "TC_PUMP_OFF":
            event_logger.log_actuator_state("tc_pump", False)
        elif cmd == "TC_HOME":
            event_logger.log_actuator_action("tc_stepper", "MOVE_HOME")
        elif cmd == "TC_PICK":
            event_logger.log_actuator_action("tc_stepper", "MOVE_PICK")
        elif cmd == "TC_STOP":
            event_logger.log_actuator_action("tc_stepper", "STOP")
        elif cmd == "GATE_OPEN":
            event_logger.log_actuator_action("shredder_gate", "OPEN")
        elif cmd == "GATE_CLOSE":
            event_logger.log_actuator_action("shredder_gate", "CLOSE")
        elif cmd == "AGITATOR_STOP":
            event_logger.log_actuator_action("agitator", "STOP")
            event_logger.log_actuator_state("agitator", False)
        elif cmd == "AGITATOR_HOME":
            event_logger.log_actuator_action("agitator", "ROTATE_HOME")
            event_logger.log_actuator_state("agitator", True)
        elif cmd == "VACUUM_STOP":
            event_logger.log_actuator_state("vacuum", False)
        elif cmd == "VACUUM_SET":
            pct = 0
            if len(parts) > 1:
                try:
                    pct = int(parts[1])
                except ValueError:
                    pct = 0
            event_logger.log_actuator_state("vacuum", pct > 0)
        elif cmd == "BLASTGATE_HOME":
            gate = parts[1] if len(parts) > 1 else "ALL"
            event_logger.log_actuator_action("blast_gate", "CLOSE", gate=gate)
        elif cmd == "BLASTGATE_HOMEMAX":
            gate = parts[1] if len(parts) > 1 else "ALL"
            event_logger.log_actuator_action("blast_gate", "OPEN", gate=gate)
        elif cmd == "BLASTGATE_CAL":
            gate = parts[1] if len(parts) > 1 else "ALL"
            event_logger.log_actuator_action("blast_gate", "CALIBRATE", gate=gate)
        elif cmd == "BLASTGATE_POS":
            gate = parts[1] if len(parts) > 1 else None
            percent = parts[2] if len(parts) > 2 else None
            event_logger.log_actuator_action("blast_gate", "POSITION", gate=gate, percent=percent)
        elif cmd == "BLASTGATE_EXT":
            gate = parts[1] if len(parts) > 1 else None
            duration_ms = parts[2] if len(parts) > 2 else None
            event_logger.log_actuator_action("blast_gate", "OPEN_JOG", gate=gate, ms=duration_ms)
        elif cmd == "BLASTGATE_RET":
            gate = parts[1] if len(parts) > 1 else None
            duration_ms = parts[2] if len(parts) > 2 else None
            event_logger.log_actuator_action("blast_gate", "CLOSE_JOG", gate=gate, ms=duration_ms)
        elif cmd == "BLASTGATE_STOP":
            event_logger.log_actuator_action("blast_gate", "STOP")

    @staticmethod
    def _track_async_firmware_line(line: str) -> None:
        if not isinstance(line, str) or not line:
            return

        if line.startswith("[SHREDDER] Shredder ON"):
            event_logger.log_actuator_state("shredder", True)
            if line.endswith(" FWD"):
                event_logger.log_actuator_action("shredder", "DIRECTION", direction="FWD")
            elif line.endswith(" REV"):
                event_logger.log_actuator_action("shredder", "DIRECTION", direction="REV")
            return

        if line.startswith("[SHREDDER] Shredder OFF"):
            event_logger.log_actuator_state("shredder", False)
            return

        if line.startswith("[TC] Sequence started - moving to "):
            bag = line.replace("[TC] Sequence started - moving to ", "").strip()
            event_logger.log_actuator_action("tc_stepper", "MOVE_TO_POSITION", target=bag)
            return

        if line.startswith("[TC] Limit hit - moving to Bag 1"):
            event_logger.log_actuator_action("tc_stepper", "MOVE_TO_POSITION", target="Bag 1")
            return

        if line.startswith("[TC] Homed - at Bag 1"):
            event_logger.log_actuator_action("tc_stepper", "HOME_COMPLETE", target="Bag 1")
            return

        if line.startswith("[TC] Next pick: "):
            rest = line.replace("[TC] Next pick: ", "", 1).strip()
            bag = rest.split("|", 1)[0].strip()
            event_logger.log_actuator_action("tc_stepper", "MOVE_TO_POSITION", target=bag)
            return

        if line.startswith("[TC] Pick complete - moving to shredder"):
            event_logger.log_actuator_action("tc_stepper", "MOVE_TO_SHREDDER")
            return

        if line.startswith("[TC] Dropped "):
            event_logger.log_actuator_action("tc_stepper", "DROP_AT_SHREDDER", detail=line.replace("[TC] ", "", 1))
            return

        if line.startswith("[TC] Vacuum detected at servo "):
            angle = line.replace("[TC] Vacuum detected at servo ", "").replace(" deg", "").strip()
            event_logger.log_actuator_action("tc_stepper", "VACUUM_DETECTED", angle_deg=angle)
            return

        if line == "[TC] Servo down":
            event_logger.log_actuator_action("tc_stepper", "SERVO_DOWN")
            return

        if line == "[TC] Servo up":
            event_logger.log_actuator_action("tc_stepper", "SERVO_UP")
            return

        if line == "[TC] Manual move complete":
            event_logger.log_actuator_action("tc_stepper", "MOVE_COMPLETE")
            return

    def _reset_serial_locked(self) -> None:
        """Close and drop the serial handle. Caller must hold self._lock."""
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        self.connected = False

    def start_supervisor(self) -> None:
        threading.Thread(target=self._supervise, daemon=True).start()

    def _supervise(self) -> None:
        while True:
            if self._want_connected and not self.connected:
                try:
                    self.connect(self._port, self._baud)
                    print(f"[arduino] connected on {self._port}")
                except Exception:
                    pass   # keep retrying quietly
            time.sleep(RECONNECT_INTERVAL)

    # ── Low-level I/O ─────────────────────────────────────────────────────────

    def _send_command(self, cmd: str) -> list[str]:
        """Send a command and collect response lines until a terminal tag or timeout."""
        if not self._ser or not self._ser.is_open:
            raise RuntimeError("Arduino not connected.")

        self._ser.reset_input_buffer()
        self._ser.write((cmd + "\n").encode("ascii"))
        self._ser.flush()

        is_status = cmd.strip().upper() == "STATUS"
        # Blast gate moves are blocking on the Mega (home / cal / pos can take a
        # few seconds) and emit several [BLASTGATE ...] lines, ending with a
        # unique [BLASTGATE_DONE] marker. Give them a longer read window.
        is_blastgate = cmd.strip().upper().startswith("BLASTGATE_")
        if is_blastgate:
            total_timeout = BLASTGATE_TIMEOUT
        else:
            total_timeout = ARDUINO_TIMEOUT
        # Tags that mark a genuine command reply. [ENERGY] is async telemetry the
        # firmware streams every 500 ms; for non-STATUS commands it must be skipped
        # so it isn't mistaken for the command's response (e.g. TC_PUMP_ON replies
        # with a [TC] line, not [ENERGY]).
        reply_tags = ("[STATUS]", "[GATE]", "[MOTOR]", "[TC]", "[SHREDDER]", "[AGITATOR]", "[AGITATOR_HOME]", "[VACUUM]", "[BLASTGATE_DONE]", "[SIZERED]", "[SYSTEM]")

        lines: list[str] = []
        seen_status = False
        deadline = time.monotonic() + total_timeout
        while time.monotonic() < deadline:
            if self._ser.in_waiting:
                raw = self._ser.readline()
                try:
                    line = raw.decode("ascii", errors="replace").strip()
                except Exception:
                    line = raw.decode("latin-1", errors="replace").strip()
                if not line:
                    continue

                self._track_async_firmware_line(line)

                if is_status:
                    # STATUS prints a "[STATUS] ..." line immediately followed by
                    # the "[ENERGY] ..." line; collect both, ignore stray telemetry.
                    if line.startswith("[STATUS]"):
                        lines.append(line)
                        seen_status = True
                    elif line.startswith("[ENERGY]") and seen_status:
                        lines.append(line)
                        break
                    continue

                # Non-STATUS command: ignore async energy telemetry entirely.
                if line.startswith("[ENERGY]"):
                    continue
                lines.append(line)
                if line.startswith(reply_tags):
                    break
            else:
                time.sleep(0.01)

        return lines

    def send(self, cmd: str) -> list[str]:
        with self._lock:
            normalized = str(cmd).strip().upper()
            try:
                lines = self._send_command(cmd)
                self._track_state_change_from_command(normalized)
                return lines
            except self.SERIAL_IO_EXCEPTIONS as exc:
                # Drop the handle so the supervisor reconnects rather than
                # leaving a half-dead port that fails every future command.
                self._reset_serial_locked()
                raise RuntimeError(f"Serial I/O error: {exc}") from exc
            except Exception:
                raise

    # ── Status parsing ────────────────────────────────────────────────────────

    @staticmethod
    def _parse_status_line(line: str) -> dict:
        """
        Parse a [STATUS] key=value line from the firmware into a dict.
        Example:  [STATUS] gate=CLOSED motor_pwm=150 motor_dir=FWD
        """
        result: dict = {}
        parts = line.replace("[STATUS]", "").strip().split()
        for part in parts:
            if "=" in part:
                k, _, v = part.partition("=")
                result[k] = v
        return result

    @staticmethod
    def _parse_energy_line(line: str) -> dict:
        """
        Parse a [ENERGY] key=value line from the firmware into a dict.
        Example:  [ENERGY] pwm=150 dir=FWD V=12.34 I=1.234 P=15.23
        """
        result: dict = {}
        parts = line.replace("[ENERGY]", "").strip().split()
        for part in parts:
            if "=" in part:
                k, _, v = part.partition("=")
                try:
                    result[k] = float(v)
                except ValueError:
                    result[k] = v
        return result

    def _lines_to_status(self, lines: list[str]) -> dict:
        result: dict = {}
        for line in lines:
            if line.startswith("[STATUS]"):
                result.update(self._parse_status_line(line))
            elif line.startswith("[ENERGY]"):
                result["energy"] = self._parse_energy_line(line)
        return result

    def get_status(self) -> dict:
        lines = self.send("STATUS")
        result = self._lines_to_status(lines)
        if result:
            self._last_status = result
            self._last_status_ts = time.monotonic()
            return result
        return {"raw": lines}

    def status_for_api(self) -> dict:
        """Return status for the HTTP layer without ever blocking for long.

        If the serial line is busy (e.g. a multi-second blast-gate move) or the
        Arduino is momentarily down, serve the last cached STATUS with
        cached=True instead of erroring, so the dashboard stays 'connected'.
        """
        if not self.connected:
            return {"connected": False, "data": dict(self._last_status), "cached": True}
        if not self._lock.acquire(timeout=0.3):
            return {"connected": True, "data": dict(self._last_status), "cached": True}
        try:
            result = self._lines_to_status(self._send_command("STATUS"))
            if result:
                self._last_status = result
                self._last_status_ts = time.monotonic()
            return {"connected": True, "data": result or dict(self._last_status),
                    "cached": not bool(result)}
        except self.SERIAL_IO_EXCEPTIONS as exc:
            self._reset_serial_locked()
            return {"connected": False, "data": dict(self._last_status),
                    "cached": True, "error": str(exc)}
        finally:
            self._lock.release()


arduino = ArduinoBridge()


# ─────────────────────────────────────────────────────────────────────────────
#  Dryer Modbus
# ─────────────────────────────────────────────────────────────────────────────

def _s16(x: int) -> int:
    return x - 65536 if x >= 32768 else x


def _u16(x: int) -> int:
    return x & 0xFFFF


def _coerce_setpoint_to_raw_deci(value: object) -> tuple[int, float]:
    """Accept C or raw-deci setpoint input and return (raw_deci, celsius).

    Operator-facing API bodies usually send Celsius values (e.g. 71), while
    Modbus registers store deci-degrees C (e.g. 710). Keep backward
    compatibility: if the absolute value is > 250, treat it as an already-raw
    deci-degree value.
    """
    num = float(value)
    if abs(num) <= 250.0:
        celsius = num
        raw = int(round(num * 10.0))
    else:
        raw = int(round(num))
        celsius = raw / 10.0
    return raw, celsius


def _arduino_send_required(cmd: str) -> list[str]:
    if not arduino.connected:
        raise RuntimeError("Arduino not connected.")
    return arduino.send(cmd)


def set_dryer_power(on: bool) -> list[str]:
    lines = _arduino_send_required("DRYER_SSR_ON" if on else "DRYER_SSR_OFF")
    event_logger.log_actuator_state("dryer_power", bool(on))
    return lines


def set_fume_extractor_power(on: bool) -> list[str]:
    lines = _arduino_send_required("FE_SSR_ON" if on else "FE_SSR_OFF")
    event_logger.log_actuator_state("fume_extractor", bool(on))
    return lines


class DryerModbus:
    def __init__(self) -> None:
        self.client = ModbusSerialClient(
            port=DRYER_PORT,
            baudrate=DRYER_BAUDRATE,
            parity=DRYER_PARITY,
            stopbits=DRYER_STOPBITS,
            bytesize=DRYER_BYTESIZE,
            timeout=DRYER_TIMEOUT,
            retries=0,
        )
        self._lock = threading.Lock()
        self.connected = False
        self._target_setpoint_reached_logged_for: Optional[int] = None
        self._last_process_temp_raw: Optional[int] = None
        self._last_run_state_raw: Optional[int] = None
        self._target_reach_armed_for: Optional[int] = None
        self._last_running_state: Optional[bool] = None

    def connect(self) -> bool:
        with self._lock:
            self.connected = self.client.connect()
            return self.connected

    def ensure_connected(self) -> bool:
        if not self.connected:
            self.connect()
        return self.connected

    def close(self) -> None:
        with self._lock:
            self.client.close()
            self.connected = False
            self._target_setpoint_reached_logged_for = None
            self._last_process_temp_raw = None
            self._last_run_state_raw = None
            self._target_reach_armed_for = None
            self._last_running_state = None

    def read_input(self, addr: int, count: int = 1) -> list[int]:
        with self._lock:
            rr = self.client.read_input_registers(
                address=addr, count=count, device_id=DRYER_DEVICE_ID
            )
            if rr.isError():
                raise RuntimeError(str(rr))
            return rr.registers

    def read_holding(self, addr: int, count: int = 1) -> list[int]:
        with self._lock:
            rr = self.client.read_holding_registers(
                address=addr, count=count, device_id=DRYER_DEVICE_ID
            )
            if rr.isError():
                raise RuntimeError(str(rr))
            return rr.registers

    def write_holding(self, addr: int, value: int) -> None:
        with self._lock:
            wr = self.client.write_register(
                address=addr, value=value & 0xFFFF, device_id=DRYER_DEVICE_ID
            )
            if wr.isError():
                raise RuntimeError(str(wr))

    def pulse_holding(
        self,
        addr: int,
        value_on: int,
        value_off: int = 0,
        pulse_time: float = 0.3,
    ) -> None:
        self.write_holding(addr, value_on)
        time.sleep(pulse_time)
        self.write_holding(addr, value_off)

    # ── Register accessors ────────────────────────────────────────────────────

    def get_actual_dewpoint(self) -> int:   return _s16(self.read_input(12)[0])
    def get_status_word(self) -> int:       return self.read_input(20)[0]
    def get_process_setpoint(self) -> int:  return _s16(self.read_input(21)[0])
    def get_dewpoint_setpoint(self) -> int: return _s16(self.read_input(23)[0])
    def get_process_temp(self) -> int:      return _s16(self.read_input(1)[0])
    def get_regen_temp(self) -> int:        return _s16(self.read_input(3)[0])
    def get_regen_outlet_temp(self) -> int: return _s16(self.read_input(4)[0])
    def get_blower_inlet_temp(self) -> int: return _s16(self.read_input(5)[0])
    def get_run_state_raw(self) -> int:     return self.read_input(6)[0]
    def get_aux_temp(self) -> int:          return _s16(self.read_input(19)[0])
    def get_mode_state(self) -> int:        return self.read_input(0)[0]
    def get_output_word(self) -> int:       return self.read_input(25)[0]

    def set_process_setpoint(self, value: int) -> None:
        self.write_holding(21, _u16(value))

    def set_dewpoint_setpoint(self, value: int) -> None:
        self.write_holding(23, _u16(value))

    def toggle_on_off(self) -> None:
        before = self.get_run_state_raw()
        before_output = self.get_output_word()
        self.pulse_holding(addr=17, value_on=1, value_off=0, pulse_time=0.3)
        time.sleep(0.2)
        after = self.get_run_state_raw()
        after_output = self.get_output_word()
        before_running = self._classify_running_state(before, before_output, self._last_running_state)
        after_running = self._classify_running_state(after, after_output, before_running)
        if before_running is not None and after_running is not None and before_running != after_running:
            event_logger.log_actuator_state("dryer", after_running)
            if not after_running:
                with self._lock:
                    self._target_setpoint_reached_logged_for = None
                    self._target_reach_armed_for = None
                    self._last_process_temp_raw = None

    # ── Composite reads ───────────────────────────────────────────────────────

    @staticmethod
    def _classify_running_state(
        run_state: int,
        output_word: int,
        prev_running: Optional[bool],
    ) -> Optional[bool]:
        if run_state in DRYER_RUN_ON_VALUES:
            return True
        if run_state in DRYER_RUN_OFF_VALUES:
            return False
        if output_word == 0:
            return False
        return prev_running

    @staticmethod
    def _dryer_state_guess(run_state: int, running: Optional[bool]) -> str:
        if running is True:
            return "ON"
        if running is False:
            return "OFF"
        return f"UNKNOWN ({run_state})"

    @staticmethod
    def _bits_set(value: int) -> list[int]:
        return [i for i in range(16) if value & (1 << i)]

    def snapshot(self) -> dict:
        if not self.ensure_connected():
            raise RuntimeError("Failed to connect to Modbus device.")

        mode_state        = self.get_mode_state()
        run_state         = self.get_run_state_raw()
        process_temp      = self.get_process_temp()
        regen_temp        = self.get_regen_temp()
        regen_outlet_temp = self.get_regen_outlet_temp()
        blower_inlet_temp = self.get_blower_inlet_temp()
        aux_temp          = self.get_aux_temp()
        actual_dp         = self.get_actual_dewpoint()
        process_sp        = self.get_process_setpoint()
        dew_sp            = self.get_dewpoint_setpoint()
        status_word       = self.get_status_word()
        output_word       = self.get_output_word()
        process_temp_c    = process_temp / 10.0
        process_sp_c      = float(process_sp)
        process_sp_target_raw = int(round(process_sp_c * 10.0))

        should_log_target_reached = False
        state_edge_on: Optional[bool] = None
        running_state: Optional[bool] = None
        with self._lock:
            prev_temp = self._last_process_temp_raw
            prev_run = self._last_run_state_raw
            prev_running = self._last_running_state
            running_state = self._classify_running_state(run_state, output_word, prev_running)
            is_running = bool(running_state)

            if prev_running is not None and running_state is not None and prev_running != running_state:
                state_edge_on = running_state

            if is_running and process_sp > 0:
                # Arm logging only after we have observed temperature below the
                # active setpoint in the current run. This avoids startup false
                # positives while still capturing true reaches even if polling
                # skips the exact crossing sample.
                run_or_target_changed = (
                    prev_run == 0 or
                    self._target_reach_armed_for != process_sp
                )
                if run_or_target_changed:
                    self._target_reach_armed_for = process_sp if process_temp < process_sp_target_raw else None
                elif self._target_reach_armed_for is None and process_temp < process_sp_target_raw:
                    self._target_reach_armed_for = process_sp

                crossed_up = (
                    prev_temp is not None and
                    prev_temp < process_sp_target_raw and
                    process_temp >= process_sp_target_raw
                )

                if (
                    (
                        crossed_up or
                        self._target_reach_armed_for == process_sp
                    ) and
                    process_temp >= process_sp_target_raw and
                    self._target_setpoint_reached_logged_for != process_sp
                ):
                    should_log_target_reached = True
                    self._target_setpoint_reached_logged_for = process_sp
                    self._target_reach_armed_for = None
            else:
                self._target_setpoint_reached_logged_for = None
                self._target_reach_armed_for = None

            self._last_process_temp_raw = process_temp
            self._last_run_state_raw = run_state
            if running_state is not None:
                self._last_running_state = running_state

        if should_log_target_reached:
            event_logger.log_actuator_action(
                "dryer",
                "TARGET_SETPOINT_REACHED",
                process_temp_raw=process_temp,
                process_setpoint_raw=process_sp_target_raw,
                process_temp_c=round(process_temp_c, 1),
                process_setpoint_c=round(process_sp_c, 1),
                run_state=run_state,
            )

        if state_edge_on is not None:
            event_logger.log_actuator_state("dryer", state_edge_on)

        return {
            "connected":          self.connected,
            "port":               DRYER_PORT,
            "baudrate":           DRYER_BAUDRATE,
            "device_id":          DRYER_DEVICE_ID,
            "mode_state":         mode_state,
            "run_state":          run_state,
            "is_running":         running_state,
            "dryer_state_guess":  self._dryer_state_guess(run_state, running_state),
            "process_temp":       process_temp,
            "regen_temp":         regen_temp,
            "regen_outlet_temp":  regen_outlet_temp,
            "blower_inlet_temp":  blower_inlet_temp,
            "aux_temp":           aux_temp,
            "actual_dewpoint":    actual_dp,
            "process_setpoint":   process_sp,
            "dewpoint_setpoint":  dew_sp,
            "status_word":        status_word,
            "status_word_hex":    f"0x{status_word:04X}",
            "status_bits_set":    self._bits_set(status_word),
            "output_word":        output_word,
            "output_word_hex":    f"0x{output_word:04X}",
        }


dryer = DryerModbus()


# ─────────────────────────────────────────────────────────────────────────────
#  Routes — Dryer
# ─────────────────────────────────────────────────────────────────────────────

@app.route("/api/dryer/connect", methods=["POST"])
def api_dryer_connect():
    try:
        set_dryer_power(True)
        time.sleep(1.0)
        ok = dryer.connect()
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500
    return jsonify({
        "ok": ok,
        "connected": ok,
        "port": DRYER_PORT,
        "baudrate": DRYER_BAUDRATE,
        "device_id": DRYER_DEVICE_ID,
    })


@app.route("/api/dryer/disconnect", methods=["POST"])
def api_dryer_disconnect():
    dryer.close()
    try:
        set_dryer_power(False)
    except Exception:
        pass
    return jsonify({"ok": True, "connected": False})


@app.route("/api/dryer/status", methods=["GET"])
def api_dryer_status():
    try:
        return jsonify({"ok": True, "data": dryer.snapshot()})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/dryer/toggle", methods=["POST"])
def api_dryer_toggle():
    try:
        before = dryer.get_run_state_raw()
        dryer.toggle_on_off()
        time.sleep(0.5)
        after = dryer.get_run_state_raw()
        return jsonify({
            "ok": True,
            "before_run_state": before,
            "after_run_state":  after,
            "message": "Toggled dryer using 40018 bit 0 pulse.",
        })
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/fe/on", methods=["POST"])
def api_fe_on():
    try:
        lines = set_fume_extractor_power(True)
        return jsonify({"ok": True, "state": "ON", "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/fe/off", methods=["POST"])
def api_fe_off():
    try:
        lines = set_fume_extractor_power(False)
        return jsonify({"ok": True, "state": "OFF", "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/fe/status", methods=["GET"])
def api_fe_status():
    try:
        lines = _arduino_send_required("FE_SSR_STATUS")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/dryer/set_process_sp", methods=["POST"])
def api_dryer_set_process_sp():
    try:
        raw, celsius = _coerce_setpoint_to_raw_deci(request.json["value"])
        dryer.set_process_setpoint(raw)
        return jsonify({"ok": True, "value_raw": raw, "value_c": round(celsius, 1)})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/dryer/set_dewpoint_sp", methods=["POST"])
def api_dryer_set_dewpoint_sp():
    try:
        raw, celsius = _coerce_setpoint_to_raw_deci(request.json["value"])
        dryer.set_dewpoint_setpoint(raw)
        return jsonify({"ok": True, "value_raw": raw, "value_c": round(celsius, 1)})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/dryer/manual_read", methods=["POST"])
def api_dryer_manual_read():
    try:
        reg = int(request.json["register"])
        if 30001 <= reg <= 39999:
            raw = dryer.read_input(reg - 30001, 1)[0]
        elif 40001 <= reg <= 49999:
            raw = dryer.read_holding(reg - 40001, 1)[0]
        else:
            raise ValueError("Register must be in 3xxxx or 4xxxx range.")
        return jsonify({
            "ok":     True,
            "register": reg,
            "raw":    raw,
            "signed": _s16(raw),
            "hex":    f"0x{raw:04X}",
        })
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/dryer/manual_write", methods=["POST"])
def api_dryer_manual_write():
    try:
        reg   = int(request.json["register"])
        value = int(request.json["value"])
        if not (40001 <= reg <= 49999):
            raise ValueError("Manual writes only for 4xxxx holding registers.")
        dryer.write_holding(reg - 40001, _u16(value))
        return jsonify({"ok": True, "register": reg, "value": value})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


# ─────────────────────────────────────────────────────────────────────────────
#  Routes — Arduino connection
# ─────────────────────────────────────────────────────────────────────────────

@app.route("/api/arduino/connect", methods=["POST"])
def api_arduino_connect():
    try:
        port     = request.json.get("port", ARDUINO_PORT)
        baudrate = int(request.json.get("baudrate", ARDUINO_BAUDRATE))
        arduino.connect(port, baudrate)
        return jsonify({"ok": True, "connected": True, "port": port, "baudrate": baudrate})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/arduino/disconnect", methods=["POST"])
def api_arduino_disconnect():
    arduino.disconnect()
    return jsonify({"ok": True, "connected": False})


@app.route("/api/arduino/status", methods=["GET"])
def api_arduino_status():
    # Always answers ok:True (with a connected flag) so a transient serial
    # hiccup or an in-progress blast-gate move never trips the dashboard's
    # error handling — the connected flag alone drives the UI state.
    snap = arduino.status_for_api()
    return jsonify({
        "ok": True,
        "connected": snap["connected"],
        "cached": snap.get("cached", False),
        "data": snap.get("data", {}),
    })


@app.route("/api/arduino/command", methods=["POST"])
def api_arduino_command():
    """Send a raw serial command to the Arduino and return the reply lines.

    Powers the dashboard's manual command console — equivalent to typing a
    command in the Arduino Serial Monitor (e.g. STATUS, GATE_OPEN, MOTOR 128 FWD).
    """
    cmd = (request.json or {}).get("cmd", "")
    cmd = str(cmd).strip()
    if not cmd:
        return jsonify({"ok": False, "error": "empty command"}), 400
    if not arduino.connected:
        return jsonify({"ok": False, "error": "Arduino not connected."}), 409
    try:
        lines = arduino.send(cmd)
        return jsonify({"ok": True, "command": cmd, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


# ─────────────────────────────────────────────────────────────────────────────
#  Routes — Shredder Gate
# ─────────────────────────────────────────────────────────────────────────────

@app.route("/api/gate/open", methods=["POST"])
def api_gate_open():
    try:
        lines = arduino.send("GATE_OPEN")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/gate/close", methods=["POST"])
def api_gate_close():
    try:
        lines = arduino.send("GATE_CLOSE")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/gate/status", methods=["GET"])
def api_gate_status():
    try:
        lines = arduino.send("GATE_STATUS")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


# ─────────────────────────────────────────────────────────────────────────────
#  Routes — DC Motor
# ─────────────────────────────────────────────────────────────────────────────

@app.route("/api/motor/set", methods=["POST"])
def api_motor_set():
    """Body: { "speed": 0-255, "dir": "FWD" | "REV" }"""
    try:
        speed = int(request.json["speed"])
        direction = str(request.json["dir"]).upper()
        if not (0 <= speed <= 255):
            raise ValueError("speed must be 0-255")
        if direction not in ("FWD", "REV"):
            raise ValueError("dir must be FWD or REV")
        lines = arduino.send(f"MOTOR_SET {speed} {direction}")
        return jsonify({"ok": True, "speed": speed, "dir": direction, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/motor/stop", methods=["POST"])
def api_motor_stop():
    try:
        lines = arduino.send("MOTOR_STOP")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/motor/status", methods=["GET"])
def api_motor_status():
    try:
        lines = arduino.send("MOTOR_STATUS")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


# ─────────────────────────────────────────────────────────────────────────────
#  Routes — Trash conveyor (pick & place)
# ─────────────────────────────────────────────────────────────────────────────
@app.route("/api/tc/home", methods=["POST"])
def api_tc_home():
    try:
        lines = arduino.send("TC_HOME")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/tc/pick", methods=["POST"])
def api_tc_pick():
    try:
        lines = arduino.send("TC_PICK")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/tc/stop", methods=["POST"])
def api_tc_stop():
    try:
        lines = arduino.send("TC_STOP")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/tc/status", methods=["GET"])
def api_tc_status():
    try:
        lines = arduino.send("TC_STATUS")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/tc/pump/on", methods=["POST"])
def api_tc_pump_on():
    try:
        lines = arduino.send("TC_PUMP_ON")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/tc/pump/off", methods=["POST"])
def api_tc_pump_off():
    try:
        lines = arduino.send("TC_PUMP_OFF")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


# ─────────────────────────────────────────────────────────────────────────────
#  Routes — Shredder
# ─────────────────────────────────────────────────────────────────────────────

@app.route("/api/shredder/on", methods=["POST"])
def api_shredder_on():
    try:
        lines = arduino.send("SHREDDER_ON")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/shredder/off", methods=["POST"])
def api_shredder_off():
    try:
        lines = arduino.send("SHREDDER_OFF")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/shredder/fwd", methods=["POST"])
def api_shredder_fwd():
    try:
        lines = arduino.send("SHREDDER_FWD")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/shredder/rev", methods=["POST"])
def api_shredder_rev():
    try:
        lines = arduino.send("SHREDDER_REV")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


# ─────────────────────────────────────────────────────────────────────────────#  Routes — Air lock home sequence
# ──────────────────────────────────────────────────────────────────

@app.route("/api/agitator/stop", methods=["POST"])
def api_agitator_stop():
    try:
        if ratio_test.status().get("running"):
            return jsonify({"ok": False, "error": "extrusion_ratio_test is running; stop it first"}), 400
        lines = arduino.send("AGITATOR_STOP")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/agitator/status", methods=["POST"])
def api_agitator_status():
    try:
        lines = arduino.send("AGITATOR_STATUS")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/agitator/home", methods=["POST"])
def api_agitator_home():
    try:
        if ratio_test.status().get("running"):
            return jsonify({"ok": False, "error": "extrusion_ratio_test is running; stop it first"}), 400
        lines = arduino.send("AGITATOR_HOME")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


# ──────────────────────────────────────────────────────────────────
#  Routes — Mixer vacuum motor (DS3502 digipot speed control)
# ──────────────────────────────────────────────────────────────────

@app.route("/api/vacuum/set", methods=["POST"])
def api_vacuum_set():
    try:
        if ratio_test.status().get("running"):
            return jsonify({"ok": False, "error": "extrusion_ratio_test is running; stop it first"}), 400
        percent = int(request.json["percent"])
        if percent < 0 or percent > 100:
            raise ValueError("percent must be 0-100")
        lines = arduino.send(f"VACUUM_SET {percent}")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/vacuum/stop", methods=["POST"])
def api_vacuum_stop():
    try:
        lines = arduino.send("VACUUM_STOP")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/vacuum/status", methods=["POST"])
def api_vacuum_status():
    try:
        lines = arduino.send("VACUUM_STATUS")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


# ──────────────────────────────────────────────────────────────────
#  Routes — Mixer blast gates (RoboClaw linear actuators)
# ─────────────────────────────────────────────────────────────────────────────

def _blastgate_gate_arg(default: str = "ALL") -> str:
    """Read + validate a gate selector (L / R / ALL) from the JSON body."""
    gate = str((request.json or {}).get("gate", default)).strip().upper()
    if gate not in ("L", "R", "LEFT", "RIGHT", "ALL"):
        raise ValueError("gate must be L, R or ALL")
    return gate


@app.route("/api/blastgate/home", methods=["POST"])
def api_blastgate_home():
    try:
        lines = arduino.send(f"BLASTGATE_HOME {_blastgate_gate_arg()}")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/blastgate/homemax", methods=["POST"])
def api_blastgate_homemax():
    try:
        lines = arduino.send(f"BLASTGATE_HOMEMAX {_blastgate_gate_arg()}")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/blastgate/cal", methods=["POST"])
def api_blastgate_cal():
    try:
        lines = arduino.send(f"BLASTGATE_CAL {_blastgate_gate_arg()}")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/blastgate/pos", methods=["POST"])
def api_blastgate_pos():
    try:
        gate = _blastgate_gate_arg(default="")
        if gate in ("ALL", ""):
            raise ValueError("pos requires a single gate: L or R")
        pct = float(request.json["percent"])
        if pct < 0 or pct > 100:
            raise ValueError("percent must be 0-100")
        lines = arduino.send(f"BLASTGATE_POS {gate} {pct:g}")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/blastgate/jog", methods=["POST"])
def api_blastgate_jog():
    try:
        gate = _blastgate_gate_arg(default="")
        if gate in ("ALL", ""):
            raise ValueError("jog requires a single gate: L or R")
        direction = str((request.json or {}).get("dir", "")).strip().upper()
        if direction not in ("EXT", "RET"):
            raise ValueError("dir must be EXT or RET")
        ms = int(request.json["ms"])
        if ms <= 0:
            raise ValueError("ms must be > 0")
        lines = arduino.send(f"BLASTGATE_{direction} {gate} {ms}")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/blastgate/speed", methods=["POST"])
def api_blastgate_speed():
    try:
        pct = int(request.json["percent"])
        if pct < 1 or pct > 100:
            raise ValueError("percent must be 1-100")
        lines = arduino.send(f"BLASTGATE_SPEED {pct}")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/blastgate/stop", methods=["POST"])
def api_blastgate_stop():
    try:
        lines = arduino.send("BLASTGATE_STOP")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/blastgate/status", methods=["POST"])
def api_blastgate_status():
    try:
        lines = arduino.send("BLASTGATE_STATUS")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


# ── Automation (Size Reduction) ──────────────────────────────────────────────

def _parse_sizered(lines: list[str]) -> dict:
    """Parse the firmware's [SIZERED] key=value status line into a dict."""
    for line in lines:
        if line.startswith("[SIZERED]"):
            result: dict = {}
            for part in line.replace("[SIZERED]", "").strip().split():
                if "=" in part:
                    k, _, v = part.partition("=")
                    result[k] = v
            return result
    return {}


@app.route("/api/sr/start", methods=["POST"])
def api_sr_start():
    """Begin the Size Reduction shred sequence: SR_START <pe_units> <pa_units>."""
    try:
        body = request.get_json(silent=True) or {}
        pe = int(body.get("pe_units", 0))
        pa = int(body.get("pa_units", 0))
        if pe < 0 or pa < 0 or (pe + pa) <= 0:
            return jsonify({"ok": False, "error": "pe_units/pa_units must be >= 0 and not both zero"}), 400
        # Close the lower blast gates first so shredded material collects in the
        # mixer barrel instead of falling straight through. (Firmware SR_START
        # opens the shredder gates so shred can drop from the chute.)
        try:
            arduino.send(BLASTGATE_CLOSE_CMD)
        except Exception:
            pass
        lines = arduino.send(f"SR_START {pe} {pa}")
        return jsonify({"ok": True, "response": lines, "data": _parse_sizered(lines)})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/sr/stop", methods=["POST"])
def api_sr_stop():
    try:
        lines = arduino.send("SR_STOP")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/sr/status", methods=["GET"])
def api_sr_status():
    try:
        lines = arduino.send("SR_STATUS")
        return jsonify({"ok": True, "data": _parse_sizered(lines), "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


# ─────────────────────────────────────────────────────────────────────────────
#  Process automation — Drying / Mixing / Discharge + Print-feed metering
#
#  These phases follow Size Reduction in the LunaRecycle process and span both
#  the Arduino FPU (mixer screw motor, shredder gates, blast gates, agitator,
#  vacuum) and the Modbus dryer, so they are orchestrated here in the backend
#  instead of the firmware. Each runs on its own background thread and drives
#  the existing firmware primitives; nothing blocks the Flask request threads.
# ─────────────────────────────────────────────────────────────────────────────

# ── Drying / mixing tunables (override via environment) ──────────────────────
MIX_ON_SECONDS       = int(os.environ.get("LUNA_MIX_ON_SEC", "120"))       # mix 2 min
MIX_PERIOD_SECONDS   = int(os.environ.get("LUNA_MIX_PERIOD_SEC", "600"))   # every 10 min
MIX_PWM              = int(os.environ.get("LUNA_MIX_PWM", "150"))          # 0-255 mixer speed
MIX_RAMP_STEPS       = int(os.environ.get("LUNA_MIX_RAMP_STEPS", "5"))     # soft-start increments
MIX_RAMP_STEP_SEC    = float(os.environ.get("LUNA_MIX_RAMP_STEP_SEC", "0.20"))  # delay between increments
MIX_UP_DIR           = os.environ.get("LUNA_MIX_UP_DIR", "FWD").upper()    # upward mixing dir
MIX_DOWN_DIR         = "REV" if MIX_UP_DIR == "FWD" else "FWD"
DISCHARGE_SECONDS    = int(os.environ.get("LUNA_DISCHARGE_SEC", "60"))     # downward mix time
DISCHARGE_SHAKE      = os.environ.get("LUNA_DISCHARGE_SHAKE", "1").strip().lower() not in ("0", "false", "off", "no")
DISCHARGE_SHAKE_SEG_SEC = float(os.environ.get("LUNA_DISCHARGE_SHAKE_SEG_SEC", "5"))
DISCHARGE_SHAKE_STOP_SEC = float(os.environ.get("LUNA_DISCHARGE_SHAKE_STOP_SEC", "1.5"))
PREHEAT_LEAD_SECONDS = int(os.environ.get("LUNA_PREHEAT_LEAD_SEC", "1500"))  # 25 min printer lead
BLASTGATE_OPEN_CMD   = os.environ.get("LUNA_BG_OPEN_CMD", "BLASTGATE_HOMEMAX ALL")
BLASTGATE_CLOSE_CMD  = os.environ.get("LUNA_BG_CLOSE_CMD", "BLASTGATE_HOME ALL")

# ── Print-feed ratio-test-style defaults ─────────────────────────────────────
FEED_RATIO_SPINS_PER_HOME = int(os.environ.get("LUNA_FEED_RATIO_SPINS_PER_HOME", str(RATIO_TEST_SPINS_PER_HOME)))
FEED_POLL_SEC             = float(os.environ.get("LUNA_FEED_POLL_SEC", str(RATIO_TEST_POLL_SEC)))
FEED_VACUUM_PCT           = int(os.environ.get("LUNA_FEED_VACUUM_PCT", str(RATIO_TEST_VACUUM_IDLE_PCT)))


class ProcessOrchestrator:
    """Runs the Drying -> Discharge phase on a background thread.

    DRYING    - shredder gates closed, dryer ON at the requested setpoint, mixer
                screw runs upward for MIX_ON every MIX_PERIOD for the requested
                duration. A ``preheat_due`` flag rises PREHEAT_LEAD before the
                end so the operator/printer can begin barrel + bed heating.
    DISCHARGE - mixer stops, dryer OFF, lower blast gates open, mixer runs
                downward for DISCHARGE_SECONDS to drop material into the crammer.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._reset()

    def _reset(self):
        self.phase = "IDLE"          # IDLE, DRYING, DISCHARGE, DONE, ABORTED, ERROR
        self.started_at = 0.0
        self.dry_end_at = 0.0
        self.total_seconds = 0
        self.temp_c = 0
        self.mixing = False
        self.dryer_on = False
        self.preheat_due = False
        self.message = ""
        self.notes: list[str] = []

    # ── helpers (never raise into the run loop) ──────────────────────────────
    def _note(self, msg: str):
        with self._lock:
            self.notes.append(msg)
            self.notes = self.notes[-8:]

    def _arduino(self, cmd: str) -> list[str]:
        try:
            return arduino.send(cmd)
        except Exception as exc:
            self._note(f"arduino '{cmd}' failed: {exc}")
            return []

    def _motor_ramp(self, target_pwm: int, direction: str):
        """Soft-start the mixer instead of slamming straight to the target PWM.

        The screw motor is the remaining likely USB-drop trigger. Ramping in a
        few small steps reduces both startup current and brush EMI versus a
        single abrupt MOTOR_SET 150 FWD/REV.
        """
        target_pwm = max(0, min(255, int(target_pwm)))
        steps = max(1, int(MIX_RAMP_STEPS))
        step_delay = max(0.0, float(MIX_RAMP_STEP_SEC))

        if target_pwm == 0:
            self._arduino("MOTOR_STOP")
            return

        for idx in range(1, steps + 1):
            if self._stop.is_set():
                return
            pwm = max(1, int(round(target_pwm * idx / steps)))
            self._arduino(f"MOTOR_SET {pwm} {direction}")
            if idx < steps:
                self._stop.wait(step_delay)

    def _dryer_on(self, temp_c: int):
        try:
            set_dryer_power(True)
            delay_s = max(0.0, float(DRYER_SSR_PREHEAT_DELAY_SEC))
            if delay_s > 0:
                self._note(f"dryer SSR enabled; waiting {delay_s:.1f}s before heater start")
                if self._stop.wait(delay_s):
                    self._note("dryer start cancelled during SSR preheat delay")
                    return
            if not dryer.connected:
                dryer.connect()
            # Automated drying uses operator-entered C directly.
            dryer.set_process_setpoint(int(round(float(temp_c))))
            if dryer.get_run_state_raw() != 100:
                dryer.toggle_on_off()
            with self._lock:
                self.dryer_on = True
            event_logger.log_actuator_state("dryer", True)
        except Exception as exc:
            self._note(f"dryer ON failed: {exc}")

    def _dryer_off(self):
        try:
            if dryer.connected and dryer.get_run_state_raw() == 100:
                dryer.toggle_on_off()
        except Exception as exc:
            self._note(f"dryer OFF failed: {exc}")
        finally:
            try:
                set_dryer_power(False)
            except Exception as exc:
                self._note(f"dryer SSR OFF failed: {exc}")
            with self._lock:
                self.dryer_on = False
            event_logger.log_actuator_state("dryer", False)

    def _run_discharge_motion(self):
        """Run discharge motion, optionally alternating directions to shake down material."""
        end = time.monotonic() + DISCHARGE_SECONDS

        if not DISCHARGE_SHAKE:
            self._motor_ramp(MIX_PWM, MIX_DOWN_DIR)
            with self._lock:
                self.mixing = True
            while not self._stop.is_set() and time.monotonic() < end:
                self._stop.wait(0.5)
            return

        seg = max(0.5, float(DISCHARGE_SHAKE_SEG_SEC))
        stop_pause = max(0.0, float(DISCHARGE_SHAKE_STOP_SEC))
        direction_idx = 0
        directions = (MIX_DOWN_DIR, MIX_UP_DIR)
        prev_direction = None

        while not self._stop.is_set() and time.monotonic() < end:
            direction = directions[direction_idx % 2]

            # Keep the very first segment behavior as-is, but for subsequent
            # shake reversals follow the same pattern as the initial dry->
            # discharge transition: stop first, hold briefly at zero, then
            # soft-ramp into the new direction.
            if prev_direction is not None and direction != prev_direction:
                self._arduino("MOTOR_STOP")
                with self._lock:
                    self.mixing = False
                if stop_pause > 0.0:
                    self._stop.wait(stop_pause)

            self._motor_ramp(MIX_PWM, direction)
            with self._lock:
                self.mixing = True
            phase_end = min(end, time.monotonic() + seg)
            while not self._stop.is_set() and time.monotonic() < phase_end:
                self._stop.wait(0.25)
            prev_direction = direction
            direction_idx += 1

    # ── background run loop ──────────────────────────────────────────────────
    def _run(self, total_seconds: int, temp_c: int):
        try:
            self._arduino("GATE_CLOSE")           # trap dried air in the barrel
            self._arduino(BLASTGATE_CLOSE_CMD)    # keep material in until discharge
            self._dryer_on(temp_c)

            now = time.monotonic()
            with self._lock:
                self.phase = "DRYING"
                self.started_at = now
                self.dry_end_at = now + total_seconds
            cycle_start = now
            mixing = False

            while not self._stop.is_set():
                now = time.monotonic()
                remaining = self.dry_end_at - now
                if remaining <= 0:
                    break
                with self._lock:
                    self.preheat_due = remaining <= PREHEAT_LEAD_SECONDS
                # Mixer cadence: MIX_ON seconds of upward mixing per MIX_PERIOD.
                phase_t = (now - cycle_start) % MIX_PERIOD_SECONDS
                want_mix = phase_t < MIX_ON_SECONDS
                if want_mix and not mixing:
                    self._motor_ramp(MIX_PWM, MIX_UP_DIR)
                    mixing = True
                    with self._lock:
                        self.mixing = True
                elif not want_mix and mixing:
                    self._arduino("MOTOR_STOP")
                    mixing = False
                    with self._lock:
                        self.mixing = False
                self._stop.wait(1.0)

            if mixing:
                self._arduino("MOTOR_STOP")
                with self._lock:
                    self.mixing = False

            if self._stop.is_set():
                self._dryer_off()
                with self._lock:
                    self.phase = "ABORTED"
                    self.message = "Drying aborted by operator."
                return

            # ── Discharge: dryer off, open gates, run mixer downward ──────────
            with self._lock:
                self.phase = "DISCHARGE"
                if DISCHARGE_SHAKE:
                    self.message = "Discharging: gates open, shaker alternating mixer direction."
                else:
                    self.message = "Discharging: gates open, mixer reversing down."
            self._dryer_off()
            self._arduino(BLASTGATE_OPEN_CMD)
            self._run_discharge_motion()
            self._arduino("MOTOR_STOP")
            with self._lock:
                self.mixing = False
                if self._stop.is_set():
                    self.phase = "ABORTED"
                    self.message = "Discharge aborted by operator."
                else:
                    self.phase = "DONE"
                    self.message = "Drying + discharge complete."

        except Exception as exc:
            self._note(f"fatal: {exc}")
            self._arduino("MOTOR_STOP")
            self._dryer_off()
            with self._lock:
                self.phase = "ERROR"
                self.message = f"Error: {exc}"

    # ── public API ───────────────────────────────────────────────────────────
    def start(self, total_seconds: int, temp_c: int):
        with self._lock:
            if self.phase in ("DRYING", "DISCHARGE"):
                raise RuntimeError("Drying cycle already running.")
            self._reset()
            self.total_seconds = int(total_seconds)
            self.temp_c = int(temp_c)
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, args=(int(total_seconds), int(temp_c)), daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()

    def status(self) -> dict:
        with self._lock:
            now = time.monotonic()
            running = self.phase in ("DRYING", "DISCHARGE")
            elapsed = int(now - self.started_at) if self.started_at else 0
            remaining = max(0, int(self.dry_end_at - now)) if self.phase == "DRYING" else 0
            return {
                "phase":             self.phase,
                "running":           running,
                "total_seconds":     self.total_seconds,
                "elapsed_seconds":   elapsed if running else 0,
                "remaining_seconds": remaining,
                "temp_c":            self.temp_c,
                "mixing":            self.mixing,
                "dryer_on":          self.dryer_on,
                "preheat_due":       self.preheat_due,
                "message":           self.message,
                "notes":             list(self.notes),
            }


class FeedController:
    """Feed wrapper that reuses the extrusion ratio test behavior.

    Feed start/stop/status remains on /api/feed/* for workflow continuity, but
    execution is delegated to the same Moonraker-driven ratio watcher used by
    /api/extrusion_ratio_test/*.
    """

    def start(self, spins_per_home: int, vacuum_pct: int, poll_sec: Optional[float] = None) -> None:
        ratio_test.start(
            spins_per_home=max(1, int(spins_per_home)),
            poll_sec=FEED_POLL_SEC if poll_sec is None else max(0.1, float(poll_sec)),
            vacuum_pct=max(0, min(100, int(vacuum_pct))),
        )

    def stop(self) -> None:
        ratio_test.stop()

    def status(self) -> dict:
        rs = ratio_test.status()
        return {
            "phase": rs.get("phase", "IDLE"),
            "running": bool(rs.get("running")),
            "ratio": int(rs.get("spins_per_home", 0) or 0),
            "spins_per_home": int(rs.get("spins_per_home", 0) or 0),
            "vacuum_pct": int(rs.get("vacuum_pct", 0) or 0),
            "moonraker_connected": bool(rs.get("moonraker_connected")),
            "live_extruder_velocity": float(rs.get("live_extruder_velocity", 0.0) or 0.0),
            "live_extruder_rotations_total": float(rs.get("live_extruder_rotations_total", 0.0) or 0.0),
            "rotations_since_home": float(rs.get("rotations_since_home", 0.0) or 0.0),
            "rotations_until_home": float(rs.get("rotations_until_home", 0.0) or 0.0),
            "home_sequences_completed": int(rs.get("home_sequences_completed", 0) or 0),
            "message": str(rs.get("message", "")),
            "last_error": str(rs.get("last_error", "")),
        }


orchestrator = ProcessOrchestrator()
feeder = FeedController()


class ExtrusionRatioTestController:
    """Watch Moonraker extrusion rotations and home the air lock on a spin threshold."""

    def __init__(self):
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._reset()

    def _reset(self):
        self.phase = "IDLE"
        self.message = "Ready."
        self.running = False
        self.started_at = 0.0
        self.spins_per_home = max(1, int(RATIO_TEST_SPINS_PER_HOME))
        self.poll_sec = max(0.1, float(RATIO_TEST_POLL_SEC))
        self.vacuum_pct = max(0, min(100, RATIO_TEST_VACUUM_IDLE_PCT))
        self.moonraker_connected = False
        self.live_extruder_velocity = 0.0
        self.live_extruder_rotations_total = 0.0
        self.rotations_since_home = 0.0
        self.rotations_until_home = float(self.spins_per_home)
        self.home_sequences_completed = 0
        self.last_home_rotations_total = 0.0
        self.last_home_response: list[str] = []
        self.last_error = ""
        self.last_sample_rotations_total: Optional[float] = None

    def _arduino(self, cmd: str) -> list[str]:
        try:
            return arduino.send(cmd)
        except Exception as exc:
            raise RuntimeError(f"arduino '{cmd}' failed: {exc}") from exc

    def _set_vacuum(self, percent: int) -> list[str]:
        percent = max(0, min(100, int(percent)))
        if percent <= 0:
            return self._arduino("VACUUM_STOP")
        return self._arduino(f"VACUUM_SET {percent}")

    def _set_mixer(self, pwm: int, direction: str = "FWD") -> list[str]:
        pwm = max(0, min(255, int(pwm)))
        if pwm <= 0:
            return self._arduino("MOTOR_STOP")
        direction = str(direction).strip().upper()
        if direction not in ("FWD", "REV"):
            direction = "FWD"
        return self._arduino(f"MOTOR_SET {pwm} {direction}")

    def _set_blastgates_open(self, open_state: bool) -> list[str]:
        return self._arduino(BLASTGATE_OPEN_CMD if open_state else BLASTGATE_CLOSE_CMD)

    def _wait_or_stop(self, seconds: float) -> bool:
        return self._stop.wait(max(0.0, float(seconds)))

    def _run_post_home_sequence(self) -> bool:
        """Run the requested mixer + blastgate choreography after each home."""
        self._set_blastgates_open(False)
        if self._wait_or_stop(RATIO_TEST_BG_SETTLE_SEC):
            return False

        self._set_mixer(0)
        if self._wait_or_stop(RATIO_TEST_STOP_SETTLE_SEC):
            return False

        self._set_mixer(RATIO_TEST_MIX_PWM, "REV")
        if self._wait_or_stop(RATIO_TEST_REVERSE_SEC):
            return False

        self._set_mixer(0)
        if self._wait_or_stop(RATIO_TEST_STOP_SETTLE_SEC):
            return False

        self._set_mixer(RATIO_TEST_MIX_PWM, "FWD")
        if self._wait_or_stop(RATIO_TEST_FORWARD_RECOVER_SEC):
            return False

        self._set_blastgates_open(True)
        return True

    def start(self, spins_per_home: Optional[int] = None, poll_sec: Optional[float] = None, vacuum_pct: Optional[int] = None):
        with self._lock:
            if self.running:
                raise RuntimeError("Extruder ratio test already running.")
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("Extruder ratio test is still stopping.")
            self._reset()
            if spins_per_home is not None:
                self.spins_per_home = max(1, int(spins_per_home))
            if poll_sec is not None:
                self.poll_sec = max(0.1, float(poll_sec))
            if vacuum_pct is not None:
                self.vacuum_pct = max(0, min(100, int(vacuum_pct)))
            self.rotations_until_home = float(self.spins_per_home)
            self.running = True
            self.phase = "ARMED"
            self.started_at = time.monotonic()
            self.message = (
                f"Watching Moonraker; homing every {self.spins_per_home} spins "
                f"with vacuum at {self.vacuum_pct}% and mixer FWD {RATIO_TEST_MIX_PWM}."
            )

        self._stop.clear()
        self._set_vacuum(self.vacuum_pct)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        try:
            self._arduino("VACUUM_STOP")
        except Exception:
            pass
        try:
            self._arduino("MOTOR_STOP")
        except Exception:
            pass
        with self._lock:
            self.running = False
            if self.phase != "ERROR":
                self.phase = "STOPPED"
                self.message = "Stopped."

    def _sample_snapshot(self) -> tuple[bool, float]:
        snap = moonraker.snapshot()
        connected = bool(snap.get("connected"))
        rotations_total_raw = snap.get("extruder_rotations_total", 0.0)
        try:
            rotations_total = float(rotations_total_raw)
        except (TypeError, ValueError):
            rotations_total = 0.0
        with self._lock:
            self.moonraker_connected = connected
            self.live_extruder_velocity = float(snap.get("live_extruder_velocity", 0.0) or 0.0)
            self.live_extruder_rotations_total = rotations_total
        return connected, rotations_total

    def _trigger_home(self, rotations_total: float):
        with self._lock:
            self.phase = "HOMING"
            self.message = f"Threshold reached at {rotations_total:.4f} spins; running air-lock home sequence."

        response = self._arduino("AGITATOR_HOME")

        with self._lock:
            self.phase = "POST_HOME_SEQUENCE"
            self.message = "Home complete. Running post-home blastgate/mixer sequence."

        completed = self._run_post_home_sequence()

        with self._lock:
            self.last_home_response = response
            self.home_sequences_completed += 1
            self.last_home_rotations_total = rotations_total
            self.rotations_since_home = 0.0
            self.rotations_until_home = float(self.spins_per_home)
            self.last_sample_rotations_total = rotations_total
            self.phase = "WATCHING"
            if completed:
                self.message = (
                    f"Homed {self.home_sequences_completed} time(s); post-home sequence complete. "
                    f"Waiting for the next {self.spins_per_home} spins."
                )
            else:
                self.message = "Post-home sequence interrupted by stop request."

    def _run(self):
        try:
            # Requested baseline behavior: keep mixer forward at 200 and gates open
            # while waiting for each spin-threshold home event.
            self._set_blastgates_open(True)
            self._set_mixer(RATIO_TEST_MIX_PWM, "FWD")

            while not self._stop.is_set():
                connected, rotations_total = self._sample_snapshot()
                should_wait = False

                with self._lock:
                    if not connected:
                        self.phase = "WAITING"
                        self.message = "Waiting for Moonraker extrusion data."
                        should_wait = True
                    elif self.last_sample_rotations_total is None:
                        self.last_sample_rotations_total = rotations_total
                        self.phase = "WATCHING"
                        self.message = f"Armed. Homing every {self.spins_per_home} spins."
                    else:
                        delta = rotations_total - self.last_sample_rotations_total
                        self.last_sample_rotations_total = rotations_total
                        if delta < 0.0 or delta > 1000.0:
                            self.rotations_since_home = 0.0
                        else:
                            self.rotations_since_home += delta
                        self.rotations_until_home = max(0.0, float(self.spins_per_home) - self.rotations_since_home)

                if should_wait:
                    self._stop.wait(self.poll_sec)
                    continue

                with self._lock:
                    ready = self.rotations_since_home >= self.spins_per_home

                if ready:
                    self._trigger_home(rotations_total)
                    self._stop.wait(self.poll_sec)
                    continue

                self._stop.wait(self.poll_sec)
        except Exception as exc:
            with self._lock:
                self.phase = "ERROR"
                self.running = False
                self.last_error = str(exc)
                self.message = f"Error: {exc}"
        finally:
            try:
                self._arduino("VACUUM_STOP")
            except Exception:
                pass
            try:
                self._arduino("MOTOR_STOP")
            except Exception:
                pass
            with self._lock:
                self.running = False
                self._thread = None

    def status(self) -> dict:
        with self._lock:
            return {
                "phase": self.phase,
                "running": self.running,
                "message": self.message,
                "spins_per_home": self.spins_per_home,
                "poll_sec": self.poll_sec,
                "vacuum_pct": self.vacuum_pct,
                "moonraker_connected": self.moonraker_connected,
                "live_extruder_velocity": self.live_extruder_velocity,
                "live_extruder_rotations_total": self.live_extruder_rotations_total,
                "rotations_since_home": self.rotations_since_home,
                "rotations_until_home": self.rotations_until_home,
                "home_sequences_completed": self.home_sequences_completed,
                "last_home_rotations_total": self.last_home_rotations_total,
                "last_home_response": list(self.last_home_response),
                "last_error": self.last_error,
            }


ratio_test = ExtrusionRatioTestController()


@app.route("/api/moonraker/status", methods=["GET"])
def api_moonraker_status():
    return jsonify({"ok": True, "data": moonraker.snapshot()})


@app.route("/api/extrusion_ratio_test/start", methods=["POST"])
def api_extrusion_ratio_test_start():
    try:
        body = request.get_json(silent=True) or {}
        ratio_test.start(
            spins_per_home=body.get("spins_per_home"),
            poll_sec=body.get("poll_sec"),
            vacuum_pct=body.get("vacuum_pct"),
        )
        return jsonify({"ok": True, "data": ratio_test.status()})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/extrusion_ratio_test/stop", methods=["POST"])
def api_extrusion_ratio_test_stop():
    try:
        ratio_test.stop()
        return jsonify({"ok": True, "data": ratio_test.status()})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/extrusion_ratio_test/status", methods=["GET"])
def api_extrusion_ratio_test_status():
    return jsonify({"ok": True, "data": ratio_test.status()})


@app.route("/api/dry/start", methods=["POST"])
def api_dry_start():
    """Begin the drying + mixing cycle. Body: {minutes, temp_c}."""
    try:
        body = request.get_json(silent=True) or {}
        minutes = float(body.get("minutes", 0))
        temp_c = int(body.get("temp_c", 0))
        if minutes <= 0:
            return jsonify({"ok": False, "error": "minutes must be > 0"}), 400
        if temp_c <= 0 or temp_c > 250:
            return jsonify({"ok": False, "error": "temp_c must be 1-250"}), 400
        orchestrator.start(int(minutes * 60), temp_c)
        return jsonify({"ok": True, "data": orchestrator.status()})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/dry/stop", methods=["POST"])
def api_dry_stop():
    try:
        orchestrator.stop()
        return jsonify({"ok": True, "data": orchestrator.status()})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/dry/status", methods=["GET"])
def api_dry_status():
    return jsonify({"ok": True, "data": orchestrator.status()})


@app.route("/api/feed/start", methods=["POST"])
def api_feed_start():
    """Start feed using ratio-test behavior. Body: {ratio|spins_per_home, vacuum_pct}."""
    try:
        body = request.get_json(silent=True) or {}
        ratio_raw = body.get("ratio", body.get("spins_per_home", FEED_RATIO_SPINS_PER_HOME))
        ratio = int(ratio_raw)
        poll_sec = body.get("poll_sec", None)
        vacuum_pct   = int(body.get("vacuum_pct", FEED_VACUUM_PCT))
        if ratio <= 0:
            return jsonify({"ok": False, "error": "ratio must be > 0"}), 400
        if not (0 <= vacuum_pct <= 100):
            return jsonify({"ok": False, "error": "vacuum_pct must be 0-100"}), 400
        feeder.start(spins_per_home=ratio, vacuum_pct=vacuum_pct, poll_sec=poll_sec)
        return jsonify({"ok": True, "data": feeder.status()})
    except RuntimeError as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/feed/stop", methods=["POST"])
def api_feed_stop():
    try:
        feeder.stop()
        return jsonify({"ok": True, "data": feeder.status()})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/feed/status", methods=["GET"])
def api_feed_status():
    return jsonify({"ok": True, "data": feeder.status()})


# ─────────────────────────────────────────────────────────────────────────────
#  Routes — Energy monitor
# ─────────────────────────────────────────────────────────────────────────────

@app.route("/api/energy/snapshot", methods=["GET"])
def api_energy_snapshot():
    """Sends STATUS and returns the parsed [ENERGY] line."""
    try:
        lines = arduino.send("STATUS")
        for line in lines:
            if line.startswith("[ENERGY]"):
                return jsonify({"ok": True, "data": arduino._parse_energy_line(line)})
        return jsonify({"ok": True, "data": {}, "note": "No energy data in response"})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


# ─────────────────────────────────────────────────────────────────────────────
#  Routes — System
# ─────────────────────────────────────────────────────────────────────────────
@app.route("/api/health", methods=["GET"])
def api_health():
    """Fast, serial-free health probe for the dashboard connection watchdog."""
    return jsonify({
        "ok": True,
        "arduino_connected": arduino.connected,
        "dryer_connected": dryer.connected,
        "moonraker": moonraker.snapshot(),
        "event_log": event_logger.info(),
    })

@app.route("/api/estop", methods=["POST"])
def api_estop():
    """Send ESTOP to Arduino and toggle off the dryer if connected."""
    errors: list[str] = []

    # Stop the background automation threads FIRST. Otherwise the drying
    # orchestrator / feed controller keep re-issuing MOTOR_SET (and other)
    # commands every second, overriding the ESTOP within ~1 s so the operator
    # "can't control anything" and the mixer keeps spinning.
    try:
        orchestrator.stop()
    except Exception as exc:
        errors.append(f"Drying stop: {exc}")
    try:
        feeder.stop()
    except Exception as exc:
        errors.append(f"Feed stop: {exc}")
    try:
        ratio_test.stop()
    except Exception as exc:
        errors.append(f"Ratio-test stop: {exc}")

    try:
        arduino.send("ESTOP")
    except Exception as exc:
        errors.append(f"Arduino ESTOP: {exc}")

    try:
        if dryer.connected:
            run_state = dryer.get_run_state_raw()
            if run_state == 100:
                dryer.toggle_on_off()
    except Exception as exc:
        errors.append(f"Dryer ESTOP: {exc}")

    try:
        set_dryer_power(False)
    except Exception as exc:
        errors.append(f"Dryer SSR: {exc}")

    try:
        set_fume_extractor_power(False)
    except Exception as exc:
        errors.append(f"FE SSR: {exc}")

    event_logger.mark_all_off()

    return jsonify({
        "ok":     len(errors) == 0,
        "errors": errors,
        "message": "ESTOP issued to all connected subsystems.",
    })



# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print(f"LunaRecycle backend starting on http://{SERVER_HOST}:{SERVER_PORT}")
    print(f"  Arduino: {ARDUINO_PORT} @ {ARDUINO_BAUDRATE} baud  (Gate servos, DC motor, INA219)")
    print(f"  Dryer  : {DRYER_PORT}   @ {DRYER_BAUDRATE} baud, ID {DRYER_DEVICE_ID}")
    print(f"  Event log: {event_logger.info()['path']}")

    # Best-effort auto-connect at boot; the supervisor keeps retrying if the
    # Arduino is unplugged or resets, so the dashboard never needs a manual
    # reconnect for a transient USB glitch.
    try:
        arduino.connect(ARDUINO_PORT, ARDUINO_BAUDRATE)
        print(f"  Arduino connected on {ARDUINO_PORT}")
    except Exception as exc:
        print(f"  Arduino not connected yet ({exc}); supervisor will retry")
    arduino.start_supervisor()

    if moonraker.enabled:
        print(f"  Moonraker WS: {MOONRAKER_WS_URL}")
        moonraker.start()
    else:
        print("  Moonraker WS: disabled (set LUNA_MOONRAKER_WS_URL to enable)")

    app.run(host=SERVER_HOST, port=SERVER_PORT, debug=False, threaded=True)
