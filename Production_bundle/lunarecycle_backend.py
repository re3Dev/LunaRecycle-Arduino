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
AGITATOR_MOVE_TIMEOUT = float(os.environ.get("LUNA_AGITATOR_MOVE_TIMEOUT", "45.0"))
# How often the background supervisor retries a dropped Arduino connection.
RECONNECT_INTERVAL = float(os.environ.get("LUNA_RECONNECT_INTERVAL", "3.0"))

DRYER_PORT      = os.environ.get("LUNA_DRYER_PORT", "/dev/ttyUSB0")
DRYER_BAUDRATE  = int(os.environ.get("LUNA_DRYER_BAUD", "57600"))
DRYER_DEVICE_ID = int(os.environ.get("LUNA_DRYER_ID", "1"))
DRYER_PARITY    = os.environ.get("LUNA_DRYER_PARITY", "N")
DRYER_STOPBITS  = int(os.environ.get("LUNA_DRYER_STOPBITS", "1"))
DRYER_BYTESIZE  = int(os.environ.get("LUNA_DRYER_BYTESIZE", "8"))
DRYER_TIMEOUT   = 0.5    # seconds

# Persistent event log path (JSON Lines). One event per line with CST timestamp.
EVENT_LOG_PATH = os.environ.get(
    "LUNA_EVENT_LOG_PATH",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "luna_actuator_events.jsonl"),
)
EVENT_LOG_TZ = timezone(timedelta(hours=-6), name="CST")

# Server bind. 0.0.0.0 makes the dashboard reachable from other devices on the
# LAN (e.g. http://<pi-ip>:5055). Set LUNA_HOST=127.0.0.1 to restrict to the Pi.
SERVER_HOST = os.environ.get("LUNA_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("LUNA_PORT", "5055"))

# Read-only viewer runtime config (served via /api/viewer/config).
VIEWER_DRYER_CAM_URL   = os.environ.get("LUNA_VIEWER_DRYER_CAM_URL", "")
VIEWER_PRINTER_CAM_URL = os.environ.get("LUNA_VIEWER_PRINTER_CAM_URL", "")
VIEWER_PRINTER_API_URL = os.environ.get("LUNA_VIEWER_PRINTER_API_URL", "")

# Moonraker (Klipper) live extrusion monitoring.
MOONRAKER_WS_URL = os.environ.get("LUNA_MOONRAKER_WS_URL", "").strip()
MOONRAKER_EXTRUDER_VELOCITY_MIN = float(os.environ.get("LUNA_MOONRAKER_EXTRUDER_VEL_MIN", "0.01"))
MOONRAKER_EXTRUDER_UNITS_PER_ROTATION = float(os.environ.get("LUNA_MOONRAKER_EXTRUDER_UNITS_PER_ROTATION", "1.0"))
RATIO_TEST_VACUUM_BOOST_SEC = float(os.environ.get("LUNA_RATIO_TEST_VACUUM_BOOST_SEC", "5.0"))
RATIO_TEST_VACUUM_IDLE_PCT = int(os.environ.get("LUNA_RATIO_TEST_VACUUM_IDLE_PCT", "15"))
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
    """Thread-safe, append-only logger for actuator ON/OFF edge events only."""

    def __init__(self, path: str) -> None:
        self._path = path
        self._lock = threading.Lock()
        self._last_error = ""
        self._last_state: dict[str, bool] = {}
        # Ensure the destination directory exists so first-write cannot fail
        # silently when a custom path points to a missing folder.
        folder = os.path.dirname(os.path.abspath(self._path))
        if folder:
            os.makedirs(folder, exist_ok=True)
        self._archive_legacy_log_if_needed()
        self._ensure_log_file_exists()

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
                    if '"event_type"' in s or '"source"' in s or '"action"' in s:
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
            "mode": "actuator_edges_only",
            "timezone": "CST",
            "exists": exists,
            "size_bytes": size,
            "last_error": self._last_error,
        }

    def log_actuator_state(self, actuator: str, is_on: bool) -> None:
        try:
            with self._lock:
                prev = self._last_state.get(actuator)
                if prev is not None and prev == is_on:
                    return
                event = {
                    "ts": datetime.now(EVENT_LOG_TZ).strftime("%Y-%m-%dT%H:%M:%S"),
                    "actuator": actuator,
                    "state": "ON" if is_on else "OFF",
                }
                with open(self._path, "a", encoding="utf-8") as fp:
                    fp.write(json.dumps(event, separators=(",", ":"), ensure_ascii=True) + "\n")
                self._last_state[actuator] = is_on
                self._last_error = ""
        except Exception:
            self._last_error = "failed_to_write_event_log"
            # Logging must never break control flow.
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

    def __init__(self, ws_url: str) -> None:
        self.ws_url = ws_url
        self.enabled = bool(ws_url)
        self._lock = threading.Lock()
        self.connected = False
        self.extruding = False
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
                self.live_extruder_velocity = 0.0
                self.axis_map = []
                self.extruder_axis_index = -1
                self.live_position = []
                self.live_extruder_position = 0.0
                self.extruder_units_total = 0.0
                self.extruder_rotations_total = 0.0
                self._last_live_extruder_position = None
                self.extruder_temps = {}
                self.max_extruder_temp_c = 0.0

    def _apply_status(self, status: dict) -> None:
        motion = status.get("motion_report", {}) if isinstance(status, dict) else {}
        gmove = status.get("gcode_move", {}) if isinstance(status, dict) else {}
        stats = status.get("print_stats", {}) if isinstance(status, dict) else {}

        velocity_raw = motion.get("live_extruder_velocity")
        try:
            velocity = float(velocity_raw)
        except (TypeError, ValueError):
            velocity = None

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

        with self._lock:
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

            # Collect any reported extruder heater temperatures.
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
                self.extruder_temps[obj_name] = temp_c
            self.max_extruder_temp_c = max(self.extruder_temps.values()) if self.extruder_temps else 0.0

            # Extruding means a meaningful live extrusion velocity.
            vel = abs(self.live_extruder_velocity)
            self.extruding = vel >= MOONRAKER_EXTRUDER_VELOCITY_MIN

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
            return ["extruder"]
        names = [str(name) for name in objects if isinstance(name, str) and name.startswith("extruder")]
        # Keep deterministic order and dedupe.
        names = sorted(set(names))
        return names or ["extruder"]

    def _run(self) -> None:
        while True:
            ws = None
            try:
                ws = websocket.create_connection(self.ws_url, timeout=5)

                # Discover available extruder heater objects first.
                ws.send(json.dumps({"jsonrpc": "2.0", "method": "printer.objects.list", "id": 2}))
                extruder_objects = ["extruder"]
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
        elif cmd == "AGITATOR_STOP":
            event_logger.log_actuator_state("agitator", False)
        elif cmd == "AGITATOR_SET":
            pct = 0
            if len(parts) > 1:
                try:
                    pct = int(parts[1])
                except ValueError:
                    pct = 0
            event_logger.log_actuator_state("agitator", pct > 0)
        elif cmd == "AGITATOR_MOVE":
            pwm = 0
            if len(parts) > 3:
                try:
                    pwm = int(parts[3])
                except ValueError:
                    pwm = 0
            event_logger.log_actuator_state("agitator", pwm > 0)
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
        is_agitator_move = cmd.strip().upper().startswith("AGITATOR_MOVE ")
        is_agitator_cal = cmd.strip().upper().startswith("AGITATOR_ENC_CAL")
        if is_blastgate:
            total_timeout = BLASTGATE_TIMEOUT
        elif is_agitator_move or is_agitator_cal:
            total_timeout = AGITATOR_MOVE_TIMEOUT
        else:
            total_timeout = ARDUINO_TIMEOUT
        # Tags that mark a genuine command reply. [ENERGY] is async telemetry the
        # firmware streams every 500 ms; for non-STATUS commands it must be skipped
        # so it isn't mistaken for the command's response (e.g. TC_PUMP_ON replies
        # with a [TC] line, not [ENERGY]).
        reply_tags = ("[STATUS]", "[GATE]", "[MOTOR]", "[TC]", "[SHREDDER]", "[AGITATOR]", "[AGITATOR_ENC]", "[AGITATOR_ENC_CAL_DONE]", "[VACUUM]", "[BLASTGATE_DONE]", "[SIZERED]", "[SYSTEM]")

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
                if is_agitator_cal:
                    if line.startswith("[AGITATOR_ENC_CAL_DONE]"):
                        break
                    continue
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
        self.pulse_holding(addr=17, value_on=1, value_off=0, pulse_time=0.3)
        time.sleep(0.2)
        after = self.get_run_state_raw()
        if before in (0, 100) and after in (0, 100) and before != after:
            event_logger.log_actuator_state("dryer", after == 100)

    # ── Composite reads ───────────────────────────────────────────────────────

    @staticmethod
    def _dryer_state_guess(run_state: int) -> str:
        if run_state == 0:
            return "OFF"
        if run_state == 100:
            return "ON"
        return f"Unknown ({run_state})"

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

        return {
            "connected":          self.connected,
            "port":               DRYER_PORT,
            "baudrate":           DRYER_BAUDRATE,
            "device_id":          DRYER_DEVICE_ID,
            "mode_state":         mode_state,
            "run_state":          run_state,
            "dryer_state_guess":  self._dryer_state_guess(run_state),
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
        value = int(request.json["value"])
        dryer.set_process_setpoint(value)
        return jsonify({"ok": True, "value": value})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/dryer/set_dewpoint_sp", methods=["POST"])
def api_dryer_set_dewpoint_sp():
    try:
        value = int(request.json["value"])
        dryer.set_dewpoint_setpoint(value)
        return jsonify({"ok": True, "value": value})
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


# ─────────────────────────────────────────────────────────────────────────────#  Routes — Mixer agitator (2nd H-bridge channel)
# ──────────────────────────────────────────────────────────────────

@app.route("/api/agitator/set", methods=["POST"])
def api_agitator_set():
    try:
        if ratio_test.status().get("running"):
            return jsonify({"ok": False, "error": "extrusion_ratio_test is running; stop it first"}), 400
        percent = int(request.json["percent"])
        direction = str(request.json.get("dir", "FWD")).strip().upper()
        if percent < 0 or percent > 100:
            raise ValueError("percent must be 0-100")
        if direction not in ("FWD", "REV"):
            raise ValueError("dir must be FWD or REV")
        lines = arduino.send(f"AGITATOR_SET {percent} {direction}")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/agitator/stop", methods=["POST"])
def api_agitator_stop():
    try:
        if ratio_test.status().get("running"):
            return jsonify({"ok": False, "error": "extrusion_ratio_test is running; stop it first"}), 400
        lines = arduino.send("AGITATOR_STOP")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/agitator/move", methods=["POST"])
def api_agitator_move():
    try:
        if ratio_test.status().get("running"):
            return jsonify({"ok": False, "error": "extrusion_ratio_test is running; stop it first"}), 400
        spins = int(request.json["spins"])
        pause_ms = int(request.json.get("pause_ms", 0))
        pwm = int(request.json["pwm"])
        direction = str(request.json.get("dir", "FWD")).strip().upper()
        pwm_after_start_ms = request.json.get("pwm_after_start_ms")
        pwm_after = request.json.get("pwm_after")

        if spins <= 0:
            raise ValueError("spins must be > 0")
        if pause_ms < 0 or pause_ms > 600000:
            raise ValueError("pause_ms must be 0-600000")
        if pwm < 0 or pwm > 255:
            raise ValueError("pwm must be 0-255")
        if direction not in ("FWD", "REV"):
            raise ValueError("dir must be FWD or REV")

        use_pwm_shift = pwm_after_start_ms is not None or pwm_after is not None
        if use_pwm_shift:
            if pwm_after_start_ms is None or pwm_after is None:
                raise ValueError("pwm_after_start_ms and pwm_after must both be provided")
            pwm_after_start_ms = int(pwm_after_start_ms)
            pwm_after = int(pwm_after)
            if pwm_after_start_ms < 0 or pwm_after_start_ms > 600000:
                raise ValueError("pwm_after_start_ms must be 0-600000")
            if pwm_after < 0 or pwm_after > 255:
                raise ValueError("pwm_after must be 0-255")

        cmd = f"AGITATOR_MOVE {spins} {pause_ms} {pwm} {direction}"
        if use_pwm_shift:
            cmd += f" {pwm_after_start_ms} {pwm_after}"

        lines = arduino.send(cmd)
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


@app.route("/api/agitator/encoder/home", methods=["POST"])
def api_agitator_encoder_home():
    try:
        if ratio_test.status().get("running"):
            return jsonify({"ok": False, "error": "extrusion_ratio_test is running; stop it first"}), 400
        body = request.get_json(silent=True) or {}
        pwm = int(body.get("pwm", 120))
        if pwm < 1 or pwm > 255:
            raise ValueError("pwm must be 1-255")
        lines = arduino.send(f"AGITATOR_ENC_HOME {pwm}")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/agitator/encoder/calibrate", methods=["POST"])
def api_agitator_encoder_calibrate():
    try:
        if ratio_test.status().get("running"):
            return jsonify({"ok": False, "error": "extrusion_ratio_test is running; stop it first"}), 400
        body = request.get_json(silent=True) or {}
        pwm = int(body.get("pwm", 120))
        revs = int(body.get("revs", 3))
        if pwm < 45 or pwm > 255:
            raise ValueError("pwm must be 45-255")
        if revs < 1 or revs > 10:
            raise ValueError("revs must be 1-10")
        lines = arduino.send(f"AGITATOR_ENC_CAL {pwm} {revs}")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/agitator/encoder/goto", methods=["POST"])
def api_agitator_encoder_goto():
    try:
        if ratio_test.status().get("running"):
            return jsonify({"ok": False, "error": "extrusion_ratio_test is running; stop it first"}), 400
        body = request.get_json(silent=True) or {}
        target_count = int(body["target_count"])
        pwm_max = int(body.get("pwm_max", 220))
        if target_count < 0 or target_count > 7427:
            raise ValueError("target_count must be 0-7427")
        if pwm_max < 45 or pwm_max > 255:
            raise ValueError("pwm_max must be 45-255")
        lines = arduino.send(f"AGITATOR_GOTO {target_count} {pwm_max}")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/agitator/encoder/goto_tolerance", methods=["POST"])
def api_agitator_encoder_goto_tolerance():
    try:
        if ratio_test.status().get("running"):
            return jsonify({"ok": False, "error": "extrusion_ratio_test is running; stop it first"}), 400
        body = request.get_json(silent=True) or {}
        undershoot_counts = int(body.get("undershoot_counts", 1))
        overshoot_counts = int(body.get("overshoot_counts", 4))
        if undershoot_counts < 0 or undershoot_counts > 31:
            raise ValueError("undershoot_counts must be 0-31")
        if overshoot_counts < 0 or overshoot_counts > 31:
            raise ValueError("overshoot_counts must be 0-31")
        cmd = f"AGITATOR_GOTO_TOL {undershoot_counts} {overshoot_counts}"
        lines = arduino.send(cmd)
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/agitator/encoder/home_trim", methods=["POST"])
def api_agitator_encoder_home_trim():
    try:
        if ratio_test.status().get("running"):
            return jsonify({"ok": False, "error": "extrusion_ratio_test is running; stop it first"}), 400
        body = request.get_json(silent=True) or {}
        counts = int(body.get("counts", 0))
        if counts < 0 or counts > 7427:
            raise ValueError("counts must be 0-7427")
        lines = arduino.send(f"AGITATOR_HOME_TRIM {counts}")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/agitator/encoder/pid", methods=["POST"])
def api_agitator_encoder_pid():
    try:
        if ratio_test.status().get("running"):
            return jsonify({"ok": False, "error": "extrusion_ratio_test is running; stop it first"}), 400
        body = request.get_json(silent=True) or {}
        kp = float(body["kp"])
        ki = float(body["ki"])
        kd = float(body["kd"])
        if kp < 0 or ki < 0 or kd < 0:
            raise ValueError("kp, ki, kd must be >= 0")
        lines = arduino.send(f"AGITATOR_PID {kp:.6f} {ki:.6f} {kd:.6f}")
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/agitator/encoder/pid_autotune", methods=["POST"])
def api_agitator_encoder_pid_autotune():
    try:
        if ratio_test.status().get("running"):
            return jsonify({"ok": False, "error": "extrusion_ratio_test is running; stop it first"}), 400
        body = request.get_json(silent=True) or {}
        target_count = int(body["target_count"])
        pwm_lo = int(body.get("pwm_lo", 120))
        pwm_hi = int(body.get("pwm_hi", 180))
        cycles = int(body.get("cycles", 6))

        if target_count < 0 or target_count > 7427:
            raise ValueError("target_count must be 0-7427")
        if pwm_lo < 110 or pwm_lo > 255:
            raise ValueError("pwm_lo must be 110-255")
        if pwm_hi < 45 or pwm_hi > 255:
            raise ValueError("pwm_hi must be 45-255")
        if pwm_hi < (pwm_lo + 20):
            raise ValueError("pwm_hi must be at least pwm_lo+20")
        if cycles < 4 or cycles > 20:
            raise ValueError("cycles must be 4-20")

        cmd = f"AGITATOR_PID_AUTOTUNE {target_count} {pwm_lo} {pwm_hi} {cycles}"
        lines = arduino.send(cmd)
        return jsonify({"ok": True, "response": lines})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/agitator/encoder/status", methods=["POST"])
def api_agitator_encoder_status():
    try:
        lines = arduino.send("AGITATOR_ENC_STATUS")
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

# ── Print-feed metering tunables ─────────────────────────────────────────────
FEED_PRIME_SECONDS        = int(os.environ.get("LUNA_FEED_PRIME_SEC", "10"))
FEED_METER_ON_SECONDS     = int(os.environ.get("LUNA_FEED_METER_ON_SEC", "3"))
FEED_METER_PERIOD_SECONDS = int(os.environ.get("LUNA_FEED_METER_PERIOD_SEC", "20"))
FEED_VACUUM_PCT           = int(os.environ.get("LUNA_FEED_VACUUM_PCT", "40"))
FEED_AGITATOR_PCT         = int(os.environ.get("LUNA_FEED_AGITATOR_PCT", "75"))
FEED_AGITATOR_DIR         = os.environ.get("LUNA_FEED_AGITATOR_DIR", "REV").upper()


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
            time.sleep(1.0)
            if not dryer.connected:
                dryer.connect()
            dryer.set_process_setpoint(int(temp_c))
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
    """Print-feed prime + intermittent metering on a background thread.

    PRIME  - vacuum + agitator both run for FEED_PRIME_SECONDS to charge the
             film crammer before the print starts.
    METER  - vacuum stays on (compaction); the agitator pulses on for
             FEED_METER_ON every FEED_METER_PERIOD to meter material in.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._reset()

    def _reset(self):
        self.phase = "IDLE"          # IDLE, PRIMING, METERING, STOPPED, ERROR
        self.started_at = 0.0
        self.vacuum_pct = 0
        self.agitator_pct = 0
        self.agitator_on = False
        self.moonraker_connected = False
        self.moonraker_extruding = False
        self.message = ""

    def _arduino(self, cmd: str):
        try:
            arduino.send(cmd)
        except Exception as exc:
            with self._lock:
                self.message = f"arduino '{cmd}' failed: {exc}"

    def _run(self, prime_sec, meter_on, meter_period, vacuum_pct, agitator_pct):
        try:
            with self._lock:
                self.phase = "PRIMING"
                self.started_at = time.monotonic()
                self.vacuum_pct = vacuum_pct
                self.agitator_pct = agitator_pct
                self.message = "Priming crammer (vacuum + agitator)."
            self._arduino(f"VACUUM_SET {vacuum_pct}")

            use_moonraker_gating = moonraker.enabled
            if use_moonraker_gating:
                # Moonraker-controlled mode: keep vacuum continuously on and only
                # run the agitator when extrusion is actively happening.
                with self._lock:
                    self.phase = "METERING"
                    self.message = "Metering gated by live extruder activity."
                while not self._stop.is_set():
                    snap = moonraker.snapshot()
                    want = bool(snap.get("extruding"))
                    connected = bool(snap.get("connected"))
                    with self._lock:
                        on = self.agitator_on
                        self.moonraker_connected = connected
                        self.moonraker_extruding = want
                    if want and not on:
                        self._arduino(f"AGITATOR_SET {agitator_pct} {FEED_AGITATOR_DIR}")
                        with self._lock:
                            self.agitator_on = True
                    elif not want and on:
                        self._arduino("AGITATOR_STOP")
                        with self._lock:
                            self.agitator_on = False
                    self._stop.wait(0.2)

                self._arduino("AGITATOR_STOP")
                self._arduino("VACUUM_STOP")
                with self._lock:
                    self.agitator_on = False
                    self.phase = "STOPPED"
                    self.message = "Feed stopped."
                return

            self._arduino(f"AGITATOR_SET {agitator_pct} {FEED_AGITATOR_DIR}")
            with self._lock:
                self.agitator_on = True
            end = time.monotonic() + prime_sec
            while not self._stop.is_set() and time.monotonic() < end:
                self._stop.wait(0.2)

            # ── Metering: vacuum steady, agitator pulses ─────────────────────
            with self._lock:
                self.phase = "METERING"
                self.message = "Metering material to the crammer."
            self._arduino("AGITATOR_STOP")
            with self._lock:
                self.agitator_on = False
            cycle_start = time.monotonic()
            while not self._stop.is_set():
                phase_t = (time.monotonic() - cycle_start) % meter_period
                want = phase_t < meter_on
                with self._lock:
                    on = self.agitator_on
                if want and not on:
                    self._arduino(f"AGITATOR_SET {agitator_pct} {FEED_AGITATOR_DIR}")
                    with self._lock:
                        self.agitator_on = True
                elif not want and on:
                    self._arduino("AGITATOR_STOP")
                    with self._lock:
                        self.agitator_on = False
                self._stop.wait(0.5)

            self._arduino("AGITATOR_STOP")
            self._arduino("VACUUM_STOP")
            with self._lock:
                self.agitator_on = False
                self.phase = "STOPPED"
                self.message = "Feed stopped."
        except Exception as exc:
            self._arduino("AGITATOR_STOP")
            self._arduino("VACUUM_STOP")
            with self._lock:
                self.phase = "ERROR"
                self.message = f"Error: {exc}"

    def start(self, prime_sec, meter_on, meter_period, vacuum_pct, agitator_pct):
        with self._lock:
            if self.phase in ("PRIMING", "METERING"):
                raise RuntimeError("Feed already running.")
            self._reset()
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            args=(prime_sec, meter_on, meter_period, vacuum_pct, agitator_pct),
            daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()

    def status(self) -> dict:
        with self._lock:
            running = self.phase in ("PRIMING", "METERING")
            return {
                "phase":        self.phase,
                "running":      running,
                "vacuum_pct":   self.vacuum_pct,
                "agitator_pct": self.agitator_pct,
                "agitator_on":  self.agitator_on,
                "moonraker_connected": self.moonraker_connected,
                "moonraker_extruding": self.moonraker_extruding,
                "message":      self.message,
            }


orchestrator = ProcessOrchestrator()
feeder = FeedController()


class ExtrusionRatioTestController:
    """Manual test mode: run vacuum + hall-homed agitator based on live extruder ratio.

    This is independent from /api/feed automation and intended for operator tuning.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._reset()

    def _reset(self):
        self.phase = "IDLE"     # IDLE, RUNNING, STOPPED, ERROR
        self.message = ""
        self.running = False
        self.started_at = 0.0
        self.vacuum_pct = 0
        self.vacuum_active_pct = 0
        self.vacuum_idle_pct = max(0, min(100, RATIO_TEST_VACUUM_IDLE_PCT))
        self.vacuum_boost_sec = max(0.0, RATIO_TEST_VACUUM_BOOST_SEC)
        self.vacuum_boost_remaining_sec = 0.0
        self.agitator_pwm = 0
        self.agitator_pwm_after_start_ms = -1
        self.agitator_pwm_after = 0
        self.agitator_dir = "FWD"
        self.agitator_pause_ms = 0
        self.extruder_rotations_per_cycle = 10.0
        self.agitator_rotations_per_cycle = 1
        self.extruder_rot_pending = 0.0
        self.agitator_rotations_commanded = 0
        self.agitator_cycles_completed = 0
        self.mixer_every_cycles = 0
        self.mixer_eligible = False
        self.mixer_active = False
        self.moonraker_connected = False
        self.live_extruder_velocity = 0.0
        self.live_extruder_rotations_total = 0.0

    def _arduino(self, cmd: str):
        arduino.send(cmd)

    def _run(self, extruder_rot_per_cycle, agitator_rot_per_cycle, agitator_pwm, agitator_pwm_after_start_ms, agitator_pwm_after, agitator_dir, agitator_pause_ms, vacuum_pct, mixer_every_cycles):
        baseline_rot = None
        vacuum_boost_until = 0.0
        vacuum_idle_pct = max(0, min(100, RATIO_TEST_VACUUM_IDLE_PCT))
        vacuum_active_pct = vacuum_idle_pct
        try:
            with self._lock:
                self.phase = "RUNNING"
                self.running = True
                self.started_at = time.monotonic()
                self.vacuum_pct = vacuum_pct
                self.vacuum_active_pct = vacuum_active_pct
                self.vacuum_idle_pct = vacuum_idle_pct
                self.vacuum_boost_sec = max(0.0, RATIO_TEST_VACUUM_BOOST_SEC)
                self.vacuum_boost_remaining_sec = 0.0
                self.agitator_pwm = agitator_pwm
                self.agitator_pwm_after_start_ms = agitator_pwm_after_start_ms
                self.agitator_pwm_after = agitator_pwm_after
                self.agitator_dir = agitator_dir
                self.agitator_pause_ms = agitator_pause_ms
                self.extruder_rotations_per_cycle = extruder_rot_per_cycle
                self.agitator_rotations_per_cycle = agitator_rot_per_cycle
                self.extruder_rot_pending = 0.0
                self.agitator_rotations_commanded = 0
                self.agitator_cycles_completed = 0
                self.mixer_every_cycles = mixer_every_cycles
                self.mixer_eligible = False
                self.mixer_active = False
                self.message = "Waiting for Moonraker extrusion data."

            # Start at idle vacuum. Boost to configured vacuum_pct only for a
            # short window after each completed agitator spin command.
            self._arduino(f"VACUUM_SET {vacuum_idle_pct}")
            self._arduino("MOTOR_STOP")

            while not self._stop.is_set():
                now = time.monotonic()
                snap = moonraker.snapshot()
                connected = bool(snap.get("connected"))
                try:
                    live_vel = float(snap.get("live_extruder_velocity", 0.0) or 0.0)
                except (TypeError, ValueError):
                    live_vel = 0.0
                try:
                    live_rot = float(snap.get("extruder_rotations_total", 0.0) or 0.0)
                except (TypeError, ValueError):
                    live_rot = 0.0

                with self._lock:
                    self.moonraker_connected = connected
                    self.live_extruder_velocity = live_vel
                    self.live_extruder_rotations_total = live_rot
                    self.vacuum_boost_remaining_sec = max(0.0, vacuum_boost_until - now)

                if not connected:
                    with self._lock:
                        self.message = "Moonraker disconnected; waiting to resume."
                    self._stop.wait(0.2)
                    continue

                if baseline_rot is None or live_rot < baseline_rot:
                    baseline_rot = live_rot
                    self._stop.wait(0.1)
                    continue

                delta_rot = max(0.0, live_rot - baseline_rot)
                baseline_rot = live_rot

                if delta_rot > 0.0:
                    with self._lock:
                        self.extruder_rot_pending += delta_rot

                run_cycle = False
                with self._lock:
                    if self.extruder_rot_pending >= self.extruder_rotations_per_cycle:
                        # Fire one burst when threshold is crossed, then drop
                        # backlog so we do not "catch up" with endless queued
                        # bursts after a long AGITATOR_MOVE cycle.
                        self.extruder_rot_pending = 0.0
                        run_cycle = True

                if run_cycle:
                    cmd = f"AGITATOR_MOVE {agitator_rot_per_cycle} {agitator_pause_ms} {agitator_pwm} {agitator_dir}"
                    if agitator_pwm_after_start_ms >= 0:
                        cmd += f" {agitator_pwm_after_start_ms} {agitator_pwm_after}"
                    self._arduino(cmd)

                    # After a completed spin cycle, hold configured vacuum for
                    # a short window, then return to idle vacuum.
                    self._arduino(f"VACUUM_SET {vacuum_pct}")
                    vacuum_active_pct = vacuum_pct
                    vacuum_boost_until = time.monotonic() + max(0.0, RATIO_TEST_VACUUM_BOOST_SEC)

                    mixer_started = False
                    with self._lock:
                        self.agitator_cycles_completed += 1
                        self.agitator_rotations_commanded += agitator_rot_per_cycle
                        if (
                            self.mixer_every_cycles > 0
                            and self.agitator_cycles_completed >= self.mixer_every_cycles
                        ):
                            self.mixer_eligible = True
                        if self.mixer_eligible and not self.mixer_active:
                            self.mixer_active = True
                            mixer_started = True
                        self.vacuum_active_pct = vacuum_active_pct
                        self.vacuum_boost_remaining_sec = max(0.0, vacuum_boost_until - time.monotonic())
                    if mixer_started:
                        self._arduino("MOTOR_SET 200 FWD")

                    with self._lock:
                        mixer_note = " Mixer running at 200 PWM FWD." if self.mixer_active else ""
                        self.message = (
                            f"Agitated {self.agitator_rotations_commanded} turns across "
                            f"{self.agitator_cycles_completed} cycles at ratio "
                            f"{self.extruder_rotations_per_cycle}:{self.agitator_rotations_per_cycle}."
                            f"{mixer_note}"
                        )
                else:
                    if vacuum_active_pct != vacuum_idle_pct and time.monotonic() >= vacuum_boost_until:
                        self._arduino(f"VACUUM_SET {vacuum_idle_pct}")
                        vacuum_active_pct = vacuum_idle_pct
                        with self._lock:
                            if self.mixer_active:
                                self.mixer_active = False
                                self._arduino("MOTOR_STOP")
                    with self._lock:
                        self.vacuum_active_pct = vacuum_active_pct
                        self.vacuum_boost_remaining_sec = max(0.0, vacuum_boost_until - time.monotonic())
                        mixer_note = (
                            " Mixer ready for blower windows."
                            if self.mixer_eligible and not self.mixer_active
                            else f" Mixer armed for cycle {self.mixer_every_cycles}."
                            if self.mixer_every_cycles > 0 and not self.mixer_eligible
                            else " Mixer running at 200 PWM FWD."
                            if self.mixer_active
                            else ""
                        )
                        self.message = (
                            f"Running ratio test ({self.extruder_rotations_per_cycle}:"
                            f"{self.agitator_rotations_per_cycle})."
                            f"{mixer_note}"
                        )

                self._stop.wait(0.1)

            self._arduino("AGITATOR_STOP")
            self._arduino("MOTOR_STOP")
            self._arduino("VACUUM_STOP")
            with self._lock:
                self.phase = "STOPPED"
                self.running = False
                self.mixer_eligible = False
                self.mixer_active = False
                self.message = "Extruder-ratio test stopped."
        except Exception as exc:
            try:
                self._arduino("AGITATOR_STOP")
                self._arduino("MOTOR_STOP")
                self._arduino("VACUUM_STOP")
            except Exception:
                pass
            with self._lock:
                self.phase = "ERROR"
                self.running = False
                self.mixer_eligible = False
                self.mixer_active = False
                self.message = f"Error: {exc}"

    def start(self, extruder_rot_per_cycle, agitator_rot_per_cycle, agitator_pwm, agitator_pwm_after_start_ms, agitator_pwm_after, agitator_dir, agitator_pause_ms, vacuum_pct, mixer_every_cycles):
        with self._lock:
            if self.running:
                raise RuntimeError("Extruder-ratio test already running.")
            self._reset()
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            args=(extruder_rot_per_cycle, agitator_rot_per_cycle, agitator_pwm, agitator_pwm_after_start_ms, agitator_pwm_after, agitator_dir, agitator_pause_ms, vacuum_pct, mixer_every_cycles),
            daemon=True,
        )
        self._thread.start()

    def stop(self):
        self._stop.set()

    def status(self) -> dict:
        with self._lock:
            return {
                "phase": self.phase,
                "running": self.running,
                "message": self.message,
                "vacuum_pct": self.vacuum_pct,
                "vacuum_active_pct": self.vacuum_active_pct,
                "vacuum_idle_pct": self.vacuum_idle_pct,
                "vacuum_boost_sec": self.vacuum_boost_sec,
                "vacuum_boost_remaining_sec": self.vacuum_boost_remaining_sec,
                "agitator_pwm": self.agitator_pwm,
                "agitator_pwm_after_start_ms": self.agitator_pwm_after_start_ms,
                "agitator_pwm_after": self.agitator_pwm_after,
                "agitator_dir": self.agitator_dir,
                "agitator_pause_ms": self.agitator_pause_ms,
                "extruder_rotations_per_cycle": self.extruder_rotations_per_cycle,
                "agitator_rotations_per_cycle": self.agitator_rotations_per_cycle,
                "agitator_consecutive_rotations": self.agitator_rotations_per_cycle,
                "extruder_rot_pending": self.extruder_rot_pending,
                "agitator_rotations_commanded": self.agitator_rotations_commanded,
                "agitator_cycles_completed": self.agitator_cycles_completed,
                "mixer_every_cycles": self.mixer_every_cycles,
                "mixer_eligible": self.mixer_eligible,
                "mixer_active": self.mixer_active,
                "moonraker_connected": self.moonraker_connected,
                "live_extruder_velocity": self.live_extruder_velocity,
                "live_extruder_rotations_total": self.live_extruder_rotations_total,
            }


ratio_test = ExtrusionRatioTestController()


@app.route("/api/moonraker/status", methods=["GET"])
def api_moonraker_status():
    return jsonify({"ok": True, "data": moonraker.snapshot()})


@app.route("/api/extrusion_ratio_test/start", methods=["POST"])
def api_extrusion_ratio_test_start():
    try:
        if not moonraker.enabled:
            return jsonify({"ok": False, "error": "Moonraker is disabled (set LUNA_MOONRAKER_WS_URL)."}), 400
        if feeder.status().get("running"):
            return jsonify({"ok": False, "error": "feed automation is running; stop it first"}), 400

        body = request.get_json(silent=True) or {}
        extruder_rot_per_cycle = float(body.get("extruder_rotations_per_cycle", 10.0))
        agitator_rot_per_cycle = int(
            body.get(
                "agitator_consecutive_rotations",
                body.get("agitator_rotations_per_cycle", 1),
            )
        )
        agitator_pwm = int(body.get("agitator_pwm", 180))
        agitator_pwm_after_start_ms = int(body.get("agitator_pwm_after_start_ms", -1))
        agitator_pwm_after = int(body.get("agitator_pwm_after", 0))
        agitator_dir = str(body.get("agitator_dir", "FWD")).strip().upper()
        agitator_pause_ms = int(body.get("agitator_pause_ms", 0))
        vacuum_pct = int(body.get("vacuum_pct", 40))
        mixer_every_cycles = int(body.get("mixer_every_cycles", 0))

        if extruder_rot_per_cycle <= 0:
            return jsonify({"ok": False, "error": "extruder_rotations_per_cycle must be > 0"}), 400
        if agitator_rot_per_cycle <= 0:
            return jsonify({"ok": False, "error": "agitator_rotations_per_cycle must be > 0"}), 400
        if not (0 <= agitator_pwm <= 255):
            return jsonify({"ok": False, "error": "agitator_pwm must be 0-255"}), 400
        if not (-1 <= agitator_pwm_after_start_ms <= 600000):
            return jsonify({"ok": False, "error": "agitator_pwm_after_start_ms must be -1 or 0-600000"}), 400
        if not (0 <= agitator_pwm_after <= 255):
            return jsonify({"ok": False, "error": "agitator_pwm_after must be 0-255"}), 400
        if agitator_dir not in ("FWD", "REV"):
            return jsonify({"ok": False, "error": "agitator_dir must be FWD or REV"}), 400
        if not (0 <= agitator_pause_ms <= 600000):
            return jsonify({"ok": False, "error": "agitator_pause_ms must be 0-600000"}), 400
        if not (0 <= vacuum_pct <= 100):
            return jsonify({"ok": False, "error": "vacuum_pct must be 0-100"}), 400
        if mixer_every_cycles < 0:
            return jsonify({"ok": False, "error": "mixer_every_cycles must be >= 0"}), 400

        ratio_test.start(
            extruder_rot_per_cycle,
            agitator_rot_per_cycle,
            agitator_pwm,
            agitator_pwm_after_start_ms,
            agitator_pwm_after,
            agitator_dir,
            agitator_pause_ms,
            vacuum_pct,
            mixer_every_cycles,
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
    """Begin print-feed prime + metering. Body overrides are optional."""
    try:
        if ratio_test.status().get("running"):
            return jsonify({"ok": False, "error": "extrusion_ratio_test is running; stop it first"}), 400
        body = request.get_json(silent=True) or {}
        prime_sec    = int(body.get("prime_sec", FEED_PRIME_SECONDS))
        meter_on     = int(body.get("meter_on_sec", FEED_METER_ON_SECONDS))
        meter_period = int(body.get("meter_period_sec", FEED_METER_PERIOD_SECONDS))
        vacuum_pct   = int(body.get("vacuum_pct", FEED_VACUUM_PCT))
        agitator_pct = int(body.get("agitator_pct", FEED_AGITATOR_PCT))
        if not (0 <= vacuum_pct <= 100):
            return jsonify({"ok": False, "error": "vacuum_pct must be 0-100"}), 400
        if not (0 <= agitator_pct <= 100):
            return jsonify({"ok": False, "error": "agitator_pct must be 0-100"}), 400
        if meter_period <= 0 or meter_on < 0 or meter_on > meter_period:
            return jsonify({"ok": False, "error": "meter timing invalid"}), 400
        feeder.start(prime_sec, meter_on, meter_period, vacuum_pct, agitator_pct)
        return jsonify({"ok": True, "data": feeder.status()})
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
