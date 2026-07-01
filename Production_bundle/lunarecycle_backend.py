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

import os
import threading
import time
from typing import Optional

import serial
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
ARDUINO_BAUDRATE = int(os.environ.get("LUNA_ARDUINO_BAUD", "9600"))
ARDUINO_TIMEOUT  = 2.0   # seconds for blocking read-until-response

DRYER_PORT      = os.environ.get("LUNA_DRYER_PORT", "/dev/ttyUSB0")
DRYER_BAUDRATE  = int(os.environ.get("LUNA_DRYER_BAUD", "57600"))
DRYER_DEVICE_ID = int(os.environ.get("LUNA_DRYER_ID", "1"))
DRYER_PARITY    = os.environ.get("LUNA_DRYER_PARITY", "N")
DRYER_STOPBITS  = int(os.environ.get("LUNA_DRYER_STOPBITS", "1"))
DRYER_BYTESIZE  = int(os.environ.get("LUNA_DRYER_BYTESIZE", "8"))
DRYER_TIMEOUT   = 0.5    # seconds

# Server bind. 0.0.0.0 makes the dashboard reachable from other devices on the
# LAN (e.g. http://<pi-ip>:5055). Set LUNA_HOST=127.0.0.1 to restrict to the Pi.
SERVER_HOST = os.environ.get("LUNA_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("LUNA_PORT", "5055"))

# ─────────────────────────────────────────────────────────────────────────────
#  Flask app
# ─────────────────────────────────────────────────────────────────────────────

app = Flask(__name__)
CORS(app)

_BUNDLE_DIR = os.path.dirname(os.path.abspath(__file__))


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

# ─────────────────────────────────────────────────────────────────────────────
#  Arduino serial bridge
# ─────────────────────────────────────────────────────────────────────────────

class ArduinoBridge:
    """Thread-safe serial bridge to the Arduino running LunaRecycle.ino."""

    def __init__(self) -> None:
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self.connected = False
        self._response_lines: list[str] = []

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self, port: str = ARDUINO_PORT, baudrate: int = ARDUINO_BAUDRATE) -> bool:
        with self._lock:
            if self._ser and self._ser.is_open:
                return True
            try:
                self._ser = serial.Serial(port, baudrate, timeout=ARDUINO_TIMEOUT)
                time.sleep(2.0)          # wait for Arduino reset after DTR pulse
                self._ser.reset_input_buffer()
                self.connected = True
                return True
            except serial.SerialException as exc:
                self.connected = False
                raise RuntimeError(str(exc)) from exc

    def disconnect(self) -> None:
        with self._lock:
            if self._ser and self._ser.is_open:
                self._ser.close()
            self._ser = None
            self.connected = False

    # ── Low-level I/O ─────────────────────────────────────────────────────────

    def _send_command(self, cmd: str) -> list[str]:
        """Send a command and collect response lines until a terminal tag or timeout."""
        if not self._ser or not self._ser.is_open:
            raise RuntimeError("Arduino not connected.")

        self._ser.reset_input_buffer()
        self._ser.write((cmd + "\n").encode("ascii"))
        self._ser.flush()

        is_status = cmd.strip().upper() == "STATUS"
        # Tags that mark a genuine command reply. [ENERGY] is async telemetry the
        # firmware streams every 500 ms; for non-STATUS commands it must be skipped
        # so it isn't mistaken for the command's response (e.g. TC_PUMP_ON replies
        # with a [TC] line, not [ENERGY]).
        reply_tags = ("[STATUS]", "[GATE]", "[MOTOR]", "[TC]", "[SHREDDER]", "[SYSTEM]")

        lines: list[str] = []
        seen_status = False
        deadline = time.monotonic() + ARDUINO_TIMEOUT
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
                if line.startswith(reply_tags):
                    break
            else:
                time.sleep(0.01)

        return lines

    def send(self, cmd: str) -> list[str]:
        with self._lock:
            return self._send_command(cmd)

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

    def get_status(self) -> dict:
        lines = self.send("STATUS")
        result: dict = {}
        for line in lines:
            if line.startswith("[STATUS]"):
                result.update(self._parse_status_line(line))
            elif line.startswith("[ENERGY]"):
                result["energy"] = self._parse_energy_line(line)
        return result if result else {"raw": lines}


arduino = ArduinoBridge()


# ─────────────────────────────────────────────────────────────────────────────
#  Dryer Modbus
# ─────────────────────────────────────────────────────────────────────────────

def _s16(x: int) -> int:
    return x - 65536 if x >= 32768 else x


def _u16(x: int) -> int:
    return x & 0xFFFF


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
        self.pulse_holding(addr=17, value_on=1, value_off=0, pulse_time=0.3)

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
    ok = dryer.connect()
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
    try:
        data = arduino.get_status()
        return jsonify({"ok": True, "connected": arduino.connected, "data": data})
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

@app.route("/api/estop", methods=["POST"])
def api_estop():
    """Send ESTOP to Arduino and toggle off the dryer if connected."""
    errors: list[str] = []

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
    app.run(host=SERVER_HOST, port=SERVER_PORT, debug=False, threaded=True)
