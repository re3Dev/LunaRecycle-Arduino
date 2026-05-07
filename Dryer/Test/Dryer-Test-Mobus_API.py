from flask import Flask, jsonify, request
from flask_cors import CORS
import threading
import time
from pymodbus.client import ModbusSerialClient

PORT = "COM11"
DEVICE_ID = 1
BAUDRATE = 57600
PARITY = "N"
STOPBITS = 1
BYTESIZE = 8
TIMEOUT = 0.5

app = Flask(__name__)
CORS(app)


def s16(x):
    return x - 65536 if x >= 32768 else x


def u16(x):
    return x & 0xFFFF


class DryerModbus:
    def __init__(self):
        self.client = ModbusSerialClient(
            port=PORT,
            baudrate=BAUDRATE,
            parity=PARITY,
            stopbits=STOPBITS,
            bytesize=BYTESIZE,
            timeout=TIMEOUT,
            retries=0,
        )
        self.lock = threading.Lock()
        self.connected = False

    def connect(self):
        with self.lock:
            self.connected = self.client.connect()
            return self.connected

    def ensure_connected(self):
        if not self.connected:
            self.connect()
        return self.connected

    def close(self):
        with self.lock:
            self.client.close()
            self.connected = False

    def read_input(self, addr, count=1):
        with self.lock:
            rr = self.client.read_input_registers(address=addr, count=count, device_id=DEVICE_ID)
            if rr.isError():
                raise RuntimeError(rr)
            return rr.registers

    def read_holding(self, addr, count=1):
        with self.lock:
            rr = self.client.read_holding_registers(address=addr, count=count, device_id=DEVICE_ID)
            if rr.isError():
                raise RuntimeError(rr)
            return rr.registers

    def write_holding(self, addr, value):
        with self.lock:
            wr = self.client.write_register(address=addr, value=value & 0xFFFF, device_id=DEVICE_ID)
            if wr.isError():
                raise RuntimeError(wr)

    def pulse_holding(self, addr, value_on, value_off=0, pulse_time=0.3):
        self.write_holding(addr, value_on)
        time.sleep(pulse_time)
        self.write_holding(addr, value_off)

    # Confirmed mappings
    def get_actual_dewpoint(self):
        return s16(self.read_input(12)[0])   # 30013

    def get_status_word(self):
        return self.read_input(20)[0]        # 30021

    def get_process_setpoint(self):
        return s16(self.read_input(21)[0])   # 30022

    def set_process_setpoint(self, value):
        self.write_holding(21, u16(value))   # 40022

    def get_dewpoint_setpoint(self):
        return s16(self.read_input(23)[0])   # 30024

    def set_dewpoint_setpoint(self, value):
        self.write_holding(23, u16(value))   # 40024

    def get_process_temp(self):
        return s16(self.read_input(1)[0])    # 30002

    def get_regen_temp(self):
        return s16(self.read_input(3)[0])    # 30004

    def get_regen_outlet_temp(self):
        return s16(self.read_input(4)[0])    # 30005

    def get_blower_inlet_temp(self):
        return s16(self.read_input(5)[0])    # 30006

    def get_run_state_raw(self):
        return self.read_input(6)[0]         # 30007

    def get_aux_temp(self):
        return s16(self.read_input(19)[0])   # 30020

    def get_mode_state(self):
        return self.read_input(0)[0]         # 30001

    def get_output_word(self):
        return self.read_input(25)[0]        # 30026

    def toggle_on_off(self):
        self.pulse_holding(addr=17, value_on=1, value_off=0, pulse_time=0.3)  # 40018

    def dryer_state_guess(self, run_state):
        if run_state == 0:
            return "OFF"
        if run_state == 100:
            return "ON"
        return f"Unknown ({run_state})"

    def bits_set(self, value):
        return [i for i in range(16) if value & (1 << i)]

    def snapshot(self):
        if not self.ensure_connected():
            raise RuntimeError("Failed to connect to Modbus device.")

        mode_state = self.get_mode_state()
        run_state = self.get_run_state_raw()
        process_temp = self.get_process_temp()
        regen_temp = self.get_regen_temp()
        regen_outlet_temp = self.get_regen_outlet_temp()
        blower_inlet_temp = self.get_blower_inlet_temp()
        aux_temp = self.get_aux_temp()
        actual_dp = self.get_actual_dewpoint()
        process_sp = self.get_process_setpoint()
        dew_sp = self.get_dewpoint_setpoint()
        status_word = self.get_status_word()
        output_word = self.get_output_word()

        return {
            "connected": self.connected,
            "port": PORT,
            "baudrate": BAUDRATE,
            "device_id": DEVICE_ID,
            "mode_state": mode_state,
            "run_state": run_state,
            "dryer_state_guess": self.dryer_state_guess(run_state),
            "process_temp": process_temp,
            "regen_temp": regen_temp,
            "regen_outlet_temp": regen_outlet_temp,
            "blower_inlet_temp": blower_inlet_temp,
            "aux_temp": aux_temp,
            "actual_dewpoint": actual_dp,
            "process_setpoint": process_sp,
            "dewpoint_setpoint": dew_sp,
            "status_word": status_word,
            "status_word_hex": f"0x{status_word:04X}",
            "status_bits_set": self.bits_set(status_word),
            "output_word": output_word,
            "output_word_hex": f"0x{output_word:04X}",
        }


dryer = DryerModbus()


@app.route("/api/dryer/connect", methods=["POST"])
def api_connect():
    ok = dryer.connect()
    return jsonify({
        "ok": ok,
        "connected": ok,
        "port": PORT,
        "baudrate": BAUDRATE,
        "device_id": DEVICE_ID,
    })


@app.route("/api/dryer/disconnect", methods=["POST"])
def api_disconnect():
    dryer.close()
    return jsonify({"ok": True, "connected": False})


@app.route("/api/dryer/status", methods=["GET"])
def api_status():
    try:
        data = dryer.snapshot()
        return jsonify({"ok": True, "data": data})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/api/dryer/toggle", methods=["POST"])
def api_toggle():
    try:
        before = dryer.get_run_state_raw()
        dryer.toggle_on_off()
        time.sleep(0.5)
        after = dryer.get_run_state_raw()
        return jsonify({
            "ok": True,
            "before_run_state": before,
            "after_run_state": after,
            "message": "Toggled dryer using 40018 bit 0 pulse."
        })
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/api/dryer/set_process_sp", methods=["POST"])
def api_set_process_sp():
    try:
        value = int(request.json.get("value"))
        dryer.set_process_setpoint(value)
        return jsonify({"ok": True, "value": value})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/api/dryer/set_dewpoint_sp", methods=["POST"])
def api_set_dewpoint_sp():
    try:
        value = int(request.json.get("value"))
        dryer.set_dewpoint_setpoint(value)
        return jsonify({"ok": True, "value": value})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/api/dryer/manual_read", methods=["POST"])
def api_manual_read():
    try:
        reg = int(request.json.get("register"))
        if 30001 <= reg <= 39999:
            addr = reg - 30001
            raw = dryer.read_input(addr, 1)[0]
        elif 40001 <= reg <= 49999:
            addr = reg - 40001
            raw = dryer.read_holding(addr, 1)[0]
        else:
            raise ValueError("Register must be in 30001+ or 40001+ range.")

        return jsonify({
            "ok": True,
            "register": reg,
            "raw": raw,
            "signed": s16(raw),
            "hex": f"0x{raw:04X}",
        })
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/api/dryer/manual_write", methods=["POST"])
def api_manual_write():
    try:
        reg = int(request.json.get("register"))
        value = int(request.json.get("value"))

        if not (40001 <= reg <= 49999):
            raise ValueError("Manual writes only supported for 4xxxx holding registers.")

        addr = reg - 40001
        dryer.write_holding(addr, u16(value))
        return jsonify({
            "ok": True,
            "register": reg,
            "addr": addr,
            "value": value
        })
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


if __name__ == "__main__":
    print("Starting dryer Modbus API on http://127.0.0.1:5055")
    app.run(host="127.0.0.1", port=5055, debug=True)