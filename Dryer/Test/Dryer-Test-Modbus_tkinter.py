import tkinter as tk
from tkinter import ttk, messagebox
import time
from pymodbus.client import ModbusSerialClient

PORT = "COM11"
DEVICE_ID = 1
BAUDRATE = 57600
PARITY = "N"
STOPBITS = 1
BYTESIZE = 8
TIMEOUT = 0.5


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

    def connect(self):
        return self.client.connect()

    def close(self):
        self.client.close()

    def read_input(self, addr, count=1):
        rr = self.client.read_input_registers(address=addr, count=count, device_id=DEVICE_ID)
        if rr.isError():
            raise RuntimeError(rr)
        return rr.registers

    def read_holding(self, addr, count=1):
        rr = self.client.read_holding_registers(address=addr, count=count, device_id=DEVICE_ID)
        if rr.isError():
            raise RuntimeError(rr)
        return rr.registers

    def write_holding(self, addr, value):
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

    # Confirmed live values from your testing
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

    def get_total_msw_power(self):
        return s16(self.read_input(18)[0])   # 30019

    def get_aux_temp(self):
        return s16(self.read_input(19)[0])   # 30020

    def get_mode_state(self):
        return self.read_input(0)[0]         # 30001

    def get_output_word(self):
        return self.read_input(25)[0]        # 30026

    # Confirmed On/Off command: 40018 bit 0 pulse
    def toggle_on_off(self):
        self.pulse_holding(addr=17, value_on=1, value_off=0, pulse_time=0.3)  # 40018


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Conair Dryer Modbus Control")
        self.geometry("940x760")
        self.resizable(True, True)

        self.mb = DryerModbus()
        self.connected = False
        self.polling = False
        self.poll_var = tk.BooleanVar(value=False)

        self.status_var = tk.StringVar(value="Disconnected")
        self.process_temp_var = tk.StringVar(value="---")
        self.regen_temp_var = tk.StringVar(value="---")
        self.regen_outlet_temp_var = tk.StringVar(value="---")
        self.blower_inlet_temp_var = tk.StringVar(value="---")
        self.aux_temp_var = tk.StringVar(value="---")
        self.actual_dewpoint_var = tk.StringVar(value="---")
        self.process_sp_var = tk.StringVar(value="---")
        self.dewpoint_sp_var = tk.StringVar(value="---")
        self.status_word_var = tk.StringVar(value="---")
        self.status_bits_var = tk.StringVar(value="---")
        self.run_state_var = tk.StringVar(value="---")
        self.mode_state_var = tk.StringVar(value="---")
        self.output_word_var = tk.StringVar(value="---")
        self.total_msw_power_var = tk.StringVar(value="---")
        self.onoff_guess_var = tk.StringVar(value="Unknown")

        self.proc_sp_entry_var = tk.StringVar()
        self.dp_sp_entry_var = tk.StringVar()

        self.manual_reg_var = tk.StringVar(value="40022")
        self.manual_val_var = tk.StringVar(value="72")
        self.manual_read_reg_var = tk.StringVar(value="30022")

        self._build_ui()

    def _build_ui(self):
        top = ttk.Frame(self, padding=12)
        top.pack(fill="x")

        ttk.Button(top, text="Connect", command=self.connect_modbus).pack(side="left")
        ttk.Button(top, text="Disconnect", command=self.disconnect_modbus).pack(side="left", padx=(8, 0))
        ttk.Button(top, text="Refresh Now", command=self.refresh_once).pack(side="left", padx=(8, 0))

        ttk.Checkbutton(
            top,
            text="Auto Poll",
            variable=self.poll_var,
            command=self.toggle_poll
        ).pack(side="left", padx=(12, 0))

        ttk.Label(top, textvariable=self.status_var).pack(side="right")

        main = ttk.Frame(self, padding=12)
        main.pack(fill="both", expand=True)

        live = ttk.LabelFrame(main, text="Live Readings", padding=12)
        live.pack(fill="x")

        self._row(live, 0, "Mode State (30001)", self.mode_state_var)
        self._row(live, 1, "Run State (30007)", self.run_state_var)
        self._row(live, 2, "Dryer State Guess", self.onoff_guess_var)
        self._row(live, 3, "Process Temp", self.process_temp_var)
        self._row(live, 4, "Regen Temp", self.regen_temp_var)
        self._row(live, 5, "Regen Outlet Temp", self.regen_outlet_temp_var)
        self._row(live, 6, "Blower Inlet Temp", self.blower_inlet_temp_var)
        self._row(live, 7, "Aux Temp", self.aux_temp_var)
        self._row(live, 8, "Actual Dew Point", self.actual_dewpoint_var)
        self._row(live, 9, "Process Setpoint", self.process_sp_var)
        self._row(live, 10, "Dew Point Setpoint", self.dewpoint_sp_var)
        self._row(live, 11, "Status Word (30021)", self.status_word_var)
        self._row(live, 12, "Status Bits Set", self.status_bits_var)
        self._row(live, 13, "Total MSW Power (30019)", self.total_msw_power_var)
        self._row(live, 14, "Output Word (30026)", self.output_word_var)

        controls = ttk.LabelFrame(main, text="Dryer Controls", padding=12)
        controls.pack(fill="x", pady=(12, 0))

        ttk.Button(
            controls,
            text="Toggle Dryer On / Off",
            command=self.toggle_dryer
        ).grid(row=0, column=0, padx=(0, 12), pady=4, sticky="w")

        ttk.Label(
            controls,
            text="Uses 40018 bit 0 pulse"
        ).grid(row=0, column=1, sticky="w")

        setpoints = ttk.LabelFrame(main, text="Setpoints", padding=12)
        setpoints.pack(fill="x", pady=(12, 0))

        ttk.Label(setpoints, text="Process Temp SP").grid(row=0, column=0, sticky="w")
        ttk.Entry(setpoints, textvariable=self.proc_sp_entry_var, width=12).grid(row=0, column=1, padx=8)
        ttk.Button(setpoints, text="Write 40022", command=self.write_process_sp).grid(row=0, column=2, padx=8)

        ttk.Label(setpoints, text="Dew Point SP").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(setpoints, textvariable=self.dp_sp_entry_var, width=12).grid(row=1, column=1, padx=8, pady=(8, 0))
        ttk.Button(setpoints, text="Write 40024", command=self.write_dewpoint_sp).grid(row=1, column=2, padx=8, pady=(8, 0))

        manual = ttk.LabelFrame(main, text="Manual Register Tools", padding=12)
        manual.pack(fill="both", expand=True, pady=(12, 0))

        read_frame = ttk.Frame(manual)
        read_frame.pack(fill="x")

        ttk.Label(read_frame, text="Read Register").grid(row=0, column=0, sticky="w")
        ttk.Entry(read_frame, textvariable=self.manual_read_reg_var, width=12).grid(row=0, column=1, padx=8)
        ttk.Button(read_frame, text="Read Once", command=self.manual_read).grid(row=0, column=2, padx=8)

        write_frame = ttk.Frame(manual)
        write_frame.pack(fill="x", pady=(12, 0))

        ttk.Label(write_frame, text="Write Register").grid(row=0, column=0, sticky="w")
        ttk.Entry(write_frame, textvariable=self.manual_reg_var, width=12).grid(row=0, column=1, padx=8)

        ttk.Label(write_frame, text="Value").grid(row=0, column=2, sticky="w")
        ttk.Entry(write_frame, textvariable=self.manual_val_var, width=12).grid(row=0, column=3, padx=8)

        ttk.Button(write_frame, text="Write Once", command=self.manual_write).grid(row=0, column=4, padx=8)

        self.log = tk.Text(manual, height=14, wrap="word")
        self.log.pack(fill="both", expand=True, pady=(12, 0))

    def _row(self, parent, row, label, var):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=(0, 12), pady=2)
        ttk.Label(parent, textvariable=var).grid(row=row, column=1, sticky="w", pady=2)

    def log_line(self, text):
        self.log.insert("end", text + "\n")
        self.log.see("end")

    def bits_set(self, value):
        return [i for i in range(16) if value & (1 << i)]

    def dryer_state_guess(self, run_state):
        if run_state == 0:
            return "OFF"
        if run_state == 100:
            return "ON"
        return f"Unknown ({run_state})"

    def connect_modbus(self):
        try:
            if self.mb.connect():
                self.connected = True
                self.status_var.set(f"Connected on {PORT} @ {BAUDRATE} N81, ID {DEVICE_ID}")
                self.log_line("Connected.")
                self.refresh_once()
            else:
                self.status_var.set("Failed to connect")
                messagebox.showerror("Connection Error", "Failed to connect to Modbus device.")
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def disconnect_modbus(self):
        try:
            self.polling = False
            self.poll_var.set(False)
            self.mb.close()
            self.connected = False
            self.status_var.set("Disconnected")
            self.log_line("Disconnected.")
        except Exception as e:
            messagebox.showerror("Disconnect Error", str(e))

    def refresh_once(self):
        if not self.connected:
            return
        try:
            mode_state = self.mb.get_mode_state()
            run_state = self.mb.get_run_state_raw()
            process_temp = self.mb.get_process_temp()
            regen_temp = self.mb.get_regen_temp()
            regen_outlet_temp = self.mb.get_regen_outlet_temp()
            blower_inlet_temp = self.mb.get_blower_inlet_temp()
            aux_temp = self.mb.get_aux_temp()
            actual_dp = self.mb.get_actual_dewpoint()
            process_sp = self.mb.get_process_setpoint()
            dew_sp = self.mb.get_dewpoint_setpoint()
            status_word = self.mb.get_status_word()
            total_msw_power = self.mb.get_total_msw_power()
            output_word = self.mb.get_output_word()

            self.mode_state_var.set(f"{mode_state} (0x{mode_state:04X})")
            self.run_state_var.set(f"{run_state} (0x{run_state:04X})")
            self.onoff_guess_var.set(self.dryer_state_guess(run_state))
            self.process_temp_var.set(str(process_temp))
            self.regen_temp_var.set(str(regen_temp))
            self.regen_outlet_temp_var.set(str(regen_outlet_temp))
            self.blower_inlet_temp_var.set(str(blower_inlet_temp))
            self.aux_temp_var.set(str(aux_temp))
            self.actual_dewpoint_var.set(str(actual_dp))
            self.process_sp_var.set(str(process_sp))
            self.dewpoint_sp_var.set(str(dew_sp))
            self.status_word_var.set(f"{status_word} (0x{status_word:04X})")
            self.status_bits_var.set(str(self.bits_set(status_word)))
            self.total_msw_power_var.set(f"{total_msw_power} W")
            self.output_word_var.set(f"{output_word} (0x{output_word:04X})")

        except Exception as e:
            self.status_var.set(f"Read error: {e}")
            self.log_line(f"Read error: {e}")

    def poll_loop(self):
        if self.polling and self.connected:
            self.refresh_once()
            self.after(1000, self.poll_loop)

    def toggle_poll(self):
        self.polling = self.poll_var.get()
        if self.polling:
            self.log_line("Auto poll enabled.")
            self.poll_loop()
        else:
            self.log_line("Auto poll disabled.")

    def write_process_sp(self):
        if not self.connected:
            return
        try:
            value = int(self.proc_sp_entry_var.get().strip())
            self.mb.set_process_setpoint(value)
            self.log_line(f"Wrote process setpoint {value} to 40022.")
            self.refresh_once()
        except Exception as e:
            messagebox.showerror("Write Error", str(e))

    def write_dewpoint_sp(self):
        if not self.connected:
            return
        try:
            value = int(self.dp_sp_entry_var.get().strip())
            self.mb.set_dewpoint_setpoint(value)
            self.log_line(f"Wrote dew point setpoint {value} to 40024.")
            self.refresh_once()
        except Exception as e:
            messagebox.showerror("Write Error", str(e))

    def toggle_dryer(self):
        if not self.connected:
            return
        try:
            before = self.mb.get_run_state_raw()
            self.log_line(f"Before toggle: run state 30007 = {before}")
            self.mb.toggle_on_off()
            time.sleep(0.5)
            self.refresh_once()
            after = self.mb.get_run_state_raw()
            self.log_line(f"Toggled dryer using 40018 bit 0 pulse. After toggle: run state 30007 = {after}")
        except Exception as e:
            messagebox.showerror("Toggle Error", str(e))

    def manual_read(self):
        if not self.connected:
            return
        try:
            reg = int(self.manual_read_reg_var.get().strip())
            if 30001 <= reg <= 39999:
                addr = reg - 30001
                raw = self.mb.read_input(addr, 1)[0]
                self.log_line(f"READ {reg}: raw={raw} signed={s16(raw)} hex=0x{raw:04X}")
            elif 40001 <= reg <= 49999:
                addr = reg - 40001
                raw = self.mb.read_holding(addr, 1)[0]
                self.log_line(f"READ {reg}: raw={raw} signed={s16(raw)} hex=0x{raw:04X}")
            else:
                raise ValueError("Register must be in 30001+ or 40001+ range.")
        except Exception as e:
            messagebox.showerror("Manual Read Error", str(e))

    def manual_write(self):
        if not self.connected:
            return
        try:
            reg = int(self.manual_reg_var.get().strip())
            value = int(self.manual_val_var.get().strip())

            if not (40001 <= reg <= 49999):
                raise ValueError("Manual writes only supported for 4xxxx holding registers.")

            addr = reg - 40001
            self.mb.write_holding(addr, u16(value))
            self.log_line(f"WROTE {value} to {reg} (addr {addr}).")
            self.refresh_once()
        except Exception as e:
            messagebox.showerror("Manual Write Error", str(e))


if __name__ == "__main__":
    app = App()
    app.mainloop()