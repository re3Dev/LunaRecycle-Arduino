# LunaRecycle — Raspberry Pi Deployment

Runs the LunaRecycle backend (Flask API + dashboard/viewer/model UI) on a
Raspberry Pi wired to the Arduino Mega and the Conair dryer.

## Hardware wiring

| Device        | Connection                        | Default port    |
| ------------- | --------------------------------- | --------------- |
| Arduino Mega  | USB (native CDC)                  | `/dev/ttyACM0`  |
| Conair dryer  | USB-to-RS485 adapter (Modbus RTU) | `/dev/ttyUSB0`  |

Confirm the enumerated ports after plugging both in:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
# or watch enumeration live:
dmesg -w
```

If they differ from the defaults, edit `lunarecycle.env` (see below).

## Install

1. Copy this `Production_bundle/` folder onto the Pi (e.g. via `git clone` or `scp`).
2. From inside the folder:

   ```bash
   chmod +x setup_pi.sh
   sudo ./setup_pi.sh
   ```

The script installs Python/venv, adds your user to the `dialout` group for
serial access, copies files to `/opt/lunarecycle`, creates a virtualenv,
installs dependencies, and enables a systemd service that starts on boot.

## Access

- On the Pi: <http://127.0.0.1:5055/>
- From another LAN device: `http://<pi-ip>:5055/`

Routes:

| Path       | Page                     |
| ---------- | ------------------------ |
| `/`        | Control dashboard        |
| `/viewer`  | Read-only status viewer  |
| `/model`   | Spatial model            |

## Configuration

All settings live in `/opt/lunarecycle/lunarecycle.env` and are read as
environment variables. Edit and restart the service:

```bash
sudo nano /opt/lunarecycle/lunarecycle.env
sudo systemctl restart lunarecycle-backend
```

| Variable             | Default          | Purpose                              |
| -------------------- | ---------------- | ------------------------------------ |
| `LUNA_ARDUINO_PORT`  | `/dev/ttyACM0`   | Arduino serial port                  |
| `LUNA_ARDUINO_BAUD`  | `115200`         | Arduino baud rate                    |
| `LUNA_DRYER_PORT`    | `/dev/ttyUSB0`   | Dryer RS485 adapter port             |
| `LUNA_DRYER_BAUD`    | `57600`          | Dryer Modbus baud rate               |
| `LUNA_DRYER_ID`      | `1`              | Dryer Modbus device ID               |
| `LUNA_MIX_RAMP_STEPS`| `5`              | Mixer soft-start step count          |
| `LUNA_MIX_RAMP_STEP_SEC` | `0.20`       | Delay between mixer ramp steps (s)   |
| `LUNA_HOST`          | `0.0.0.0`        | Bind address (`127.0.0.1` = Pi only) |
| `LUNA_PORT`          | `5055`           | HTTP port                            |

### Stable port names (recommended)

USB enumeration order can change between boots. To pin each device, create udev
rules keyed on the adapter serial numbers and point the env vars at the symlinks:

```bash
# Find identifiers:
udevadm info -a -n /dev/ttyUSB0 | grep -E 'serial|idVendor|idProduct'

# /etc/udev/rules.d/99-lunarecycle.rules  (example)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{serial}=="XXManufSerial", SYMLINK+="lunarecycle-dryer"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", SYMLINK+="lunarecycle-arduino"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then set `LUNA_ARDUINO_PORT=/dev/lunarecycle-arduino` and
`LUNA_DRYER_PORT=/dev/lunarecycle-dryer`.

## Service management

```bash
sudo systemctl status  lunarecycle-backend   # health
sudo systemctl restart lunarecycle-backend   # apply config changes
sudo systemctl stop    lunarecycle-backend
journalctl -u lunarecycle-backend -f         # live logs
```

## Manual run (debugging)

```bash
cd /opt/lunarecycle
set -a; . ./lunarecycle.env; set +a
./venv/bin/python lunarecycle_backend.py
```

## Troubleshooting

- **Permission denied on `/dev/ttyACM0` / `/dev/ttyUSB0`** — the user isn't in
  `dialout` yet; log out/in or reboot after running `setup_pi.sh`.
- **Port not found** — check `ls /dev/ttyACM* /dev/ttyUSB*` and update the env file.
- **Dashboard loads but data is blank** — the API is reachable but serial isn't;
  check `journalctl -u lunarecycle-backend -f` for connection errors.
