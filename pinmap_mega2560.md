# LunaRecycle FPU — Arduino Mega 2560 Pin Map

---

## Arduino Mega (FPU_Mega)
| Pin  | Signal        | Notes              |
|------|---------------|--------------------|
| D0   | Serial0 RX    | USB / RPi comms    |
| D1   | Serial0 TX    | USB / RPi comms    |
| D20  | I2C SDA       | Shared I2C bus     |
| D21  | I2C SCL       | Shared I2C bus     |

---

## Trash Conveyor

### `TC_stepperMotorController` — Stepper Motor Controller
| Pin  | Signal                              | Notes               |
|------|-------------------------------------|---------------------|
| D4   | `TC_stepperMotorController_step`    | STEP pulse          |
| D22  | `TC_stepperMotorController_dir`     | DIR                 |
| D23  | `TC_stepperMotorController_enable`  | ENABLE (active-low) |

### `TC_servoMotor` — Vacuum Picker Lift Servo
| Pin  | Signal          | Notes         |
|------|-----------------|---------------|
| D5   | `TC_servoMotor` | RC servo PWM  |

### `TC_vacuumPumpRelay` — Vacuum Pump Relay
| Pin  | Signal              | Notes      |
|------|---------------------|------------|
| D24  | `TC_vacuumPumpRelay`| ON/OFF     |

### `TC_vacuumSensor` — Vacuum Pressure Sensor
| Pin  | Signal            | Notes            |
|------|-------------------|------------------|
| A0   | `TC_vacuumSensor` | 0–5V analog      |

### `TC_stepperHomeLimit` — Homing Limit Switch
| Pin  | Signal               | Notes         |
|------|----------------------|---------------|
| D34  | `TC_stepperHomeLimit`| Digital input |

### `TC_leftFilmSensor` / `TC_rightFilmSensor` — IR Film Sensors
| Pin  | Signal               | Notes         |
|------|----------------------|---------------|
| D35  | `TC_leftFilmSensor`  | Digital input |
| D36  | `TC_rightFilmSensor` | Digital input |

---

## Shredder

### `Shredder_motorController` — Motor Controller (ON/OFF + Direction)
| Pin  | Signal                            | Notes           |
|------|-----------------------------------|-----------------|
| D25  | `Shredder_motorController_onOff`  | ON/OFF          |
| D26  | `Shredder_motorController_direction` | FWD/REV      |

### `Shredder_motorControllerACCurrent` — AC Current Sensor
| Pin  | Signal                              | Notes       |
|------|-------------------------------------|-------------|
| A3   | `Shredder_motorControllerACCurrent` | Analog      |

---

## Mixer

### `Mixer_shredderGateLeftServoMotor` / `Mixer_shredderGateRightServoMotor` — Shredder Gate Servos
| Pin  | Signal                                 | Notes        |
|------|----------------------------------------|--------------|
| D6   | `Mixer_shredderGateLeftServoMotor`     | RC servo PWM |
| D7   | `Mixer_shredderGateRightServoMotor`    | RC servo PWM |

### `Mixer_motorController` — Screw Motor H-Bridge
| Pin  | Signal                      | Notes              |
|------|-----------------------------|--------------------|
| D10  | `Mixer_motorController_ENA` | PWM speed          |
| D27  | `Mixer_motorController_IN1` | Direction A        |
| D28  | `Mixer_motorController_IN2` | Direction B        |

### `Mixer_agitatorMotor` — Bottom Agitator (2nd H-Bridge channel)
| Pin  | Signal                       | Notes                                   |
|------|------------------------------|-----------------------------------------|
| D11   | `Mixer_agitatorMotor_ENB`    | PWM enable — power **capped at 50%**    |
| D42  | `Mixer_agitatorMotor_IN3`    | Direction A                             |
| D43  | `Mixer_agitatorMotor_IN4`    | Direction B (shares Mega built-in LED)  |

### `Mixer_vacuumMotor` — Vacuum Motor (DS3502 digital-pot speed control)
| Bus  | Signal                       | Notes                                   |
|------|------------------------------|-----------------------------------------|
| I2C  | `Mixer_vacuumMotorDigipot` (DS3502) | Sets the vacuum motor speed, wiper 0–127 (0–100%); default addr 0x28 |

> **Note:** D3 was previously listed as a reserved `FPU_emergencyStop` interrupt, but that
> interrupt is not implemented in firmware. D3 is now used for the agitator ENB. If an
> external e-stop interrupt is added later, relocate it to another INT pin.

### `Mixer_screwMotorCurrentSensor` — INA219 Current Sensor (I2C)
| Bus  | Address | Notes |
|------|---------|-------|
| I2C  | 0x40    | SDA D20 / SCL D21 |

### `Mixer_screwRotationSensor` — Hall-Effect Speed Sensor
| Pin        | Signal                      | Notes                           |
|------------|-----------------------------|---------------------------------|
| D2 (INT0)  | `Mixer_screwRotationSensor` | Interrupt — accurate RPM count  |

### `Mixer_linearMotorController` — RoboClaw Blast Gate Controller
| Pin  | Signal                               | Notes                 |
|------|--------------------------------------|-----------------------|
| D8   | `Mixer_blastGateLeft` → RoboClaw ch1 | RC pulse 1000–2000 µs |
| D9   | `Mixer_blastGateRight` → RoboClaw ch2| RC pulse 1000–2000 µs |

### Blast Gate Limit Switches
| Pin  | Signal                          | Notes         |
|------|---------------------------------|---------------|
| D37  | `Mixer_blastGateLeftMinLimit`   | Digital input |
| D38  | `Mixer_blastGateLeftMaxLimit`   | Digital input |
| D39  | `Mixer_blastGateRightMinLimit`  | Digital input |
| D40  | `Mixer_blastGateRightMaxLimit`  | Digital input |

---

## ConAir Dryer
### Dryer Control — Modbus RTU over USB
| Host | Interface | Notes                                          |
|------|-----------|------------------------------------------------|
| RPi 5| USB       | Modbus RTU — RPi talks to dryer directly       |

> The dryer's control interface is handled entirely by the RPi 5 via USB Modbus. The Mega has no direct connection to the dryer control registers.
### `Dryer_SSR` — SSR AC Power Control
| Pin  | Signal      | Notes                    |
|------|-------------|--------------------------|
| D31  | `Dryer_SSR` | SSR — dryer AC power     |

### `Dryer_regenExhaustEnvironmentalSensor` — Temp/Humidity (I2C)
| Bus  | Address | Notes                        |
|------|---------|------------------------------|
| I2C  | 0x46    | ⚠️ Verify address variant    |

### `Dryer_ACCurrent` — AC Current Sensor
| Pin  | Signal           | Notes  |
|------|------------------|--------|
| A5   | `Dryer_ACCurrent`| Analog |

---

## Feedstock Conveyor

### `FC_BLDCmotorController` — BLDC Motor Controller
| Pin  | Signal                             | Notes                          |
|------|------------------------------------|--------------------------------|
| D33  | `FC_BLDCmotorController_forward`   | Digital enable/forward signal  |
| I2C  | 0x60 (MCP4725 digipot)             | Throttle 0–5V control          |

### `FC_48VsupplyACCurrent` — AC Current Sensor
| Pin  | Signal                  | Notes  |
|------|-------------------------|--------|
| A2   | `FC_48VsupplyACCurrent` | Analog |

---

## Fume Extractor

### `FE_SSR` — SSR AC Power Control
| Pin  | Signal   | Notes                         |
|------|----------|-------------------------------|
| D32  | `FE_SSR` | SSR — fume extractor AC power |

### `FE_ACCurrent` — AC Current Sensor
| Pin  | Signal         | Notes  |
|------|----------------|--------|
| A6   | `FE_ACCurrent` | Analog |

---

## Environmental Monitoring

### Temp/Humidity Sensors (I2C, SHT45)
| Bus  | Address | Component                                  |
|------|---------|--------------------------------------------|
| I2C  | 0x44    | `FPU_environmentalSensor`                  |
| I2C  | 0x45    | `GBX_environmentalSensor` ⚠️ Verify addr  |
| I2C  | 0x46    | `Dryer_regenExhaustEnvironmentalSensor` ⚠️ |

---

## Power Monitoring

### AC Current Sensors (Analog)
| Pin  | Signal                      | Notes  |
|------|-----------------------------|--------|
| A1   | `FPU_24VsupplyACCurrent`    | Analog |
| A2   | `FC_48VsupplyACCurrent`     | Analog |
| A3   | `Shredder_motorControllerACCurrent` | Analog |
| A4   | `GBX_ACCurrent`             | Analog |
| A5   | `Dryer_ACCurrent`           | Analog |
| A6   | `FE_ACCurrent`              | Analog |

---

## RPi 5 USB Peripherals

### Webcams
| Host  | Interface | Component       | Notes                                      |
|-------|-----------|-----------------|--------------------------------------------|
| RPi 5 | USB       | `TC_webcam`     | Nocturne 1080p — trash conveyor view       |
| RPi 5 | USB       | `Mixer_webcam`  | Nocturne 1080p — mixing chamber view       |

### ConAir Dryer Modbus
| Host  | Interface | Notes                                              |
|-------|-----------|----------------------------------------------------|
| RPi 5 | USB       | Modbus RTU adapter — dryer parameter control       |

---

## Safety Systems

### `FPU_emergencyStop` — Emergency Stop
| Pin        | Signal              | Notes                      |
|------------|---------------------|----------------------------|
| D3 (INT1)  | `FPU_emergencyStop` | Interrupt — fast response  |

---

## Design Notes & Caveats

1. **SHT45 I2C addresses** — The SHT45 has a fixed address (0x44). We will need Adafruit alternate-address breakout variants or a **TCA9548A I2C multiplexer** to run all three environmental sensors on the same bus simultaneously.

2. **RoboClaw blast gate control** — Currently mapped in RC pulse mode (same as existing test sketches). If switching to RoboClaw packet serial mode, D8/D9 can be freed and **Serial1 (D18 TX1 / D19 RX1)** used instead for more reliable communication.

3. **Test sketch pin remapping** — Existing test sketches were written for Uno pin numbers and will need their `const int` assignments updated to match this Mega layout before integration into production firmware.

4. **Spare resources** — Approximately 30 digital pins (including D12, D13, D41–D43, D44–D53) and 9 analog pins (A7–A15) remain free for future use, such as feedstock detection sensors, the status stack light signal, or vacuum sensors along the feedstock conveyance tubing. D13 has the built-in LED; keep spare to avoid signal noise.
