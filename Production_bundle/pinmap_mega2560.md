# LunaRecycle FPU — Arduino Mega 2560 Pin Map

## Reserved / Fixed-Function
| Pin  | Signal       | Notes                  |
|------|--------------|------------------------|
| D0   | Serial0 RX   | USB / RPi comms        |
| D1   | Serial0 TX   | USB / RPi comms        |
| D20  | I2C SDA      | Shared I2C bus         |
| D21  | I2C SCL      | Shared I2C bus         |

---

## Hardware Interrupt Inputs
| Pin        | Signal                      | Notes                               |
|------------|-----------------------------|-------------------------------------|
| D2 (INT0)  | `Mixer_screwRotationSensor` | Hall-effect; interrupt for accurate RPM |
| D3 (INT1)  | `FPU_emergencyStop`         | E-stop — needs fast response        |

---

## PWM / Servo Outputs
| Pin  | Signal                               | Notes                              |
|------|--------------------------------------|------------------------------------|
| D4   | `TC_stepperMotorController_step`     | Stepper STEP pulse                 |
| D5   | `TC_servoMotor`                      | RC servo (vacuum picker lift)      |
| D6   | `Mixer_shredderGateLeftServoMotor`   | RC servo                           |
| D7   | `Mixer_shredderGateRightServoMotor`  | RC servo                           |
| D8   | `Mixer_blastGateLeft` → RoboClaw ch1 | RC pulse 1000–2000 µs              |
| D9   | `Mixer_blastGateRight` → RoboClaw ch2| RC pulse 1000–2000 µs              |
| D10  | `Mixer_motorController_ENA`          | PWM — screw motor speed            |
| D11  | `Mixer_motorController_ENB`          | PWM — agitator motor speed         |

> D12, D13, D44, D45, D46 remain as spare PWM pins. D13 has the built-in LED; keep spare to avoid signal noise.

---

## General Digital Outputs
| Pin  | Signal                                  | Notes                              |
|------|-----------------------------------------|------------------------------------|
| D22  | `TC_stepperMotorController_dir`         | Stepper DIR                        |
| D23  | `TC_stepperMotorController_enable`      | Stepper ENABLE (active-low)        |
| D24  | `TC_vacuumPumpRelay`                    | Relay ON/OFF                       |
| D25  | `Shredder_motorController_onOff`        | Shredder ON/OFF                    |
| D26  | `Shredder_motorController_direction`    | Shredder FWD/REV                   |
| D27  | `Mixer_motorController_IN1`             | Screw motor dir A                  |
| D28  | `Mixer_motorController_IN2`             | Screw motor dir B                  |
| D29  | `Mixer_motorController_IN3`             | Agitator motor dir A               |
| D30  | `Mixer_motorController_IN4`             | Agitator motor dir B               |
| D31  | `Dryer_SSR`                             | SSR — ConAir dryer AC power        |
| D32  | `FE_SSR`                                | SSR — fume extractor AC power      |
| D33  | `FC_BLDCmotorController_forward`        | BLDC enable/forward signal         |

---

## General Digital Inputs
| Pin  | Signal                          | Notes            |
|------|---------------------------------|------------------|
| D34  | `TC_stepperHomeLimit`           | Homing limit switch |
| D35  | `TC_leftFilmSensor`             | IR sensor        |
| D36  | `TC_rightFilmSensor`            | IR sensor        |
| D37  | `Mixer_blastGateLeftMinLimit`   | Limit switch     |
| D38  | `Mixer_blastGateLeftMaxLimit`   | Limit switch     |
| D39  | `Mixer_blastGateRightMinLimit`  | Limit switch     |
| D40  | `Mixer_blastGateRightMaxLimit`  | Limit switch     |

---

## Analog Inputs
| Pin  | Signal                              | Notes                  |
|------|-------------------------------------|------------------------|
| A0   | `TC_vacuumSensor`                   | 0–5V vacuum pressure   |
| A1   | `FPU_24VsupplyACCurrent`            | AC current sensor      |
| A2   | `FC_48VsupplyACCurrent`             | AC current sensor      |
| A3   | `Shredder_motorControllerACCurrent` | AC current sensor      |
| A4   | `GBX_ACCurrent`                     | AC current sensor      |
| A5   | `Dryer_ACCurrent`                   | AC current sensor      |
| A6   | `FE_ACCurrent`                      | AC current sensor      |

---

## I2C Bus (D20 SDA / D21 SCL) — Shared
| Device                                        | Address | Notes                              |
|-----------------------------------------------|---------|------------------------------------|
| `Mixer_screwMotorCurrentSensor` (INA219)      | 0x40    |                                    |
| `FPU_environmentalSensor` (SHT45)             | 0x44    |                                    |
| `GBX_environmentalSensor` (SHT45)             | 0x45    | ⚠️ Verify address variant          |
| `Dryer_regenExhaustEnvironmentalSensor` (SHT45)| 0x46   | ⚠️ Verify address variant          |
| `FC_BLDCmotorController` digipot (MCP4725)    | 0x60    | Throttle 0–5V control              |

---

## Design Notes & Caveats

1. **SHT45 I2C addresses** — The SHT45 has a fixed address (0x44). We will need Adafruit alternate-address breakout variants or a **TCA9548A I2C multiplexer** to run all three environmental sensors on the same bus simultaneously.

2. **Test sketch pin remapping** — Existing test sketches were written for Uno pin numbers and will need their `const int` assignments updated to match this Mega layout before integration into production firmware.

3. **Spare resources** — Approximately 30 digital pins (including D12, D13, D41–D43, D44–D53) and 9 analog pins (A7–A15) remain free for future use, such as feedstock detection sensors, the status stack light signal, or vacuum sensors along the feedstock conveyance tubing.
