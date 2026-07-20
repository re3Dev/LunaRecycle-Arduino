/*
 * Stepper Subsystem Test Utility - Target Board: Arduino Mega 2560
 * Configured for Leadshine EM882S driver @ 400 pulses/rev (2 microsteps)
 * Feature: Gearbox Output-Centric Variables & Dynamic Calculations
 */

#include <AccelStepper.h>

// ============================================================================
// Pin Assignments (Change variables as needed)
// ============================================================================
int pin_PUL  = 19;  // To Driver PUL+ (PUL- to GND)
int pin_DIR  = 18;  // To Driver DIR+ (DIR- to GND)
int pin_HALL = 3;   // To Hall Signal Out (hardware interrupt input)

// ============================================================================
// Gearbox & Driver Physical Parameters
// ============================================================================
const float GEAR_RATIO = 18.0;             // 18:1 Planetary Reducer
const float DRIVER_PULSES_PER_REV = 400.0; // Driver default switch setting (2 microsteps)

// ============================================================================
// Intuitive Output Configuration (Change these to tune the physical output)
// ============================================================================
const float OUTPUT_TARGET_RPM = 17;      // Desired speed at the visible output shaft (15 RPM)
const float OUTPUT_OFFSET_DEGREES = 5.75;  // Desired over-travel degrees at the output shaft (Yields 115 steps)

// ============================================================================
// Automatic Speed & Step Translations (Calculated by the CPU at compile time)
// ============================================================================
// Translate output shaft RPM to motor shaft RPM: 15 RPM * 18 = 270 RPM
const float MOTOR_TARGET_RPM = OUTPUT_TARGET_RPM * GEAR_RATIO; 

// Calculate step pulse frequency (Hz): (270 RPM * 400 pulses) / 60 seconds = 1800 Hz
const float STEPPER_SPEED_HZ = (MOTOR_TARGET_RPM * DRIVER_PULSES_PER_REV) / 60.0; 

// Translate output degrees to raw motor step counts: (5.75° / 360°) * 18 * 400 = 115 steps
const long OUTPUT_OFFSET_STEPS = (long)((OUTPUT_OFFSET_DEGREES / 360.0) * GEAR_RATIO * DRIVER_PULSES_PER_REV);

// Sensor Active Edge Config
const int HALL_TRIGGER_EDGE = FALLING; 

// ============================================================================
// Subsystem State Machine & Objects
// ============================================================================
enum MechanismState {
  STATE_IDLE,
  STATE_SEEKING_HALL,
  STATE_DRIVING_PAST_HOME,
  STATE_ARRIVED
};

MechanismState currentSystemState = STATE_IDLE;
volatile bool hallEdgeDetected = false;

// Initialize AccelStepper in DRIVER mode
AccelStepper SubsystemStepper(AccelStepper::DRIVER, pin_PUL, pin_DIR);

// ============================================================================
// Hardware Interrupt Service Routine (ISR)
// ============================================================================
void hallSensorISR() {
  hallEdgeDetected = true; 
}

// ============================================================================
// Subsystem Core Functions 
// ============================================================================

// Call this function to initiate the cycle
void startRotationToHallMark() {
  Serial.println(F("[SUBSYSTEM] Initiating seek cycle..."));
  
  hallEdgeDetected = false;
  currentSystemState = STATE_SEEKING_HALL;
  
  SubsystemStepper.enableOutputs();
  SubsystemStepper.setMaxSpeed(STEPPER_SPEED_HZ);
  SubsystemStepper.setSpeed(STEPPER_SPEED_HZ); 
}

void updateSubsystem() {
  switch (currentSystemState) {
    
    case STATE_SEEKING_HALL:
      // Run continuously at our constant target speed profile
      SubsystemStepper.runSpeed();
      
      // Look for the instant the magnet triggers the falling edge interrupt
      if (hallEdgeDetected) {
        // 1. Instantly declare this physical point as step 0
        SubsystemStepper.setCurrentPosition(0); 
        
        // 2. Assign the target absolute position to our translated offset steps
        SubsystemStepper.moveTo(OUTPUT_OFFSET_STEPS);
        
        // 3. Shift the state machine to track the precision offset leg
        currentSystemState = STATE_DRIVING_PAST_HOME;
        Serial.println(F("[SUBSYSTEM] Sensor tripped! Zero calibrated."));
      }
      break;

    case STATE_DRIVING_PAST_HOME:
      // run() handles absolute position tracking and deceleration limits
      SubsystemStepper.run();
      
      // Stop the motor completely once it reaches the target offset position
      if (SubsystemStepper.distanceToGo() == 0) {
        SubsystemStepper.disableOutputs();
        currentSystemState = STATE_ARRIVED;
        Serial.println(F("[SUBSYSTEM] Offset reached!"));
      }
      break;

    case STATE_ARRIVED:
      // Parked state; awaits a fresh 'G' command via serial
      break;
      
    case STATE_IDLE:
    default:
      break;
  }
}

// ============================================================================
// Setup and Main Loop
// ============================================================================
void setup() {
  Serial.begin(115200);
  
  // Configure the Hall sensor pin with the internal pull-up resistor
  pinMode(pin_HALL, INPUT_PULLUP);
  
  // Set up stepper motor tracking limits
  SubsystemStepper.setMinPulseWidth(2); 
  SubsystemStepper.setAcceleration(4000); // Sharp, responsive stopping deceleration
  SubsystemStepper.disableOutputs();

  // Attach our precise hardware edge-detection routine to the hall input pin
  attachInterrupt(digitalPinToInterrupt(pin_HALL), hallSensorISR, HALL_TRIGGER_EDGE);

  Serial.println(F("[SYSTEM] Test Environment Online. Send 'G' to run cycle."));
}

void loop() {
  // Continuously process the non-blocking state machine rules
  updateSubsystem();

  // Serial listener to activate the function manually on your bench
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 'G' || input == 'g') {
      startRotationToHallMark();
    }
  }
}