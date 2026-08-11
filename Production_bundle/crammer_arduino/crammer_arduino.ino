/*
  Leadshine CS1-D507S -- Dual-Exponent Power-Law Clutch Engine
  -------------------------------------------------------------
  STEP  -> Pin 9
  ENC A -> Pin 18  (Hardware Interrupt)
  ENC B -> Pin 19  (Hardware Interrupt)
 
  G = Run | S = Stop

  REQUIREMENT: Set PA0.01 = 2 (Closed-Loop) and PA0.00 = 10 (2000 Steps/Rev).
*/

const byte STEP_PIN  = 9;
const byte ENC_A_PIN = 18;
const byte ENC_B_PIN = 19;

const int DRIVER_STEPS = 2000;      // Matches PA0.00 microstepping
const int ENCODER_COUNTS = 4000;    // 4x Quadrature Resolution (1000 lines * 4)

volatile long encoderCount = 0;
volatile long commandedSteps = 0;
volatile bool pulseState = false;
volatile bool running = false;
volatile bool timerActive = false;

String cmdBuffer;

// ------------------------
// KINEMATIC CONFIGURATION
// ------------------------
const float MAX_RPM = 60.0;         // Configured to your 180 RPM profile
volatile float currentRPM = 0;
volatile float targetRPM  = MAX_RPM;
const float ACCEL = 150.0;           // Linear acceleration rate (RPM/sec)

// ----------------------------------------
// DETERMINISTIC CLUTCH CONFIGURATION
// ----------------------------------------
const float LAG_START = 44.0;        // Hardcoded baseline free-running noise floor from spreadsheet
float lagStop;                       // Calculated ONCE during setup based on MAX_RPM

// ---------------------------------------------------------
// TUNABLE DUAL-EXPONENT POWER LAW CONFIGURATION
// ---------------------------------------------------------
const float BREAK_LAG_PCT  = 0.05;   // 0.05 = Transition breakpoint happens at 5% of the tick window
const float BREAK_LOAD_PCT = 0.43;   // 0.43 = Command exactly a 43% speed droop at that breakpoint

const float EXPONENT_1     = 0.20;   // Your preferred hyper-responsive initial bite exponent
const float EXPONENT_2     = 1.20;   // Smooth, progressive landing exponent to eliminate the stall cliff
// ---------------------------------------------------------

// ----------------------------------------
// TELEMETRY FILTER VARIABLE
// ----------------------------------------
float smoothedLoad = 0.0;            
float lastPhysRPM = 0.0;
float lastLagTicks = 0.0;
float lastDisplayLoad = 0.0;
unsigned long lastUpdate = 0;
unsigned long lastPrintTime = 0;
long lastEncoderCount = 0;
long lastEncoderPrintCount = 0;

// ---------------------------------------------------------
// Direction-Aware 4x Quadrature Decoding Interrupt Handlers
// ---------------------------------------------------------
void encoderA_ISR()
{
    if (digitalRead(ENC_A_PIN) == digitalRead(ENC_B_PIN)) {
        encoderCount--;
    } else {
        encoderCount++;
    }
}

void encoderB_ISR()
{
    if (digitalRead(ENC_A_PIN) != digitalRead(ENC_B_PIN)) {
        encoderCount--;
    } else {
        encoderCount++;
    }
}

ISR(TIMER1_COMPA_vect)
{
    if (!running || !timerActive)
        return;

    pulseState = !pulseState;
    digitalWrite(STEP_PIN, pulseState);

    if (!pulseState)
        commandedSteps++;
}

void updateTimerFrequency(float rpm)
{
    if (rpm < 1.0)
    {
        timerActive = false;
        digitalWrite(STEP_PIN, LOW);
        return;
    }

    float stepsPerSecond = rpm * DRIVER_STEPS / 60.0;
    float interruptsPerSecond = stepsPerSecond * 2.0;

    unsigned long compare = (2000000.0 / interruptsPerSecond) - 1;

    if (compare < 10) compare = 10;
    if (compare > 65500UL) compare = 65500UL;

    noInterrupts();
    OCR1A = compare;
    if (TCNT1 > OCR1A) {
        TCNT1 = 0;
    }
    timerActive = true;
    interrupts();
}

void startRun()
{
    noInterrupts();
    commandedSteps = 0;
    encoderCount = 0;
    currentRPM = 0.0;
    targetRPM = MAX_RPM;
    interrupts();

    smoothedLoad = 0.0;
    lastPhysRPM = 0.0;
    lastLagTicks = 0.0;
    lastDisplayLoad = 0.0;
    lastEncoderCount = 0;
    running = true;
    lastUpdate = millis();
    lastPrintTime = millis();
    lastEncoderPrintCount = 0;
    Serial.println(F("[CRAMMER] state=RUN"));
}

void stopRun()
{
    running = false;
    timerActive = false;
    digitalWrite(STEP_PIN, LOW);
    currentRPM = 0.0;
    targetRPM = 0.0;
    Serial.println(F("[CRAMMER] state=STOP"));
}

void printStatus()
{
    Serial.print(F("[CRAMMER] running="));
    Serial.print(running ? 1 : 0);
    Serial.print(F(" load_pct="));
    Serial.print(lastDisplayLoad, 1);
    Serial.print(F(" phys_rpm="));
    Serial.print(lastPhysRPM, 2);
    Serial.print(F(" out_rpm="));
    Serial.print(currentRPM, 2);
    Serial.print(F(" lag_ticks="));
    Serial.println(lastLagTicks, 1);
}

void handleCommand(const String& cmd)
{
    String u = cmd;
    u.trim();
    u.toUpperCase();
    if (u.length() == 0) {
        return;
    }

    if (u == "G" || u == "RUN") {
        startRun();
    } else if (u == "S" || u == "STOP") {
        stopRun();
    } else if (u == "STATUS" || u == "L" || u == "LOAD") {
        printStatus();
    } else if (u == "PING") {
        Serial.println(F("[CRAMMER] pong=1"));
    } else {
        Serial.print(F("[CRAMMER] ERROR unknown="));
        Serial.println(u);
    }
}

void readSerialCommands()
{
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdBuffer.length() > 0) {
                handleCommand(cmdBuffer);
                cmdBuffer = "";
            }
        } else {
            cmdBuffer += c;
            if (cmdBuffer.length() > 48) {
                cmdBuffer = "";
            }
        }
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(STEP_PIN, OUTPUT);
    digitalWrite(STEP_PIN, LOW);

    pinMode(ENC_A_PIN, INPUT_PULLUP);
    pinMode(ENC_B_PIN, INPUT_PULLUP);
   
    attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderB_ISR, CHANGE);

    TCCR1A = 0; TCCR1B = 0;
    TCCR1B |= (1 << WGM12); // CTC Mode
    TCCR1B |= (1 << CS11);  // Prescaler 8 (2 MHz base clock)
    OCR1A = 312;            

    TIMSK1 |= (1 << OCIE1A);

    // Compute deterministic stall lag bound ONCE based strictly on MAX_RPM
    lagStop = (16.636 * MAX_RPM) - 4.607;

    Serial.println(F("=================================================="));
    Serial.println(F(" 4x Quadrature Dual-Exponent Clutch Engine Active "));
    Serial.print(F("[CONFIG CHECK] Target Cruise Ceiling : ")); Serial.print(MAX_RPM, 1); Serial.println(F(" RPM"));
    Serial.print(F("[CONFIG CHECK] Fixed Stall Threshold : ")); Serial.print(lagStop, 1);  Serial.println(F(" Ticks"));
    Serial.println(F("=================================================="));
}

void loop()
{
    readSerialCommands();

    if (!running)
        return;

    if (millis() - lastUpdate >= 10) // 10ms processing frame
    {
        float dt = (millis() - lastUpdate) / 1000.0;
        lastUpdate = millis();

        // 1. Snapshot synchronized tracking registers
        noInterrupts();
        long currentSteps = commandedSteps;
        long currentEncoder = encoderCount;
        interrupts();

        // 2. High-Speed Telemetry Velocity Aggregation Slicing
        long deltaTicks = currentEncoder - lastEncoderCount;
        lastEncoderCount = currentEncoder;
       
        float instPhysRPM = ((float)deltaTicks * 60.0) / ((float)ENCODER_COUNTS * dt);
        if (instPhysRPM < 0.0) instPhysRPM = -instPhysRPM;
       
        // Calculate inverted instantaneous load mapping (100% at Standstill, 0% at MAX_RPM)
        float instLoad = (1.0 - (instPhysRPM / MAX_RPM)) * 100.0;
        if (instLoad < 0.0)   instLoad = 0.0;
        if (instLoad > 100.0) instLoad = 100.0;
       
        // Apply Continuous Exponential Moving Average Filter (Alpha = 0.05 at 100Hz)
        smoothedLoad = (0.05 * instLoad) + (0.95 * smoothedLoad);

        // 3. Calculate real-time tracking lag (Clean 2:1 hardware scalar ratio)
        float expectedTicks = (float)currentSteps * 2.0;
        float lag = expectedTicks - (float)currentEncoder;
        if (lag < 0.0) lag = 0.0;
        lastLagTicks = lag;

        // 4. Dual-Exponent Power-Law Speed Droop Architecture Control Math
        float factor = 0.0;
        if (lag <= LAG_START)
        {
            targetRPM = MAX_RPM;
        }
        else if (lag >= lagStop)
        {
            targetRPM = 0.0;
            factor = 1.0;
        }
        else
        {
            // Find where we sit linearly inside the total hardware bucket (0.0 to 1.0)
            float linearFactor = (lag - LAG_START) / (lagStop - LAG_START);
           
            if (linearFactor < BREAK_LAG_PCT)
            {
                // Zone 1: The original high-sensitivity power-law bite
                float normalizedZone1 = linearFactor / BREAK_LAG_PCT;
                factor = BREAK_LOAD_PCT * pow(normalizedZone1, EXPONENT_1);
            }
            else
            {
                // Zone 2: Smooth progressive second-stage power-law transition to full stall
                float normalizedZone2 = (linearFactor - BREAK_LAG_PCT) / (1.0 - BREAK_LAG_PCT);
                factor = BREAK_LOAD_PCT + (1.0 - BREAK_LOAD_PCT) * pow(normalizedZone2, EXPONENT_2);
            }
           
            targetRPM = MAX_RPM * (1.0 - factor);
        }

        // 5. Asymmetrical Acceleration Engine
        if (currentRPM < targetRPM)
        {
            float maxChange = ACCEL * dt;
            currentRPM += maxChange;
            if (currentRPM > targetRPM) currentRPM = targetRPM;
        }
        else if (currentRPM > targetRPM)
        {
            currentRPM = targetRPM;
        }

        updateTimerFrequency(currentRPM);

        // Telemetry Monitor Output
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 250)
        {
            unsigned long printDtMs = millis() - lastPrintTime;
            lastPrintTime = millis();
            long printDeltaTicks = currentEncoder - lastEncoderPrintCount;
            lastEncoderPrintCount = currentEncoder;
           
            float physRPM = ((float)printDeltaTicks * 60.0) / ((float)ENCODER_COUNTS * (printDtMs / 1000.0));
            if (abs(physRPM) < 0.5) physRPM = 0.0;
            lastPhysRPM = physRPM;
           
            float finalDisplayLoad = smoothedLoad;
            if (finalDisplayLoad < 0.0)   finalDisplayLoad = 0.0;
            if (finalDisplayLoad > 100.0) finalDisplayLoad = 100.0;
            lastDisplayLoad = finalDisplayLoad;
           
            lastPrint = millis();
            Serial.print(F("[CRAMMER_TLM] phys_rpm="));      Serial.print(physRPM, 1);
            Serial.print(F(" out_rpm="));                    Serial.print(currentRPM, 1);
            Serial.print(F(" lag_ticks="));                  Serial.print(lag, 1);
            Serial.print(F(" load_pct="));                   Serial.println(finalDisplayLoad, 0);
        }
    }
}
