/*
 * Copyright (C) re:3D, Inc. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Mixer - Blast Gate Linear Actuator Position Utility
 * ---------------------------------------------------
 * A small serial utility to drive the blast-gate linear actuators (RoboClaw
 * RC-pulse channels) to a target position that you type in.
 *
 * The actuators have NO absolute position feedback - only MIN / MAX limit
 * switches. Position is therefore ESTIMATED from timed motion:
 *   1. Home to MIN  -> that end is 0 %.
 *   2. Calibrate    -> time a full MIN->MAX stroke to learn travel time.
 *   3. Move to N %  -> drive for the time that corresponds to the delta,
 *                      stopping early if a limit switch trips.
 *
 * 0 %   = fully retracted (MIN limit)
 * 100 % = fully extended  (MAX limit)
 *
 * -- Serial commands (9600 baud, newline-terminated) -------------------------
 *   home <L|R|ALL>          Retract to MIN and set that gate to 0 %
 *   homemax <L|R|ALL>       Extend to MAX and set that gate to 100 %
 *   cal  <L|R|ALL>          Home, then time a full stroke to MAX (calibrate)
 *   pos  <L|R> <0-100>      Move a gate to a % of its stroke
 *   ext  <L|R> <ms>         Jog extend  (toward MAX) for <ms> milliseconds
 *   ret  <L|R> <ms>         Jog retract (toward MIN) for <ms> milliseconds
 *   speed <1-100>           Set motion speed as a % of full speed
 *   stop                    Stop both actuators immediately
 *   status                  Print calibration + estimated position
 *   help                    Reprint this command list
 *
 * Any serial byte received during a move aborts it (press Enter to e-stop).
 */

#include <Servo.h>

// ── Pin assignments ─────────────────────────────────────────────────────────
// Limit-switch mapping verified against actual bench presses:
//   LEFT  gate  -> RC D9, limits MIN=D37 MAX=D38
//   RIGHT gate  -> RC D8, limits MIN=D39 MAX=D40
const int GATE_PIN[2]      = { 9, 8 };     // [LEFT, RIGHT] RC pulse output
const int GATE_MIN_PIN[2]  = { 37, 39 };   // [LEFT, RIGHT] MIN limit switch
const int GATE_MAX_PIN[2]  = { 38, 40 };   // [LEFT, RIGHT] MAX limit switch
const char* GATE_NAME[2]   = { "LEFT", "RIGHT" };

const int LEFT  = 0;
const int RIGHT = 1;

// ── RC pulse widths ─────────────────────────────────────────────────────────
const int STOP_US    = 1500;
const int EXTEND_US  = 1000;   // drive toward MAX (fully extended) at full speed
const int RETRACT_US = 2000;   // drive toward MIN (shortest stroke) at full speed

// Motion speed as a percentage of full speed (1-100). Scales how far the RC
// pulse sits from the 1500 us neutral: 100% uses the full extremes above,
// lower values move the pulse closer to neutral for slower travel.
int speedPct = 100;

// ── Motion tunables ─────────────────────────────────────────────────────────
const unsigned long MOVE_TIMEOUT_MS   = 8000;   // guard for any single move
const unsigned long DEFAULT_STROKE_MS = 3000;   // assumed travel time until calibrated
const long          POS_DEADBAND_MS   = 40;     // ignore moves smaller than this
const unsigned long MOVE_TICK_MS      = 5;      // control-loop period

// ── State ───────────────────────────────────────────────────────────────────
Servo servos[2];
unsigned long strokeMs[2]  = { DEFAULT_STROKE_MS, DEFAULT_STROKE_MS };
long          positionMs[2] = { 0, 0 };         // estimated ms of travel from MIN
bool          calibrated[2] = { false, false };

// Edge-tracking for limit-switch press/release reporting.
bool          prevMin[2] = { false, false };
bool          prevMax[2] = { false, false };

String cmdBuffer;

// ── Low-level helpers ───────────────────────────────────────────────────────
bool minHit(int g) { return digitalRead(GATE_MIN_PIN[g]) == LOW; }  // pressed = LOW
bool maxHit(int g) { return digitalRead(GATE_MAX_PIN[g]) == LOW; }

void gateStop(int g)    { servos[g].writeMicroseconds(STOP_US); }
void gateExtend(int g)  {
  int us = STOP_US + (int)((long)(EXTEND_US - STOP_US) * speedPct / 100);
  servos[g].writeMicroseconds(us);
}
void gateRetract(int g) {
  int us = STOP_US + (int)((long)(RETRACT_US - STOP_US) * speedPct / 100);
  servos[g].writeMicroseconds(us);
}

void stopAll() {
  gateStop(LEFT);
  gateStop(RIGHT);
}

// True if the user sent any byte (used to abort a blocking move).
bool abortRequested() {
  if (Serial.available() > 0) {
    while (Serial.available() > 0) Serial.read();  // flush
    return true;
  }
  return false;
}

// Watch all four limit switches and report press / release transitions.
// Called both at idle and inside every move loop, so presses are caught
// at any time.
void monitorLimits() {
  for (int g = 0; g < 2; g++) {
    bool m = minHit(g);
    if (m != prevMin[g]) {
      Serial.print(F("["));
      Serial.print(GATE_NAME[g]);
      Serial.println(m ? F("] MIN limit PRESSED") : F("] MIN limit released"));
      prevMin[g] = m;
    }
    bool x = maxHit(g);
    if (x != prevMax[g]) {
      Serial.print(F("["));
      Serial.print(GATE_NAME[g]);
      Serial.println(x ? F("] MAX limit PRESSED") : F("] MAX limit released"));
      prevMax[g] = x;
    }
  }
}

// ── Homing / calibration ────────────────────────────────────────────────────
bool gateHome(int g) {
  Serial.print(F("["));
  Serial.print(GATE_NAME[g]);
  Serial.println(F("] Homing to MIN..."));

  unsigned long start = millis();
  while (!minHit(g)) {
    monitorLimits();
    if (abortRequested()) { gateStop(g); Serial.println(F("  aborted")); return false; }
    gateRetract(g);
    if (millis() - start > MOVE_TIMEOUT_MS) {
      gateStop(g);
      Serial.println(F("  MIN timeout - check wiring/stroke time"));
      return false;
    }
    delay(MOVE_TICK_MS);
  }
  gateStop(g);
  positionMs[g] = 0;
  Serial.println(F("  at MIN (0%)"));
  return true;
}

bool gateHomeMax(int g) {
  Serial.print(F("["));
  Serial.print(GATE_NAME[g]);
  Serial.println(F("] Homing to MAX..."));

  unsigned long start = millis();
  while (!maxHit(g)) {
    monitorLimits();
    if (abortRequested()) { gateStop(g); Serial.println(F("  aborted")); return false; }
    gateExtend(g);
    if (millis() - start > MOVE_TIMEOUT_MS) {
      gateStop(g);
      Serial.println(F("  MAX timeout - check wiring/stroke time"));
      return false;
    }
    delay(MOVE_TICK_MS);
  }
  gateStop(g);
  positionMs[g] = strokeMs[g];
  Serial.println(F("  at MAX (100%)"));
  return true;
}

bool gateCalibrate(int g) {
  if (!gateHome(g)) return false;

  Serial.print(F("["));
  Serial.print(GATE_NAME[g]);
  Serial.println(F("] Calibrating - timing MIN->MAX..."));

  unsigned long start = millis();
  while (!maxHit(g)) {
    monitorLimits();
    if (abortRequested()) { gateStop(g); Serial.println(F("  aborted")); return false; }
    gateExtend(g);
    if (millis() - start > MOVE_TIMEOUT_MS) {
      gateStop(g);
      Serial.println(F("  MAX timeout - calibration failed"));
      return false;
    }
    delay(MOVE_TICK_MS);
  }
  gateStop(g);

  strokeMs[g]   = millis() - start;
  positionMs[g] = strokeMs[g];
  calibrated[g] = true;

  Serial.print(F("  stroke = "));
  Serial.print(strokeMs[g]);
  Serial.println(F(" ms (now at MAX / 100%)"));
  return true;
}

// ── Positioning ─────────────────────────────────────────────────────────────
void gateMoveToPercent(int g, float pct) {
  pct = constrain(pct, 0.0, 100.0);

  if (!calibrated[g]) {
    Serial.print(F("["));
    Serial.print(GATE_NAME[g]);
    Serial.print(F("] WARNING: not calibrated - using assumed stroke "));
    Serial.print(strokeMs[g]);
    Serial.println(F(" ms. Run 'cal' for accuracy."));
  }

  long targetMs = (long)((float)strokeMs[g] * pct / 100.0);
  long deltaMs  = targetMs - positionMs[g];

  if (labs(deltaMs) < POS_DEADBAND_MS) {
    Serial.print(F("["));
    Serial.print(GATE_NAME[g]);
    Serial.println(F("] Already at target"));
    return;
  }

  bool extending = deltaMs > 0;
  unsigned long moveTime = (unsigned long)labs(deltaMs);

  Serial.print(F("["));
  Serial.print(GATE_NAME[g]);
  Serial.print(F("] Moving to "));
  Serial.print(pct, 1);
  Serial.print(F("% ("));
  Serial.print(extending ? F("extend ") : F("retract "));
  Serial.print(moveTime);
  Serial.println(F(" ms)"));

  unsigned long start = millis();
  bool hitLimit = false;
  while (millis() - start < moveTime) {
    monitorLimits();
    if (abortRequested()) { Serial.println(F("  aborted")); break; }
    if (extending) {
      if (maxHit(g)) { hitLimit = true; break; }
      gateExtend(g);
    } else {
      if (minHit(g)) { hitLimit = true; break; }
      gateRetract(g);
    }
    if (millis() - start > MOVE_TIMEOUT_MS) { Serial.println(F("  move timeout")); break; }
    delay(MOVE_TICK_MS);
  }
  gateStop(g);

  // Update the position estimate from the time actually spent moving.
  unsigned long elapsed = millis() - start;
  positionMs[g] += extending ? (long)elapsed : -(long)elapsed;

  // Snap to a known reference if a limit switch tripped.
  if (minHit(g)) positionMs[g] = 0;
  if (maxHit(g)) positionMs[g] = strokeMs[g];
  positionMs[g] = constrain(positionMs[g], 0L, (long)strokeMs[g]);

  Serial.print(F("  done at ~"));
  Serial.print(gatePercent(g), 1);
  Serial.print(F("%"));
  if (hitLimit) Serial.print(F(" (limit reached)"));
  Serial.println();
}

// Timed jog in one direction; keeps the position estimate in sync.
void gateJog(int g, bool extend, unsigned long ms) {
  Serial.print(F("["));
  Serial.print(GATE_NAME[g]);
  Serial.print(extend ? F("] Extending ") : F("] Retracting "));
  Serial.print(ms);
  Serial.println(F(" ms"));

  unsigned long start = millis();
  while (millis() - start < ms) {
    monitorLimits();
    if (abortRequested()) { Serial.println(F("  aborted")); break; }
    if (extend) {
      if (maxHit(g)) { Serial.println(F("  MAX limit")); break; }
      gateExtend(g);
    } else {
      if (minHit(g)) { Serial.println(F("  MIN limit")); break; }
      gateRetract(g);
    }
    delay(MOVE_TICK_MS);
  }
  gateStop(g);

  unsigned long elapsed = millis() - start;
  positionMs[g] += extend ? (long)elapsed : -(long)elapsed;
  if (minHit(g)) positionMs[g] = 0;
  if (maxHit(g)) positionMs[g] = strokeMs[g];
  positionMs[g] = constrain(positionMs[g], 0L, (long)strokeMs[g]);
}

float gatePercent(int g) {
  if (strokeMs[g] == 0) return 0.0;
  return 100.0 * (float)positionMs[g] / (float)strokeMs[g];
}

void printStatus() {
  for (int g = 0; g < 2; g++) {
    Serial.print(F("["));
    Serial.print(GATE_NAME[g]);
    Serial.print(F("] pos=~"));
    Serial.print(gatePercent(g), 1);
    Serial.print(F("%  stroke="));
    Serial.print(strokeMs[g]);
    Serial.print(F(" ms  cal="));
    Serial.print(calibrated[g] ? F("YES") : F("NO"));
    Serial.print(F("  MIN="));
    Serial.print(minHit(g) ? F("HIT") : F("open"));
    Serial.print(F("  MAX="));
    Serial.println(maxHit(g) ? F("HIT") : F("open"));
  }
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  home <L|R|ALL>       Retract to MIN, set 0%"));
  Serial.println(F("  homemax <L|R|ALL>    Extend to MAX, set 100%"));
  Serial.println(F("  cal  <L|R|ALL>       Home then time a full stroke"));
  Serial.println(F("  pos  <L|R> <0-100>   Move gate to a % of stroke"));
  Serial.println(F("  ext  <L|R> <ms>      Jog extend for <ms>"));
  Serial.println(F("  ret  <L|R> <ms>      Jog retract for <ms>"));
  Serial.println(F("  speed <1-100>        Set motion speed (% of full)"));
  Serial.println(F("  stop                 Stop both actuators"));
  Serial.println(F("  status               Show positions"));
  Serial.println(F("  help                 This list"));
}

// ── Command parsing ─────────────────────────────────────────────────────────
// Returns -1 for invalid gate token, else LEFT/RIGHT (ALL handled separately).
int parseGate(const String& token) {
  String t = token;
  t.toUpperCase();
  if (t == "L" || t == "LEFT")  return LEFT;
  if (t == "R" || t == "RIGHT") return RIGHT;
  return -1;
}

void handleCommand(const String& line) {
  String s = line;
  s.trim();
  if (s.length() == 0) return;

  // Split into up to 3 tokens: verb, arg1, arg2.
  String verb, a1, a2;
  int sp1 = s.indexOf(' ');
  if (sp1 < 0) {
    verb = s;
  } else {
    verb = s.substring(0, sp1);
    String rest = s.substring(sp1 + 1);
    rest.trim();
    int sp2 = rest.indexOf(' ');
    if (sp2 < 0) {
      a1 = rest;
    } else {
      a1 = rest.substring(0, sp2);
      a2 = rest.substring(sp2 + 1);
      a2.trim();
    }
  }
  verb.toLowerCase();

  if (verb == "stop") {
    stopAll();
    Serial.println(F("Stopped both."));
    return;
  }
  if (verb == "status") { printStatus(); return; }
  if (verb == "help")   { printHelp();   return; }

  if (verb == "speed") {
    if (a1.length() == 0) { Serial.println(F("Usage: speed <1-100>")); return; }
    int v = a1.toInt();
    if (v < 1 || v > 100) { Serial.println(F("Speed must be 1-100")); return; }
    speedPct = v;
    Serial.print(F("Speed set to "));
    Serial.print(speedPct);
    Serial.println(F("%"));
    return;
  }
  if (verb == "home" || verb == "homemax" || verb == "cal") {
    String g = a1; g.toUpperCase();
    bool doL = (g == "ALL" || parseGate(a1) == LEFT);
    bool doR = (g == "ALL" || parseGate(a1) == RIGHT);
    if (!doL && !doR) { Serial.println(F("Usage: home/homemax/cal <L|R|ALL>")); return; }
    if (verb == "home") {
      if (doL) gateHome(LEFT);
      if (doR) gateHome(RIGHT);
    } else if (verb == "homemax") {
      if (doL) gateHomeMax(LEFT);
      if (doR) gateHomeMax(RIGHT);
    } else {
      if (doL) gateCalibrate(LEFT);
      if (doR) gateCalibrate(RIGHT);
    }
    return;
  }

  if (verb == "pos") {
    int g = parseGate(a1);
    if (g < 0 || a2.length() == 0) { Serial.println(F("Usage: pos <L|R> <0-100>")); return; }
    gateMoveToPercent(g, a2.toFloat());
    return;
  }

  if (verb == "ext" || verb == "ret") {
    int g = parseGate(a1);
    long ms = a2.toInt();
    if (g < 0 || ms <= 0) { Serial.println(F("Usage: ext/ret <L|R> <ms>")); return; }
    gateJog(g, verb == "ext", (unsigned long)ms);
    return;
  }

  Serial.print(F("Unknown command: "));
  Serial.println(verb);
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdBuffer.length() > 0) {
        handleCommand(cmdBuffer);
        cmdBuffer = "";
      }
    } else {
      cmdBuffer += c;
    }
  }
}

// ── Setup / Loop ────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  for (int g = 0; g < 2; g++) {
    pinMode(GATE_MIN_PIN[g], INPUT_PULLUP);
    pinMode(GATE_MAX_PIN[g], INPUT_PULLUP);
    servos[g].attach(GATE_PIN[g]);
  }
  stopAll();
  delay(2000);   // let the RoboClaw arm on a steady 1500 us

  Serial.println(F("Blast Gate Position Utility ready."));
  Serial.println(F("Tip: run 'cal ALL' first for accurate positioning."));
  printHelp();
}

void loop() {
  readSerial();
  monitorLimits();
}
