#include <Servo.h>

const int TC_servoMotor_pin = 9;
const unsigned long CYCLE_INTERVAL_MS = 5;

Servo TC_servoMotor;
String command = "";
bool cycling = false;
int currentAngle = 0;
int cycleDirection = 1;
unsigned long lastCycleMove = 0;

void setup() {
  Serial.begin(9600);

  TC_servoMotor.attach(TC_servoMotor_pin);
  TC_servoMotor.write(currentAngle);

  Serial.println("Servo ready at 0 degrees.");
  Serial.println("Enter an angle from 0 to 180, or type cycle:");
}

void loop() {
  readSerialCommand();
  updateCycle();
}

void readSerialCommand() {
  while (Serial.available() > 0) {
    char incoming = Serial.read();

    if (incoming == '\n' || incoming == '\r') {
      if (command.length() > 0) {
        handleCommand(command);
        command = "";
      }
    } else {
      command += incoming;
    }
  }
}

void handleCommand(String input) {
  input.trim();
  input.toLowerCase();

  if (input == "cycle") {
    cycling = true;
    lastCycleMove = millis();
    Serial.println("Cycling between 0 and 180 degrees. Send any angle to stop.");
    return;
  }

  int angle = constrain(input.toInt(), 0, 180);
  cycling = false;
  currentAngle = angle;
  TC_servoMotor.write(currentAngle);

  Serial.print("Moved TC_servoMotor to ");
  Serial.print(currentAngle);
  Serial.println(" degrees.");
}

void updateCycle() {
  if (!cycling || millis() - lastCycleMove < CYCLE_INTERVAL_MS) {
    return;
  }

  lastCycleMove = millis();
  currentAngle += cycleDirection;

  if (currentAngle >= 180) {
    currentAngle = 180;
    cycleDirection = -1;
  } else if (currentAngle <= 0) {
    currentAngle = 0;
    cycleDirection = 1;
  }

  TC_servoMotor.write(currentAngle);
}
