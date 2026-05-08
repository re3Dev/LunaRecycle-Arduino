#include <Servo.h>

const int SERVO_PIN = 9;
const unsigned long CYCLE_INTERVAL_MS = 5;

Servo servo;
String command = "";
bool cycling = false;
int currentAngle = 0;
int cycleDirection = 1;
unsigned long lastCycleMove = 0;

void setup() {
  Serial.begin(9600);

  servo.attach(SERVO_PIN);
  servo.write(currentAngle);

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
  servo.write(currentAngle);

  Serial.print("Moved servo to ");
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

  servo.write(currentAngle);
}
