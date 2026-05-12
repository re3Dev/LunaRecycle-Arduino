#include <Servo.h>

Servo Mixer_shredderGateLeftServoMotor;
Servo Mixer_shredderGateRightServoMotor;
const int Mixer_shredderGateLeftServoMotor_pin = 9;
const int Mixer_shredderGateRightServoMotor_pin = 10;

void setup() {
  Mixer_shredderGateLeftServoMotor.attach(Mixer_shredderGateLeftServoMotor_pin);  
  Mixer_shredderGateRightServoMotor.attach(Mixer_shredderGateRightServoMotor_pin);  
  
}

void loop() {
    Mixer_shredderGateLeftServoMotor.write(0);
    Mixer_shredderGateRightServoMotor.write(0);
    delay(5000);
    Mixer_shredderGateLeftServoMotor.write(55); //max position is 55
    Mixer_shredderGateRightServoMotor.write(55);
    delay(5000);
}
