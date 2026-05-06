#include <Servo.h>

Servo servo1;
Servo servo2;
const int servo1pin = 9;
const int servo2pin = 10;

void setup() {
  servo1.attach(servo1pin);  
  servo2.attach(servo2pin);  
  
}

void loop() {
    servo1.write(0);
    servo2.write(0);
    delay(5000);
    servo1.write(55); //max position is 55
    servo2.write(55);
    delay(5000);
}
