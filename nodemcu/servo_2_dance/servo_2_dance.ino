#include<Servo.h>

Servo servo1;
Servo servo2;


void setup() {
  // put your setup code here, to run once:
  servo1.attach(5, 500, 2400);//D1 of NodeMcu
  servo1.write(0);
  delay(1000);
  servo2.attach(0, 500, 2400);
  servo2.write(0);
  delay(1000);
}

void move(Servo& servo, int start, int end, int inc, int d) {
 for (int pos = start; pos <= end; pos += inc) { 
    servo.write(pos); 
    delay(d);
  }
}


void loop() {
  move(servo2, 10, 30, 5, 0);
  delay(300);
}
