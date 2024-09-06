#include<Servo.h>

Servo servo;
int pos = 0;    // variable to store the servo position


void setup() {
  // put your setup code here, to run once:
  servo.attach(5, 500, 2400);//D1 of NodeMcu
  servo.write(0);
  delay(2000);
}


void loop() {
  // goes from 0 degrees to 180 degrees
  for (pos = 1; pos <= 180; pos += 1) {
    // in steps of 1 degree
    servo.write(pos);  // tell servo to go to position in variable 'pos'
    delay(15);   // waits 15ms for the servo to reach the position
  }
  // goes from 180 degrees to 0 degrees
  for (pos = 180; pos >= 1; pos -= 1) {
    servo.write(pos); // tell servo to go to position in variable 'pos'
    delay(15);  // waits 15ms for the servo to reach the position
  }
}
