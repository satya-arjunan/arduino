#include<Servo.h>

Servo servo;
Servo servoa;
int pos = 0;    // variable to store the servo position
int posi = 0;

void setup() {
  // put your setup code here, to run once:
  servo.attach(5, 500, 2400);//D1 of NodeMcu
  servo.write(0);
  delay(1000);
  servoa.attach(0, 500, 2400);
  servoa.write(0);
  delay(1000);
}


void loop() {
 for (pos = 1; pos <= 90; pos += 1) { 
    servo.write(pos); 
    delay(15);
  }
  for (pos = 90; pos >= 1; pos -= 1) {
    servo.write(pos);
    delay(15);
  }
  for (posi = 1; posi <=90; posi += 1) {
    servoa.write(posi);
    delay(15);
    }
  for (posi = 90; posi >= 1; posi -= 1) {
    servoa.write(posi);
    delay(15);
  }
}
