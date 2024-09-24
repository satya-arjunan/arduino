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
 for (pos = 10; pos <= 80; pos += 20) { 
    servo.write(pos); 
    delay(15);
  }
  for (pos = 80; pos >= 10; pos -= 10) {
    servo.write(pos);
    delay(15);
  }
  for (posi = 10; posi <=80; posi += 10) {
    servoa.write(posi);
    delay(15);
    }
  for (posi = 80; posi >= 10; posi -= 10) {
    servoa.write(posi);
    delay(15);
  }
}
