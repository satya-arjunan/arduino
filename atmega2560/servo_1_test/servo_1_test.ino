#include <Servo.h> 

Servo   servoLR;

void setup() { 
	servoLR.attach(2);
} 

void loop() 
{ 
    int pos;
    
    for(pos = 0; pos < 180; pos++) {
        servoLR.write(pos);
        delay(15);
    }
    for(pos = 180; pos > 0; pos--) {
        servoLR.write(pos);
        delay(15);
    }
    servoLR.write(90);

    delay(1000);
}
