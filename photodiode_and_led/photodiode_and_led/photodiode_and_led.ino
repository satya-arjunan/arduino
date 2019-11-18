// Project 9– Light the lamp
int LED = 13;                                     //define LED digital pin 13
int val = 0;                                         //define the voltage value of photo diode in digital pin 0

void setup(){
pinMode(LED,OUTPUT);                // Configure LED as output mode
Serial.begin(9600);                         //Configure baud rate 9600
}

void loop(){
val = analogRead(0);                     // Read voltage value ranging from 0 -1023
Serial.println(val);                         // read voltage value from serial monitor
if(val>=31){                                  // If lower than 1000, turn off LED
digitalWrite(LED,HIGH);
}else{                                              // Otherwise turn on LED
digitalWrite(LED,LOW);
}
                                     // delay for 10ms
}
