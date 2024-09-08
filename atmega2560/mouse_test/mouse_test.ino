#include <Mouse.h>

void setup() {
  pinMode(2, INPUT);
}

void loop() {
  //initiate the Mouse library when button is pressed
  if (digitalRead(2) == HIGH) {
    Mouse.begin();
  }
}