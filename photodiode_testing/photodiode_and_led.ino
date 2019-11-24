int analogPin = A0 ; // potentiometer wiper (middle terminal) connected to analog pin 3
                    // outside leads to ground and +5V
int cnt(1000);

void setup() {
  Serial.begin(9600);           //  setup serial
}

void loop() {
  /*
  int val(0);
  for (int i(0); i < cnt; ++i) {
    val += analogRead(analogPin);  // read the input pin
    delay(1);
  }
  */
  int val = analogRead(analogPin);  // read the input pin
  Serial.println(val);          // debug value
}
