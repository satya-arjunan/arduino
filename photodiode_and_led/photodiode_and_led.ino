int LED = 13;

const int numReadings = 50;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

int inputPin = A0;

void setup() {
  pinMode(LED,OUTPUT);
  Serial.begin(9600);
  for (int i(0); i < numReadings; i++) {
    readings[i] = 0;
  }
}

void loop() {
  total = total - readings[readIndex];
  readings[readIndex] = analogRead(inputPin);
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }

  average = total / numReadings;
  Serial.println(average);
  if (average >= 320) {
    digitalWrite(LED,HIGH);
  }
  else {
    digitalWrite(LED,LOW);
  }
  delay(1);        // delay in between reads for stability
}

/*
  5.6M 180 (low 160, ambience 165, high 190, side angle 180)
  10M (low 290, ambience 297, high 350, side angle 315)
  */
