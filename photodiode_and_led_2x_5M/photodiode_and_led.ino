int LED = 13;
int LED2 = 9;

const int numReadings = 50;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

int readings2[numReadings];      // the readings from the analog input
int readIndex2 = 0;              // the index of the current reading
int total2 = 0;                  // the running total
int average2 = 0;                // the average

int inputPin = A0;
int inputPin2 = A1;

void setup() {
  pinMode(LED,OUTPUT);
  pinMode(LED2,OUTPUT);
  Serial.begin(9600);
  for (int i(0); i < numReadings; i++) {
    readings[i] = 0;
    readings2[i] = 0;
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
  if (average >= 141) {
    digitalWrite(LED,HIGH);
  }
  else {
    digitalWrite(LED,LOW);
  }

  total2 = total2 - readings2[readIndex2];
  readings2[readIndex2] = analogRead(inputPin2);
  total2 = total2 + readings2[readIndex2];
  readIndex2 = readIndex2 + 1;
  if (readIndex2 >= numReadings) {
    readIndex2 = 0;
  }

  average2 = total2 / numReadings;
  //Serial.println(average2);
  if (average2 >= 103) {
    digitalWrite(LED2,HIGH);
  }
  else {
    digitalWrite(LED2,LOW);
  }
  delay(1);        // delay in between reads for stability
}

/*
  5.6M 180 (low 160, ambience 165, high 190, side angle 180)
  10M (low 290, ambience 297, high 350, side angle 315)
  */
