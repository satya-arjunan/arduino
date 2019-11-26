const int size(6);
const int read_size(25);
const int south = A2;
const int south_LED = 11;
const int west = A3;
const int west_LED = 10;
const int north = A5;
const int north_LED = 8;
const int east = A1;
const int east_LED = 12;
const int north_east = A0;
const int north_east_LED = 13;
const int north_west = A4;
const int north_west_LED = 9;
const int north_value = 35;
const int south_value = 35;
const int east_value = 34;
const int west_value = 37;
const int north_east_value = 34;
const int north_west_value = 39;
const int north_idx = 0;
const int south_idx = 5;
const int east_idx = 3;
const int west_idx = 4;
const int north_east_idx = 1;
const int north_west_idx = 2;

//calibrate from 5th line front edge, bottom left edge of tape

const int serial_select(north_idx);
int LED[size] = {north_LED, north_east_LED, north_west_LED, east_LED, west_LED,
  south_LED};
const int dist_values[6] = {north_value, north_east_value, north_west_value,
 east_value, west_value, south_value}; 

int inputPin[size] = {north, north_east, north_west, east, west, south};
int readings[size][read_size];
int total[size];
int average[size];
int readIndex(0);

void setup() {
  Serial.begin(9600);
  for (unsigned i(0); i < size; ++i) {
    pinMode(LED[i], OUTPUT);
    total[i] = 0;
    average[i] = 0;
    for (unsigned j(0); j < read_size; j++) {
      readings[i][j] = 0;
    }
  }
}

void loop() {
  for (unsigned i(0); i < size; ++i) {
    total[i] -= readings[i][readIndex];
    readings[i][readIndex] = analogRead(inputPin[i]);
    total[i] += readings[i][readIndex];
    average[i] = total[i] / read_size;
    if (i == serial_select) {
      Serial.println(average[i]);
    }
    if (average[i] >= dist_values[i]) {
      digitalWrite(LED[i], HIGH);
    }
    else {
      digitalWrite(LED[i], LOW);
    }
  }
  readIndex = readIndex + 1;
  if (readIndex >= read_size) {
    readIndex = 0;
  }
  delay(1);
}

/*
  5.6M 180 (low 160, ambience 165, high 190, side angle 180)
  10M (low 290, ambience 297, high 350, side angle 315)
  */
