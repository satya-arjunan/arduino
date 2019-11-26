#include "./Gaussian.h"

const int sensor_size(6);
const int read_size(25);
const int south = A2;
const int south_LED = 11;
const int west = A4;
const int west_LED = 9;
const int north = A5;
const int north_LED = 8;
const int east = A1;
const int east_LED = 12;
const int north_east = A0;
const int north_east_LED = 13;
const int north_west = A3;
const int north_west_LED = 10;
const int north_idx = 0;
const int south_idx = 5;
const int east_idx = 3;
const int west_idx = 4;
const int north_east_idx = 1;
const int north_west_idx = 2;

/*
//calibrate from 6th line front edge
const int north_value = 20;
const int south_value = 27;
const int east_value = 21;
const int west_value = 26;
const int north_east_value = 23;
const int north_west_value = 25;
*/

//calibrate from 5th line front edge, bottom left edge of tape
const int north_value = 30;
const int south_value = 32;
const int east_value = 34;
const int west_value = 42;
const int north_east_value = 35;
const int north_west_value = 35;


const int serial_select(west_idx);
int LED[sensor_size] = {north_LED, north_east_LED, north_west_LED, east_LED,
 west_LED, south_LED};
const int dist_values[6] = {north_value, north_east_value, north_west_value,
 east_value, west_value, south_value}; 

int inputPin[sensor_size] = {north, north_east, north_west, east, west, south};
int readings[sensor_size][read_size];
int total[sensor_size];
int average[sensor_size];
int readIndex(0);


int Right_motor_reverse = 7; 
int Right_motor_forward = 5; 
int Right_motor_enable = 6; 
int Left_motor_reverse = 4; 
int Left_motor_forward = 2; 
int Left_motor_enable = 3;

Gaussian gaussian = Gaussian();
int default_speed = 175; //batcar 150
int max_speed = 255;  //batcar 230
//calibration parameters:
const float speed_right_fraction = 1.0;
const float speed_left_fraction = 0.946;
const float spin_360_left_delay = 1027;
const float spin_360_right_delay = 1028;

void update_sensors_after_1ms() {
  for (unsigned i(0); i < sensor_size; ++i) {
    total[i] -= readings[i][readIndex];
    readings[i][readIndex] = analogRead(inputPin[i]);
    total[i] += readings[i][readIndex];
    average[i] = total[i] / read_size;
    if (i == serial_select) {
      //Serial.println(average[i]);
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
}

void local_delay(int ms) {
  for (unsigned i(0); i < ms/1.72; ++i) {
    update_sensors_after_1ms();
    delay(1);
  }
}


void spin_right() {
  digitalWrite(Right_motor_forward, LOW);
  digitalWrite(Right_motor_reverse, HIGH);
  analogWrite(Right_motor_enable, default_speed);
  digitalWrite(Left_motor_forward, HIGH);
  digitalWrite(Left_motor_reverse, LOW);
  analogWrite(Left_motor_enable, default_speed);
}

void spin_left() {
  digitalWrite(Right_motor_forward, HIGH);
  digitalWrite(Right_motor_reverse, LOW);
  analogWrite(Right_motor_enable, default_speed);
  digitalWrite(Left_motor_forward, LOW);
  digitalWrite(Left_motor_reverse, HIGH);
  analogWrite(Left_motor_enable, default_speed);
}

void spin_right_angle(int angle) {
  spin_right();
  local_delay(spin_360_right_delay/360*angle);
}

void spin_left_angle(int angle) {
  spin_left();
  local_delay(spin_360_left_delay/360*angle);
}

void run_at_speed(int speed) {
  digitalWrite(Right_motor_forward, HIGH);
  digitalWrite(Right_motor_reverse, LOW);
  analogWrite(Right_motor_enable, speed*speed_right_fraction);

  digitalWrite(Left_motor_forward, HIGH);
  digitalWrite(Left_motor_reverse, LOW);
  analogWrite(Left_motor_enable, speed*speed_left_fraction);
}

void brake() {
  digitalWrite(Left_motor_reverse, LOW);
  digitalWrite(Left_motor_forward, LOW);
  digitalWrite(Right_motor_forward, LOW);
  digitalWrite(Right_motor_reverse, LOW);
}

void orient(int new_orientation) {
  int rotation = new_orientation;
  if (rotation < 0) {
    rotation += 360;
  }
  if (rotation <= 180) {
    spin_left_angle(rotation);
  }
  else {
    spin_right_angle(360-rotation);
  }
  brake(); //stop
  local_delay(100);
}

void neutrophil_inverse_crw() {
  double speedStd = 8.5;
  double speedMean = 5.95;
  double attackFactor = 0.55;
  double correlationFactor = 1;
  double pitchRateMean = 2.65;
  double pitchRateStd = 1.5;

  // apply movement to the cell in the direction that it faces 
  gaussian.mean = speedMean;
  gaussian.variance = speedStd*speedStd;
  double currentSpeed = gaussian.random();
  // units in um/min. Ensure not faster than maximum possible neutrophil spd.
  double maxSpeed = 25.0;
  currentSpeed = min(maxSpeed, currentSpeed); 
  // cell's can't move reverse
  currentSpeed = abs(currentSpeed);

  gaussian.mean = pitchRateMean;
  gaussian.variance = pitchRateStd*pitchRateStd;
  double pitch = gaussian.random();
  /* This is the critical bit, relating the last speed to the pitch. 
   * Correlation factor describes the sensitivity to speed.
   * When correlationFactor=0, speed has no influence
   * on pitch. When correlationFactor=1, speed as a proportion of maximum
   * speed completely scales pitch.  
   */
  double scaledMaxSpeed = pow(maxSpeed, attackFactor);
  double scaledSpeed = pow(currentSpeed, attackFactor);
  pitch *= (1.0 - correlationFactor) +
    (correlationFactor * (scaledMaxSpeed - scaledSpeed) / scaledMaxSpeed);
  //pitch *= Simulation.timeSlice;    // account for timestep.    
  // multiply orientation by rotateQ, because pitchQ is calculated relative to
  // cell, not in absolute space.
  double new_orientation = pitch/(M_PI*2)*360; //convert radians to degrees
  orient(new_orientation);
  run_at_speed(currentSpeed/maxSpeed*max_speed);
  local_delay(500); // walk at selected speed for 500 ms
}

void neutrophil_brownian() {
  double speedStd = 10;
  gaussian.mean = 0;
  gaussian.variance = speedStd*speedStd;
  double currentSpeed = gaussian.random();
  double maxSpeed = 25.0;
  currentSpeed = min(maxSpeed, currentSpeed); 
  currentSpeed = abs(currentSpeed);
  orient(random(0, 360));
  run_at_speed(currentSpeed/maxSpeed*max_speed);
  local_delay(500); // walk at selected speed for 500 ms
}

void calibrate_wheel_alignment() {
  run_at_speed(default_speed);
  local_delay(5000);
  brake();
  delay(100000);
}

void calibrate_right_360_angle() {
  spin_right_angle(360);
  brake();
  local_delay(3000);
}

void calibrate_left_360_angle() {
  spin_left_angle(360);
  brake();
  local_delay(3000);
}

void setup_sensors() {
  Serial.begin(9600);
  for (unsigned i(0); i < sensor_size; ++i) {
    pinMode(LED[i], OUTPUT);
    total[i] = 0;
    average[i] = 0;
    for (unsigned j(0); j < read_size; j++) {
      readings[i][j] = 0;
    }
  }
}

void setup_motors() {
  pinMode(Left_motor_forward, OUTPUT);
  pinMode(Left_motor_reverse, OUTPUT); 
  pinMode(Right_motor_forward, OUTPUT); 
  pinMode(Right_motor_reverse, OUTPUT); 
  pinMode(Right_motor_enable, OUTPUT); 
  pinMode(Left_motor_enable, OUTPUT); 
  digitalWrite(Left_motor_enable, HIGH);
  digitalWrite(Right_motor_enable, HIGH);
}

void setup() {
  setup_motors();
  setup_sensors();
  delay(1500);
}


void loop_motors() {
  neutrophil_inverse_crw();
  //neutrophil_brownian();
  //calibrate_wheel_alignment();
  //calibrate_left_360_angle();
  //calibrate_right_360_angle();
}

void loop() {
  loop_motors();
  //update_sensors_after_1ms();
  //delay(1);
}

