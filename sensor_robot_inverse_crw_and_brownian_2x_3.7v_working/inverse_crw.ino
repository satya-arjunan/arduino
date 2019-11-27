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

const int led_sequence[sensor_size] = {north_idx, north_east_idx, east_idx,
  south_idx, west_idx, north_west_idx};

const float spin_threshold(1.0);

//calibrate angles
const float north_angle = 0;
const float north_east_angle = -59;
const float north_west_angle = 57;
const float east_angle = -105;
const float west_angle = 95;
const float south_angle = 172;
const float sensor_angles[sensor_size] = {north_angle, north_east_angle, north_west_angle, east_angle, west_angle, south_angle};

/*
//calibrate from 6th line front edge
const int north_value = 20;
const int south_value = 27;
const int east_value = 21;
const int west_value = 26;
const int north_east_value = 23;
const int north_west_value = 25;
*/

/*
//calibrate from 5th line front edge, bottom left edge of tape (night)
const int north_value = 30;
const int south_value = 32;
const int east_value = 34;
const int west_value = 42;
const int north_east_value = 35;
const int north_west_value = 35;
*/

//calibrate from 6th line front edge (13:14)
const int north_value = 36;
const int south_value = 49;
const int east_value = 39;
const int west_value = 49;
const int north_east_value = 47;
const int north_west_value = 43;

//calibrate main lights on
const int north_illum = 30;
const int south_illum = 18;
const int east_illum = 22;
const int west_illum = 20;
const int north_east_illum = 49;
const int north_west_illum = 52;

//calibrate near gradient
const int north_source = 75;
const int south_source = 16;
const int east_source = 20;
const int west_source = 12;
const int north_east_source = 172;
const int north_west_source = 64;

//calibrate from 6th line front edge facing north
const int north_north_value = 37;
const int south_north_value = 17;
const int east_north_value = 14;
const int west_north_value = 12;
const int north_east_north_value = 27;
const int north_west_north_value = 21;

const int illum_total = north_illum+south_illum+east_illum+west_illum+north_east_illum+north_west_illum; 
const int illum_difference = 52-22;

const bool enable_serial(false);
const int serial_select(west_idx);

int state = 0;


int LED[sensor_size] = {north_LED, north_east_LED, north_west_LED, east_LED,
 west_LED, south_LED};
const float sensor_values[6] = {north_value, north_east_value, north_west_value,
 east_value, west_value, south_value}; 

const float sensor_illums[6] = {north_illum, north_east_illum, north_west_illum,
 east_illum, west_illum, south_illum}; 

int inputPin[sensor_size] = {north, north_east, north_west, east, west, south};
int readings[sensor_size][read_size];
float total[sensor_size];
float average[sensor_size];
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

bool spin_toward_sensor() {
  float max_ratio(0);
  int sensor_idx(0);
  for (unsigned i(0); i < sensor_size; ++i) {
    float ratio(average[i]/sensor_values[i]);
    if (ratio > max_ratio) {
      max_ratio = ratio;
      sensor_idx = i;
    }
  }
  if (max_ratio > spin_threshold) {
    orient(sensor_angles[sensor_idx]);
    return true;
  }
  return false;
}

void update_state() {
  float minv(1024);
  float maxv(0);
  float total_average(0);
  for (unsigned i(0); i < sensor_size; ++i) {
    total_average += average[i];
    if (average[i] > maxv) {
      maxv = average[i];
    }
    if (average[i] < minv) {
      minv = average[i];
    }
  }
  float difference((maxv-minv)/total_average);
  if (difference > 0.639) {
    state = 2;
  }
  else if (difference > 0.27) {
    state = 1;
  }
  else {
    state = 0;
  }
}

void update_sensors_after_1ms() {
  for (unsigned i(0); i < sensor_size; ++i) {
    total[i] -= readings[i][readIndex];
    readings[i][readIndex] = analogRead(inputPin[i]);
    total[i] += readings[i][readIndex];
    average[i] = total[i] / read_size;
    if (enable_serial && i == serial_select) {
      Serial.println(average[i]);
    }
    if (state != 2) {
      if (average[i] >= sensor_values[i] || state == 0) {
        digitalWrite(LED[i], HIGH);
      }
      else {
        digitalWrite(LED[i], LOW);
      }
    }
  }
  readIndex = readIndex + 1;
  if (readIndex >= read_size) {
    readIndex = 0;
  }
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

void calibrate_spin_north_east() {
  orient(360+north_east_angle);
  brake();
  local_delay(3000);
}

void calibrate_spin_north_west() {
  orient(north_west_angle);
  brake();
  local_delay(3000);
}

void calibrate_spin_east() {
  orient(360+east_angle);
  brake();
  local_delay(3000);
}

void calibrate_spin_west() {
  orient(west_angle);
  brake();
  local_delay(3000);
}

void calibrate_spin_south() {
  orient(south_angle);
  brake();
  local_delay(3000);
}

void setup_sensors() {
  if (enable_serial) {
    Serial.begin(9600);
  }
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

void ring_led() {
  for (unsigned n(0); n < 5; ++n) {
    for (unsigned i(0); i < sensor_size; ++i) {
      for (unsigned j(0); j < sensor_size; ++j) {
        if (i == j) {
          digitalWrite(LED[led_sequence[j]], HIGH);
        }
        else {
          digitalWrite(LED[led_sequence[j]], LOW);
        }
      }
      local_delay(70);
    }
  }
}


void loop_motors() {
  neutrophil_inverse_crw();
  //neutrophil_brownian();
  //calibrate_wheel_alignment();
  //calibrate_left_360_angle();
  //calibrate_right_360_angle();
  //calibrate_spin_north_east();
  //calibrate_spin_north_west();
  //calibrate_spin_east();
  //calibrate_spin_west();
  //calibrate_spin_south();
}

void loop() {
  loop_motors();
  update_state();
  if (state == 1) {
    if (spin_toward_sensor()) {
      brake();
      local_delay(500);
    }
  }
  else if (state == 2) {
    brake();
    ring_led();
    state = 1;
  }
  //update_sensors_after_1ms();
  delay(1);
}

