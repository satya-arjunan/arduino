#include "./Gaussian.h"

int Right_motor_reverse = 7; 
int Right_motor_forward = 5; 
int Right_motor_en = 6; 
int Left_motor_reverse = 4; 
int Left_motor_forward = 2; 
int Left_motor_en = 3;

int orientation = 0;
Gaussian gaussian = Gaussian();
int default_speed = 230; //batcar 150
int max_speed = 255;  //batcar 230
//calibration parameters:
const float speed_right_fraction = 1.0;
const float speed_left_fraction = 0.93;
const float spin_360_left_delay = 66.5;
const float spin_360_right_delay = 64.7;

void setup() {
  pinMode(Left_motor_forward, OUTPUT);
  pinMode(Left_motor_reverse, OUTPUT); 
  pinMode(Right_motor_forward, OUTPUT); 
  pinMode(Right_motor_reverse, OUTPUT); 
  pinMode(Right_motor_en, OUTPUT); 
  pinMode(Left_motor_en, OUTPUT); 
  digitalWrite(Left_motor_en, HIGH); // set left motor enble
  digitalWrite(Right_motor_en, HIGH); // set right motor enble
  delay(3000); // walk at selected speed for 500 ms
}

void spin_right() {
  digitalWrite(Right_motor_forward, LOW);  // right motor reverse off
  digitalWrite(Right_motor_reverse, HIGH);
  // Pulse Width Modulation(0~255) default_speed speed
  analogWrite(Right_motor_en, default_speed);
  digitalWrite(Left_motor_forward, HIGH); // left motor forward ahead
  digitalWrite(Left_motor_reverse, LOW);
  // Pulse Width Modulation(0~255) default_speed speed
  analogWrite(Left_motor_en, default_speed);
}

void spin_left() {
  digitalWrite(Right_motor_forward, HIGH);  // right motor forward ahead
  digitalWrite(Right_motor_reverse, LOW);
  analogWrite(Right_motor_en, default_speed);
  digitalWrite(Left_motor_forward, LOW);  // left motor reverse off
  digitalWrite(Left_motor_reverse, HIGH);
  //PWM--Pulse Width Modulation(0~255) default_speed speed
  analogWrite(Left_motor_en, default_speed);
}

void spin_right_angle(int angle) {
  spin_right();
  delay(spin_360_right_delay*angle);
}

void spin_left_angle(int angle) {
  spin_left();
  delay(spin_360_left_delay*angle);
}

void run_at_speed(int speed) {
  digitalWrite(Right_motor_forward, HIGH);
  digitalWrite(Right_motor_reverse, LOW);
  analogWrite(Right_motor_en, speed*speed_right_fraction);

  digitalWrite(Left_motor_forward, HIGH);
  digitalWrite(Left_motor_reverse, LOW);
  analogWrite(Left_motor_en, speed*speed_left_fraction);
}

void brake() {
  digitalWrite(Left_motor_reverse, LOW);
  digitalWrite(Left_motor_forward, LOW);
  digitalWrite(Right_motor_forward, LOW);
  digitalWrite(Right_motor_reverse, LOW);
}

void orient(int new_orientation) {
  int rotation = new_orientation-orientation;
  if (rotation < 0) {
    rotation += 12;
  }
  if (rotation <= 5) {
    spin_left_angle(rotation);
  }
  else {
    spin_right_angle(12-rotation);
  }
  brake(); //stop
  delay(100);
  orientation = new_orientation;
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
  double new_orientation = orientation+pitch/(M_PI*2)*12;
  orient(new_orientation);
  run_at_speed(currentSpeed/maxSpeed*max_speed); //200 is the maxSpeed of robot
  delay(500); // walk at selected speed for 500 ms
}

void neutrophil_brownian() {
  double speedStd = 10;
  gaussian.mean = 0;
  gaussian.variance = speedStd*speedStd;
  double currentSpeed = gaussian.random();
  double maxSpeed = 25.0;
  currentSpeed = min(maxSpeed, currentSpeed); 
  currentSpeed = abs(currentSpeed);
  orient(random(0, 12));
  run_at_speed(currentSpeed/maxSpeed*max_speed); //200 is the maxSpeed of robot
  delay(500); // walk at selected speed for 500 ms
}

void calibrate_wheel_alignment() {
  run_at_speed(default_speed);
  delay(2000);
  brake();
  delay(100000);
}

void calibrate_right_360_angle() {
  spin_right_angle(2*M_PI);
  brake();
  delay(3000);
}

void calibrate_left_360_angle() {
  spin_left_angle(2*M_PI);
  brake();
  delay(3000);
}

void loop() {
  neutrophil_inverse_crw();
  //neutrophil_brownian();
  //calibrate_wheel_alignment();
  //calibrate_left_360_angle();
  //calibrate_right_360_angle();
}

//power robot parameters
//at speed 100, left should be 0.95 of total speed
//set default_speed to 100
//for 360 degree turns, use 2*MPI
//max speed in inverse_crw and brownian to 150
/*
void spin_right_angle(int angle) {
  spin_right();
  delay(782.0*angle/6);
}

void spin_left_angle(int angle) {
  spin_left();
  delay(780.0*angle/6);
}
 */

//slow robot parameters
//at speed 250, right should be 0.99 of total speed
//set default_speed to 250
//for 360 degree turns, use 2*MPI
//max speed in inverse_crw and brownian to 150

/*
void spin_right_angle(int angle) {
  spin_right();
  delay(1020.0*angle/6);
}

void spin_left_angle(int angle) {
  spin_left();
  delay(1020.0*angle/6);
}
 */

