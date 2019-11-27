#include "./Gaussian.h"

int Right_motor_back = 7; 
int Right_motor_go = 5; 
int Right_motor_en = 6; 
int Left_motor_back = 4; 
int Left_motor_go = 2; 
int Left_motor_en = 3; 

int orientation = 0;
Gaussian gaussian = Gaussian();
int default_speed = 100; //PWM default_speed

void setup() {
  pinMode(Left_motor_go, OUTPUT);
  pinMode(Left_motor_back, OUTPUT); 
  pinMode(Right_motor_go, OUTPUT); 
  pinMode(Right_motor_back, OUTPUT); 
  pinMode(Right_motor_en, OUTPUT); 
  pinMode(Left_motor_en, OUTPUT); 
  digitalWrite(Left_motor_en, HIGH); // set left motor enble
  digitalWrite(Right_motor_en, HIGH); // set right motor enble
  delay(3000); // walk at selected speed for 500 ms
}

void spin_right() {
  digitalWrite(Right_motor_go, LOW);  // right motor back off
  digitalWrite(Right_motor_back, HIGH);
  // Pulse Width Modulation(0~255) default_speed speed
  analogWrite(Right_motor_en, default_speed);
  digitalWrite(Left_motor_go, HIGH); // left motor go ahead
  digitalWrite(Left_motor_back, LOW);
  // Pulse Width Modulation(0~255) default_speed speed
  analogWrite(Left_motor_en, default_speed);
}

void spin_left() {
  digitalWrite(Right_motor_go, HIGH);  // right motor go ahead
  digitalWrite(Right_motor_back, LOW);
  analogWrite(Right_motor_en, default_speed);
  digitalWrite(Left_motor_go, LOW);  // left motor back off
  digitalWrite(Left_motor_back, HIGH);
  //PWM--Pulse Width Modulation(0~255) default_speed speed
  analogWrite(Left_motor_en, default_speed);
}

void spin_right_angle(int angle) {
  spin_right();
  delay(700.0*angle/6);
}

void spin_left_angle(int angle) {
  spin_left();
  delay(700.0*angle/6);
}

void run_at_speed(int speed) {
  digitalWrite(Right_motor_go, HIGH);
  digitalWrite(Right_motor_back, LOW);
  analogWrite(Right_motor_en, speed*0.99);

  digitalWrite(Left_motor_go, HIGH);
  digitalWrite(Left_motor_back, LOW);
  analogWrite(Left_motor_en, speed);
}

void brake() {
  digitalWrite(Left_motor_back, LOW);
  digitalWrite(Left_motor_go, LOW);
  digitalWrite(Right_motor_go, LOW);
  digitalWrite(Right_motor_back, LOW);
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
  // cell's can't move backwards
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
  run_at_speed(currentSpeed/maxSpeed*150); //200 is the maxSpeed of robot
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
  run_at_speed(currentSpeed/maxSpeed*150); //200 is the maxSpeed of robot
  delay(500); // walk at selected speed for 500 ms
}

void calibrate_wheel_alignment() {
  run_at_speed(default_speed);
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
