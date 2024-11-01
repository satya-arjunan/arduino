#include <Servo.h> 
#include <Arduino.h>
#include <PS2X_lib.h>

#define PIN_RR_COXA         44  //Rear Right leg Hip Horizontal
#define PIN_RR_FEMUR        45  //Rear Right leg Hip Vertical
#define PIN_RR_TIBIA        46  //Rear Right leg Knee

#define PIN_RM_COXA         35  //Middle Right leg Hip Horizontal
#define PIN_RM_FEMUR        33  //Middle Right leg Hip Vertical
#define PIN_RM_TIBIA        36  //Middle Right leg Knee

#define PIN_RF_COXA         12 //Front Right leg Hip Horizontal
#define PIN_RF_FEMUR        32  //Front Right leg Hip Vertical
#define PIN_RF_TIBIA        34  //Front Right leg Knee

#define PIN_LR_COXA         9  //Rear Left leg Hip Horizontal
#define PIN_LR_FEMUR        10  //Rear Left leg Hip Vertical
#define PIN_LR_TIBIA        11  //Rear Left leg Knee

#define PIN_LM_COXA         6  //Middle Left leg Hip Horizontal
#define PIN_LM_FEMUR        7  //Middle Left leg Hip Vertical
#define PIN_LM_TIBIA        8  //Middle Left leg Knee

#define PIN_LF_COXA         2  //Front Left leg Hip Horizontal
#define PIN_LF_FEMUR        3  //Front Left leg Hip Vertical
#define PIN_LF_TIBIA        5  //Front Left leg Knee

#define PS2_CMD      A0        
#define PS2_DAT      A1
#define PS2_SEL      A2
#define PS2_CLK      A3

Servo lf_coxa;
Servo lf_femur;
Servo lf_tibia;
Servo lm_coxa;
Servo lm_femur;
Servo lm_tibia;
Servo lr_coxa;
Servo lr_femur;
Servo lr_tibia;

Servo rf_coxa;
Servo rf_femur;
Servo rf_tibia;
Servo rm_coxa;
Servo rm_femur;
Servo rm_tibia;
Servo rr_coxa;
Servo rr_femur;
Servo rr_tibia;

PS2X ps2x;

void init_servos() {
  lf_coxa.attach(PIN_LF_COXA);
  lf_femur.attach(PIN_LF_FEMUR);
  lf_tibia.attach(PIN_LF_TIBIA);
  lm_coxa.attach(PIN_LM_COXA);
  lm_femur.attach(PIN_LM_FEMUR);
  lm_tibia.attach(PIN_LM_TIBIA);
  lr_coxa.attach(PIN_LR_COXA);
  lr_femur.attach(PIN_LR_FEMUR);
  lr_tibia.attach(PIN_LR_TIBIA);

  rf_coxa.attach(PIN_RF_COXA);
  rf_femur.attach(PIN_RF_FEMUR);
  rf_tibia.attach(PIN_RF_TIBIA);
  rm_coxa.attach(PIN_RM_COXA);
  rm_femur.attach(PIN_RM_FEMUR);
  rm_tibia.attach(PIN_RM_TIBIA);
  rr_coxa.attach(PIN_RR_COXA);
  rr_femur.attach(PIN_RR_FEMUR);
  rr_tibia.attach(PIN_RR_TIBIA);
}

void init_ps2() {
  int error;
  // Setup gamepad (clock, command, attention, data) pins
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT);
}

void rotate_servo(Servo& servo) {
  servo.write(0);
  delay(50);
  servo.write(90);
}

void get_ps2_input() {
  ps2x.read_gamepad();
  if ((ps2x.Analog(1) & 0xf0) != 0x70) {
      return;
  }
  Serial.print("PS2 Input: ");
  Serial.print(ps2x.ButtonDataByte(), HEX);
  Serial.print(":");
  Serial.print(ps2x.Analog(PSS_LX), DEC);
  Serial.print(" ");
  Serial.print(ps2x.Analog(PSS_LY), DEC);
  Serial.print(" ");
  Serial.print(ps2x.Analog(PSS_RX), DEC);
  Serial.print(" ");
  Serial.println(ps2x.Analog(PSS_RY), DEC);
  if (ps2x.ButtonPressed(PSB_START)) {
    Serial.print(" ");
    Serial.println("PSB_START");
  } 
  else if (ps2x.ButtonPressed(PSB_L1)) {// L1 Button Test
    Serial.print(" ");
    Serial.println("PSB_L1");
    rotate_servo(lf_femur);
  }
  else if (ps2x.ButtonPressed(PSB_L2)) {    // L2 Button Test
    Serial.print(" ");
    Serial.println("PSB_L2");
    rotate_servo(lf_tibia);
  }
  else if (ps2x.ButtonPressed(PSB_CIRCLE)) {
    Serial.print(" ");
    Serial.println("PSB_CIRCLE");
    rotate_servo(lm_tibia);
  }
  else if (ps2x.ButtonPressed(PSB_CROSS)) { // CROSS Button Test
    Serial.print(" ");
    Serial.println("PSB_CROSS");
    rotate_servo(lm_femur);
  }
  else if (ps2x.ButtonPressed(PSB_SQUARE)) { // Square Button Test
    Serial.print(" ");
    Serial.println("PSB_SQUARE");
    rotate_servo(lm_coxa);
  }
  else if (ps2x.ButtonPressed(PSB_TRIANGLE)) { // Triangle - Button Test
    Serial.print(" ");
    Serial.println("PSB_TRIANGLE");
    rotate_servo(lr_coxa);
  }
  else if (ps2x.ButtonPressed(PSB_PAD_UP)) {// D-Up - Button Test
    Serial.print(" ");
    Serial.println("PSB_PAD_UP");
    rotate_servo(lr_femur);
  }
  else if (ps2x.ButtonPressed(PSB_PAD_DOWN)) {// D-Down - Button Test
    Serial.print(" ");
    Serial.println("PSB_PAD_DOWN");
    rotate_servo(lr_tibia);
  }
  else if (ps2x.ButtonPressed(PSB_PAD_RIGHT)) { // D-Right - Button Test
    Serial.print(" ");
    Serial.println("PSB_PAD_RIGHT");
    rotate_servo(rf_tibia);
  }
  else if (ps2x.ButtonPressed(PSB_PAD_LEFT)) { // D-Left - Button Test
    Serial.print(" ");
    Serial.println("PSB_PAD_LEFT");
    rotate_servo(rf_femur);
  }
  else if (ps2x.ButtonPressed(PSB_SELECT)) {           // Select Button Test
    Serial.print(" ");
    Serial.println("PSB_SELECT");
    rotate_servo(rf_coxa);
  }
  else if (ps2x.ButtonPressed(PSB_R1)) { // R1 Button Test
    Serial.print(" ");
    Serial.println("PSB_R1");
    rotate_servo(rr_coxa);
  }
  else if (ps2x.ButtonPressed(PSB_R2)) {// R2 Button Test
    Serial.print(" ");
    Serial.println("PSB_R2");
    rotate_servo(rr_femur);
  }
  else if (ps2x.ButtonPressed(PSB_R3)) {// R3 Button Test
    Serial.print(" ");
    Serial.println("PSB_R3");
    rotate_servo(rr_tibia);
  }
}

void setup() { 
  init_servos();
  init_ps2();
  Serial.begin(9600);
}

void write_pos(int pos) {
  lf_coxa.write(pos);
  lf_femur.write(pos);
  lf_tibia.write(pos);
  lm_coxa.write(pos);
  lm_femur.write(pos);
  lm_tibia.write(pos);
  lr_coxa.write(pos);
  lr_femur.write(pos);
  lr_tibia.write(pos);

  rf_coxa.write(pos);
  rf_femur.write(pos);
  rf_tibia.write(pos);
  rm_coxa.write(pos);
  rm_femur.write(pos);
  rm_tibia.write(pos);
  rr_coxa.write(pos);
  rr_femur.write(pos);
  rr_tibia.write(pos);
}

void loop() { 
    get_ps2_input();
    /*
    int pos;
    for(pos = 0; pos < 180; pos++) {
        write_pos(pos);
        delay(15);
    }
    for(pos = 180; pos > 0; pos--) {
        write_pos(pos);
        delay(15);
    }
    */
    delay(300);
}

