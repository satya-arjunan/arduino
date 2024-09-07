#include <Servo.h> 
#include <Adafruit_HMC5883_U.h>
#include "config.h"
#include "phoenix.h"
#include "phoenix_input_ps2.h"

INCONTROLSTATE   g_InControlState;      // This is our global Input control state object...

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

void GaitSelect(void) {}
void MSound(byte cNotes, ...) {}
void AdjustLegPositionsToBodyHeight(void) {}

void setup() { 
  init_servos();
  g_InputController.Init();
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

void loop() 
{ 
    int pos;
    
    for(pos = 0; pos < 180; pos++) {
        write_pos(pos);
        delay(15);
    }
    for(pos = 180; pos > 0; pos--) {
        write_pos(pos);
        delay(15);
    }
}

