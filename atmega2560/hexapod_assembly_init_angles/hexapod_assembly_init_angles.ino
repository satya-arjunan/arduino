#include <Servo.h> 

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


void setup() { 
	lf_coxa.attach(PIN_LF_COXA, 500, 2500);
  lf_femur.attach(PIN_LF_FEMUR, 500, 2500);
  lf_tibia.attach(PIN_LF_TIBIA, 500, 2500);
  lm_coxa.attach(PIN_LM_COXA, 500, 2500);
  lm_femur.attach(PIN_LM_FEMUR, 500, 2500);
  lm_tibia.attach(PIN_LM_TIBIA, 500, 2500);
  lr_coxa.attach(PIN_LR_COXA, 500, 2500);
  lr_femur.attach(PIN_LR_FEMUR, 500, 2500);
  lr_tibia.attach(PIN_LR_TIBIA, 500, 2500);

  rf_coxa.attach(PIN_RF_COXA, 500, 2500);
  rf_femur.attach(PIN_RF_FEMUR, 500, 2500);
  rf_tibia.attach(PIN_RF_TIBIA, 500, 2500);
  rm_coxa.attach(PIN_RM_COXA, 500, 2500);
  rm_femur.attach(PIN_RM_FEMUR, 500, 2500);
  rm_tibia.attach(PIN_RM_TIBIA, 500, 2500);
  rr_coxa.attach(PIN_RR_COXA, 500, 2500);
  rr_femur.attach(PIN_RR_FEMUR, 500, 2500);
  rr_tibia.attach(PIN_RR_TIBIA, 500, 2500);
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

void wipe() {
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
void loop() 
{ 
  write_pos(90);
  //wipe();
}

