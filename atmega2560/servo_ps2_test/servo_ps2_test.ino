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
  /*
#endif
#endif
// In an analog mode so should be OK...
g_sPS2ErrorCnt = 0;    // clear out error count...

if (ps2x.ButtonPressed(PSB_START)) {// OK lets try "0" button for Start. 
  if (g_InControlState.fHexOn) {
    PS2TurnRobotOff();
  } 
  else {
    //Turn on
    g_InControlState.fHexOn = 1;
    fAdjustLegPositions = true;
  }
}

if (g_InControlState.fHexOn) {
  // [SWITCH MODES]

  //Translate mode
  if (ps2x.ButtonPressed(PSB_L1)) {// L1 Button Test
    MSound( 1, 50, 2000);  
    if (ControlMode != TRANSLATEMODE )
      ControlMode = TRANSLATEMODE;
    else {
      if (g_InControlState.SelectedLeg==255) 
        ControlMode = WALKMODE;
      else
        ControlMode = SINGLELEGMODE;
    }
  }

  //Rotate mode
  if (ps2x.ButtonPressed(PSB_L2)) {    // L2 Button Test
    MSound( 1, 50, 2000);
    if (ControlMode != ROTATEMODE)
      ControlMode = ROTATEMODE;
    else {
      if (g_InControlState.SelectedLeg == 255) 
        ControlMode = WALKMODE;
      else
        ControlMode = SINGLELEGMODE;
    }
  }
#if LED_EYE_PWM_PIN!=0

  if (ps2x.ButtonPressed(PSB_CIRCLE)) 
  {
    EyeOn=!EyeOn;
    MSound(1, 50, 2000);  
    SetEyes();
  }
#endif
  
#ifdef OPT_SINGLELEG
  //Single leg mode fNO
  if (ps2x.ButtonPressed(PSB_CIRCLE)) {// O - Circle Button Test
    if (abs(g_InControlState.TravelLength.x)<cTravelDeadZone && abs(g_InControlState.TravelLength.z)<cTravelDeadZone 
      && abs(g_InControlState.TravelLength.y*2)<cTravelDeadZone )   {
      if (ControlMode != SINGLELEGMODE) {
        ControlMode = SINGLELEGMODE;
        if (g_InControlState.SelectedLeg == 255)  //Select leg if none is selected
          g_InControlState.SelectedLeg=cRF; //Startleg
      } 
      else {
        ControlMode = WALKMODE;
        g_InControlState.SelectedLeg=255;
      }
    }
  }      
#endif // OPT_SINGLELEG

#ifdef STABILISATOR
  //[Common functions]
  //Switch Balance mode on/off 
  if (ps2x.ButtonPressed(PSB_CROSS)) { // CROSS Button Test
    StalilisatorOn = !StalilisatorOn;
    if (StalilisatorOn) {
      MSound(1, 250, 1500); 
      ref_StabilisationX=  event.magnetic.x;
      ref_StabilisationY=  event.magnetic.y;
      ref_StabilisationZ=  event.magnetic.z;
    } 
    else {
      MSound( 2, 100, 2000, 50, 4000);
    }
  }

#endif// STABILISATOR


  //[Common functions]
  //Switch Balance mode on/off 
  if (ps2x.ButtonPressed(PSB_SQUARE)) { // Square Button Test
    g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
    if (g_InControlState.BalanceMode) {
      MSound(1, 250, 1500); 
    } 
    else {
      MSound( 2, 100, 2000, 50, 4000);
    }
  }

  //Stand up, sit down  
  if (ps2x.ButtonPressed(PSB_TRIANGLE)) { // Triangle - Button Test
    if (g_BodyYOffset==0) 
      g_BodyYOffset = CHexInitY/2;
    else if(g_BodyYOffset==CHexInitY/2)
      g_BodyYOffset = CHexInitY;
    else
      g_BodyYOffset = 0;
    fAdjustLegPositions = true;
  }

  if (ps2x.ButtonPressed(PSB_PAD_UP)) {// D-Up - Button Test
    g_BodyYOffset += D_PAD_STEP;

    // And see if the legs should adjust...
    fAdjustLegPositions = true;
    if (g_BodyYOffset > MAX_BODY_Y)
      g_BodyYOffset = MAX_BODY_Y;
  }

  if (ps2x.ButtonPressed(PSB_PAD_DOWN) && g_BodyYOffset) {// D-Down - Button Test
    if (g_BodyYOffset > D_PAD_STEP)
      g_BodyYOffset -= D_PAD_STEP;
    else
      g_BodyYOffset = 0;      // constrain don't go less than zero.

    // And see if the legs should adjust...
    fAdjustLegPositions = true;
  }

  if (ps2x.ButtonPressed(PSB_PAD_RIGHT)) { // D-Right - Button Test
    if (g_InControlState.SpeedControl>0) {
      g_InControlState.SpeedControl = g_InControlState.SpeedControl - 50;
      MSound( 1, 50, 2000);  
    }
  }

  if (ps2x.ButtonPressed(PSB_PAD_LEFT)) { // D-Left - Button Test
    if (g_InControlState.SpeedControl<2000 ) {
      g_InControlState.SpeedControl = g_InControlState.SpeedControl + 50;
      MSound( 1, 50, 2000); 
    }
  }

  //[Walk functions]
  if (ControlMode == WALKMODE) {
    //Switch gates
    if (ps2x.ButtonPressed(PSB_SELECT)            // Select Button Test
    && abs(g_InControlState.TravelLength.x)<cTravelDeadZone //No movement
    && abs(g_InControlState.TravelLength.z)<cTravelDeadZone 
      && abs(g_InControlState.TravelLength.y*2)<cTravelDeadZone  ) {
      g_InControlState.GaitType = g_InControlState.GaitType+1;                    // Go to the next gait...
      if (g_InControlState.GaitType<NUM_GAITS) {                 // Make sure we did not exceed number of gaits...
        MSound( 1, 50, 2000); 
      } 
      else {
        MSound(2, 50, 2000, 50, 2250); 
        g_InControlState.GaitType = 0;
      }
      GaitSelect();
    }

    //Double leg lift height
    if (ps2x.ButtonPressed(PSB_R1)) { // R1 Button Test
      MSound( 1, 50, 2000); 
      DoubleHeightOn = !DoubleHeightOn;
      if (DoubleHeightOn)
        g_InControlState.LegLiftHeight = 50;
      else
        g_InControlState.LegLiftHeight = 25;
    }

    //Double Travel Length
    if (ps2x.ButtonPressed(PSB_R2)) {// R2 Button Test
      MSound(1, 50, 2000); 
      DoubleTravelOn = !DoubleTravelOn;
    }

    // Switch between Walk method 1 && Walk method 2
    if (ps2x.ButtonPressed(PSB_R3)) { // R3 Button Test
      MSound(1, 50, 2000); 
      WalkMethod = !WalkMethod;
    }

    //Walking
    if (WalkMethod)  //(Walk Methode) 
      g_InControlState.TravelLength.z = (ps2x.Analog(PSS_RY)-128)/2; //Right Stick Up/Down  

    else {
      g_InControlState.TravelLength.x = -(ps2x.Analog(PSS_LX) - 128)*3/5;
      g_InControlState.TravelLength.z = (ps2x.Analog(PSS_LY) - 128)*3/5;
    }

    if (!DoubleTravelOn) {  //(Double travel length)
      g_InControlState.TravelLength.x = g_InControlState.TravelLength.x/2;
      g_InControlState.TravelLength.z = g_InControlState.TravelLength.z/2;
    }

    g_InControlState.TravelLength.y = -(ps2x.Analog(PSS_RX) - 128)/4; //Right Stick Left/Right 
  }

  //[Translate functions]
  g_BodyYShift = 0;
  if (ControlMode == TRANSLATEMODE) {
    g_InControlState.BodyPos.x = (ps2x.Analog(PSS_LX) - 128)/3;
    g_InControlState.BodyPos.z = -(ps2x.Analog(PSS_LY) - 128)/3;
    g_InControlState.BodyRot1.y = (ps2x.Analog(PSS_RX) - 128)*2;
    g_BodyYShift = (-(ps2x.Analog(PSS_RY) - 128)/2);
  }

  //[Rotate functions]
  if (ControlMode == ROTATEMODE) {
    g_InControlState.BodyRot1.x = (ps2x.Analog(PSS_LY) - 128);
    g_InControlState.BodyRot1.y = (ps2x.Analog(PSS_RX) - 128)*2;
    g_InControlState.BodyRot1.z = (ps2x.Analog(PSS_LX) - 128);
    g_BodyYShift = (-(ps2x.Analog(PSS_RY) - 128)/2);
  }

  //[Single leg functions]
  if (ControlMode == SINGLELEGMODE) {
    //Switch leg for single leg control
    if (ps2x.ButtonPressed(PSB_SELECT)) { // Select Button Test
      MSound(1, 50, 2000); 
      if (g_InControlState.SelectedLeg<5)
        g_InControlState.SelectedLeg = g_InControlState.SelectedLeg+1;
      else
        g_InControlState.SelectedLeg=0;
    }

    g_InControlState.SLLeg.x= (ps2x.Analog(PSS_LX) - 128)/2; //Left Stick Right/Left
    g_InControlState.SLLeg.y= (ps2x.Analog(PSS_RY) - 128)/10; //Right Stick Up/Down
    g_InControlState.SLLeg.z = (ps2x.Analog(PSS_LY) - 128)/2; //Left Stick Up/Down

    // Hold single leg in place
    if (ps2x.ButtonPressed(PSB_R2)) { // R2 Button Test
      MSound(1, 50, 2000);  
      g_InControlState.fSLHold = !g_InControlState.fSLHold;
    }
  }


#ifdef STABILISATOR
    if (StalilisatorOn) {
      float dx = ref_StabilisationX-event.magnetic.x;
      float dy = ref_StabilisationY-event.magnetic.y;
      float dz = ref_StabilisationZ-event.magnetic.z;
      if(dx>STABILISATOR_TRIGGERVALUE||dx<-STABILISATOR_TRIGGERVALUE)
        g_InControlState.BodyRot1.x -= dx;
      if(dz>STABILISATOR_TRIGGERVALUE||dz<-STABILISATOR_TRIGGERVALUE)
        g_InControlState.BodyRot1.y -= dz;
      if(dy>STABILISATOR_TRIGGERVALUE||dy<-STABILISATOR_TRIGGERVALUE)
        g_InControlState.BodyRot1.z -= dy;
      //g_InControlState.BodyRot1.z = (ps2x.Analog(PSS_LX) - 128);
      
      }
#endif // STABILISATOR
#ifdef OPT_GPPLAYER
  //[GPPlayer functions]
  if (ControlMode == GPPLAYERMODE) {

    // Lets try some speed control... Map all values if we have mapped some before
    // or start mapping if we exceed some minimum delta from center
    // Have to keep reminding myself that commander library already subtracted 128...
    if (g_ServoDriver.FIsGPSeqActive() ) {
      if ((g_sGPSMController != 32767)  
        || (ps2x.Analog(PSS_RY) > (128+16)) || (ps2x.Analog(PSS_RY) < (128-16)))
      {
        // We are in speed modify mode...
        short sNewGPSM = map(ps2x.Analog(PSS_RY), 0, 255, -200, 200);
        if (sNewGPSM != g_sGPSMController) {
          g_sGPSMController = sNewGPSM;
          g_ServoDriver.GPSetSpeedMultiplyer(g_sGPSMController);
        }

      }
    }

    //Switch between sequences
    if (ps2x.ButtonPressed(PSB_SELECT)) { // Select Button Test
      if (!g_ServoDriver.FIsGPSeqActive() ) {
        if (GPSeq < 5) {  //Max sequence
          MSound(1, 50, 1500);
          GPSeq = GPSeq+1;
        } 
        else {
          MSound(2, 50, 2000, 50, 2250);
          GPSeq=0;
        }
      }
    }
    //Start Sequence
    if (ps2x.ButtonPressed(PSB_R2))// R2 Button Test
      if (!g_ServoDriver.FIsGPSeqActive() ) {
      g_ServoDriver.GPStartSeq(GPSeq);
        g_sGPSMController = 32767;  // Say that we are not in Speed modify mode yet... valid ranges are 50-200 (both postive and negative... 
      }
      else {
        g_ServoDriver.GPStartSeq(0xff);    // tell the GP system to abort if possible...
        MSound (2, 50, 2000, 50, 2000);
      }


  }
#endif // OPT_GPPLAYER

  //Calculate walking time delay
  g_InControlState.InputTimeDelay = 128 - max(max(abs(ps2x.Analog(PSS_LX) - 128), abs(ps2x.Analog(PSS_LY) - 128)), abs(ps2x.Analog(PSS_RX) - 128));
}
*/
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
    delay(1000);
}

