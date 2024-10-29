//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix software
//Software version: V2.0
//Date: 29-10-2009
//Programmer: Jeroen Janssen [aka Xan]
//         Kurt Eckhardt(KurtE) converted to C and Arduino
//   KÃ¥re Halvorsen aka Zenta - Makes everything work correctly!
//
// This version of the Phoenix code was ported over to the Arduino Environement
// and is specifically configured for the Lynxmotion BotBoarduino
//
// Date : 12-07-2015
// Programmer : PingguSoft (pinggusoft@gmail.com)
//              Code rework for smartphone control and readability
//              Android App : BTCon4Drone
//               https://play.google.com/store/apps/details?id=com.pinggusoft.btcon
//=============================================================================

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
    #define SERIAL_RX_BUFFER_SIZE 256
    #define SERIAL_TX_BUFFER_SIZE 256
#endif

#define DEFINE_HEX_GLOBALS
#if ARDUINO > 99
#include <Arduino.h>
#endif
#include <Wire.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

#include "common.h"
#include "ServoEx.h"
#include "config.h"
#include "PhoenixCore.h"
#include "PhoenixInput.h"
#include "PhoenixInputSerial.h"
#include "PhoenixInputPS2.h"
#include "PhoenixInputBTCon.h"
#include "PhoenixServo.h"
#include "PhoenixServoSW.h"
#include "PhoenixServoUSC.h"

enum {
    MODE_WALK = 0,
    MODE_TRANSLATE,
    MODE_ROTATE,
    MODE_SINGLE_LEG
};

PhoenixCore  *core;
PhoenixInput *input;
CTRL_STATE   ctrlState;

u8        mColor;
s16       mBodyYOffset;
u16       mErrorCnt;
s16       mBodyYShift;
u8        mModeControl;
bool      mBoolDoubleHeight;
bool      mBoolDblTravel;
bool      mBoolWalkMode2;



int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


#if (CONFIG_CTRL_TYPE == CONFIG_CTRL_TYPE_BTCON)
u8 scale = 30;
s8 inputCallback(u8 cmd, u8 *data, u8 size, u8 *res)
{
    s8 ret = -1;

    switch (cmd) {
        case PhoenixInputBTCon::MSP_ANALOG:
            if (core) {
                u8 *ptr = (u8*)res;

                *ptr = core->getBattLevel(scale);
                *(ptr + 3) = core->getBattLevel();
                ret = 7;
            }
            break;

        case PhoenixInputBTCon::MSP_SET_MISC:
            scale = *(data + 18);
            ret = 0;
            break;
    }
    return ret;
}
#endif

enum {
    COLOR_BLACK  = 0,
    COLOR_RED    = 1,
    COLOR_GREEN  = 2,
    COLOR_YELLOW = 3,
    COLOR_BLUE   = 4,
    COLOR_PURPLE = 5,
    COLOR_CYAN   = 6,
    COLOR_WHITE  = 7
};

void showLED(u8 color)
{
    //printf(F("COLOR : %d\n"), color);
    digitalWrite(PIN_STATUS_RED,   color & 0x01);
    digitalWrite(PIN_STATUS_GREEN, color & 0x02);
    digitalWrite(PIN_STATUS_BLUE,  color & 0x04);
}

void setup()
{
    pinMode(PIN_STATUS_RED,   OUTPUT);
    pinMode(PIN_STATUS_GREEN, OUTPUT);
    pinMode(PIN_STATUS_BLUE,  OUTPUT);
    pinMode(PIN_SOUND, OUTPUT);

    mColor       = 0;
    mBodyYOffset = 0;
    mBodyYShift  = 0;
    mErrorCnt    = 0;

    mModeControl      = MODE_WALK;
    mBoolDoubleHeight = FALSE;
    mBoolDblTravel    = FALSE;
    mBoolWalkMode2    = FALSE;

#ifdef CONFIG_DBG_SERIAL
    CONFIG_DBG_SERIAL.begin(CONFIG_DEBUG_BAUD);
#endif


#if (CONFIG_CTRL_TYPE == CONFIG_CTRL_TYPE_SERIAL)
    input = new PhoenixInputSerial();
	input->init(NULL);

#elif (CONFIG_CTRL_TYPE == CONFIG_CTRL_TYPE_PS2)
    CONFIG_CTRL_SERIAL.begin(CONFIG_CTRL_BAUD);
    input = new PhoenixInputPS2(&CONFIG_CTRL_SERIAL);
	input->init(NULL);

#elif (CONFIG_CTRL_TYPE == CONFIG_CTRL_TYPE_BTCON)
    CONFIG_CTRL_SERIAL.begin(CONFIG_CTRL_BAUD);
    input = new PhoenixInputBTCon(&CONFIG_CTRL_SERIAL);
	input->init(inputCallback);

#else
    #error No Controller !!
#endif

    PhoenixServo    *servo;
#if (CONFIG_SERVO == CONFIG_SERVO_SW_PWM)
    servo = new PhoenixServoSW();

#elif (CONFIG_SERVO == CONFIG_SERVO_USC)
    #if (CONFIG_BOARD == CONFIG_NASSPOP_MINI) && defined(CONFIG_SERVO_USC_TX)
        servo = new PhoenixServoUSC();
    #else
        servo = new PhoenixServoUSC(&CONFIG_CTRL_SERIAL);
    #endif

#endif

    core = new PhoenixCore(servo, &ctrlState);
	core->init();
    printf(F("FREE RAM : %d\n"), freeRam());
}

#define BUTTON_PRESSED(stat, mask) (stat & (mask))

void turnOff(void) {
  mBodyYOffset = 0;
  mBodyYShift  = 0;
  core->initCtrl();
  ctrlState.fHexOnOld = FALSE;
}

void turnOn(void) {
}

void travel(u8 lx, u8 ly, u8 rx) {
  ctrlState.sLegLiftHeight = 80;
  ctrlState.c3dTravelLen.x = -(lx - 128);
  ctrlState.c3dTravelLen.z = (ly - 128);
  //ctrlState.c3dTravelLen.y = -(rx - 128)/4; //Right Stick Left/Right
}

/*
  translate
  rx & ry when less than 128 the height of pod goes up
  rx & ry when more than 128 the pod twists clockwise
  lx & ly when more than 128 the slants to front left corner
  lx & ly when less than 128 the slants to bottom right corner
  lx when less or more than 128 no changes
  ly when more than 128 slants to front left corner
  ly when less than 128 slants to bottom right corner
  ry when less than 128 back goes down front stays same level
  ry when more than 128 no changes
  rx when more than 128 body same level but twisting clockwise at same spot
  rx when less than 128 body same level but twisting anticlockwise at same spot
  */
void translate(u8 lx, u8 ly, u8 rx, u8 ry) {
  ctrlState.c3dBodyPos.x = (ly - 128)/2;
  ctrlState.c3dBodyPos.z = -(ly - 128)/3;
  ctrlState.c3dBodyRot.y = (rx - 128)*2;
  mBodyYShift = (-(ry - 128)/2);
  ctrlState.bInputTimeDelay = 128 - max( max(abs(lx - 128), abs(ly - 128)),
                                         abs(rx - 128));
  ctrlState.c3dBodyPos.y = min(max(mBodyYOffset + mBodyYShift,  0), MAX_BODY_Y);
}

void move_3_legs_updown() {
  core->adjustLegPosToBodyHeight();
  core->loop();
}

void move_up_then_none() {
  // the loop function below does the actual movement execution of servos or
  // shuts them all down (no power to servos)
  core->loop();
}

void print_ctrl_state() {
  printf(F("pos x:%d y:%d z:%d\n"),
         (int)ctrlState.c3dBodyPos.x,
         (int)ctrlState.c3dBodyPos.y,
         (int)ctrlState.c3dBodyPos.z);
  printf(F("rot x:%d y:%d z:%d\n"),
         (int)ctrlState.c3dBodyRot.x,
         (int)ctrlState.c3dBodyRot.y,
         (int)ctrlState.c3dBodyRot.z);
  printf(F("rotoff x:%d y:%d z:%d\n"),
         (int)ctrlState.c3dBodyRotOff.x,
         (int)ctrlState.c3dBodyRotOff.y,
         (int)ctrlState.c3dBodyRotOff.z);
  printf(F("len x:%d y:%d z:%d\n"),
         (int)ctrlState.c3dTravelLen.x,
         (int)ctrlState.c3dTravelLen.y,
         (int)ctrlState.c3dTravelLen.z);
  printf(F("gait:%d\n"), (unsigned)ctrlState.bGaitType);
  printf(F("lift height:%d\n"), (int)ctrlState.sLegLiftHeight);
  printf(F("gait leg init:%d %d %d %d %d %d\n"),
         (unsigned)core->mGaitLegInits[0],
         (unsigned)core->mGaitLegInits[1],
         (unsigned)core->mGaitLegInits[2],
         (unsigned)core->mGaitLegInits[3],
         (unsigned)core->mGaitLegInits[4],
         (unsigned)core->mGaitLegInits[5]);
}

void update_servos() {
  /*
  ctrlState.c3dBodyPos.x = 128;
  ctrlState.c3dBodyPos.z = 128;
  ctrlState.c3dBodyPos.y = 128;
  ctrlState.c3dTravelLen.x = 128;
  ctrlState.c3dTravelLen.y = 128;
  ctrlState.c3dTravelLen.z = 128;
  ctrlState.c3dBodyRot.x = 128;
  ctrlState.c3dBodyRot.y = 128;
  ctrlState.c3dBodyRot.z = 128;
  */
  //printf(F("before\n"));
  //print_ctrl_state();
  move_up_then_none();
  //move_3_legs_updown();
}

bool check_ps2(u8 &lx, u8 &ly, u8 &rx, u8 &ry) {
  u32  dwButton;
  dwButton = input->get(&lx, &ly, &rx, &ry);
  if (BUTTON_PRESSED(dwButton, PSB_START)) {
    printf(F("turn on\n"));
    core->init();
    ctrlState.fHexOn = TRUE;
    core->loop();
    return true;
  }
  if (BUTTON_PRESSED(dwButton, PSB_CROSS)) {
    printf(F("turn off\n"));
    ctrlState.fHexOn = FALSE;
    core->loop();
    return true;
  }
  return false;
}


void finalise_loop() {
  ctrlState.c3dBodyPos.y = min(max(mBodyYOffset + mBodyYShift,  0),
                               MAX_BODY_Y);
  ctrlState.bGaitType = 5;
  core->adjustLegPosToBodyHeight();
  core->loop();
}

void move_leg(u8 x, u8 y, u8 z) {
  u8   lx, ly, rx, ry;
  if (check_ps2(lx, ly, rx, ry) == true) {
    return;
  }
  ctrlState.c3dSingleLeg.x = (x - 128) / 2;     // Left Stick Right/Left
  ctrlState.c3dSingleLeg.z = (z - 128) / 2;     // Left Stick Up/Down
  ctrlState.c3dSingleLeg.y = (y - 128) / 2;    // Right Stick Up/Down
  //ctrlState.bInputTimeDelay = 128 - max( max(abs(x - 128), abs(y - 128)),
  //                                       abs(z - 128));
  ctrlState.bInputTimeDelay = 0;
  finalise_loop();
}

void move_leg_rf(u8 x, u8 y, u8 z) {
  ctrlState.bSingleLegCurSel = 4;
  move_leg(x, y, z);
}

/*
void loop() {
  move_leg_rf(128, 128, 80);
  move_leg_rf(128, 128, 128);
  delay(2000);
  move_leg_rf(128, 128, 178);
  delay(2000);
}
*/

/*
// single leg ps2 control
void loop() {
  u8   lx, ly, rx, ry;
  if (check_ps2(lx, ly, rx, ry) == true) {
    return;
  }
  if (mModeControl == MODE_WALK) {
    ctrlState.bSingleLegCurSel = 1;
    ctrlState.c3dSingleLeg.x = (lx - 128) / 2;     // Left Stick Right/Left
    ctrlState.c3dSingleLeg.z = (ly - 128) / 2;     // Left Stick Up/Down
    ctrlState.c3dSingleLeg.y = (ry - 128) / 2;    // Right Stick Up/Down
    ctrlState.bInputTimeDelay = 128 - max( max(abs(lx - 128), abs(ly - 128)),
                                           abs(rx - 128));
    ctrlState.c3dBodyPos.y = min(max(mBodyYOffset + mBodyYShift,  0),
                                 MAX_BODY_Y);
    ctrlState.bGaitType = 5;
    core->adjustLegPosToBodyHeight();
    core->loop();
    return;
  }
}
*/

// working walk
void loop() {
  u32  dwButton;
  u8   lx, ly, rx, ry;
  dwButton = input->get(&lx, &ly, &rx, &ry);
  if (BUTTON_PRESSED(dwButton, PSB_START)) {
    //printf(F("turn on\n"));
    core->init();
    ctrlState.fHexOn = TRUE;
    core->loop();
    return;
  }
  if (BUTTON_PRESSED(dwButton, PSB_CROSS)) {
    //printf(F("turn off\n"));
    ctrlState.fHexOn = FALSE;
    core->loop();
    return;
  }
  if (BUTTON_PRESSED(dwButton, PSB_SQUARE)) {
      mModeControl = MODE_WALK;
  }
  if (mModeControl == MODE_WALK) {
      //printf(F("lx:%d ly:%d rx:%d\n"), (int)lx, (int)ly, (int)rx);
      ctrlState.sLegLiftHeight = 80;
      ctrlState.c3dTravelLen.x = -(lx - 128);
      ctrlState.c3dTravelLen.z = (ly - 128);
      ctrlState.c3dTravelLen.y = -(rx - 128)/4;
      ctrlState.c3dTravelLen.x = ctrlState.c3dTravelLen.x / 2;
      ctrlState.c3dTravelLen.z = ctrlState.c3dTravelLen.z / 2;
      //ctrlState.bInputTimeDelay = 50;
      ctrlState.bInputTimeDelay = 128 - max( max(abs(lx - 128), abs(ly - 128)),
                                             abs(rx - 128));
      ctrlState.c3dBodyPos.y = min(max(mBodyYOffset + mBodyYShift,  0),
                                  MAX_BODY_Y);
      ctrlState.bGaitType = 5;
      core->adjustLegPosToBodyHeight();
      core->loop();
      return;
  }
}

/*
  if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_SHIFT)) {
    if (mModeControl != MODE_TRANSLATE ) {
      mModeControl = MODE_TRANSLATE;
    }
    else {
      mModeControl = MODE_WALK;
    }
    printf(F("Toggle shift mode:%d\n"), mModeControl);
  }
  if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_ROTATE)) {
    if (mModeControl != MODE_ROTATE) {
      mModeControl = MODE_ROTATE;
    }
    else {
      mModeControl = MODE_WALK;
    }
    printf(F("Toggle rotate mode:%d\n"), mModeControl);
  }
  if (mModeControl == MODE_WALK) {
    if (BUTTON_PRESSED(dwButton, INPUT_OPT_SEL)) {
      if (abs(ctrlState.c3dTravelLen.x) < CONFIG_TRAVEL_DEAD_ZONE &&
          abs(ctrlState.c3dTravelLen.z) < CONFIG_TRAVEL_DEAD_ZONE &&
          abs(ctrlState.c3dTravelLen.y*2) < CONFIG_TRAVEL_DEAD_ZONE) {
        ctrlState.bGaitType = ctrlState.bGaitType + 1;
        if (ctrlState.bGaitType < NUM_GAITS) {
          Utils::sound(300, 0, 0, 50, 300);
        }
        else {
          Utils::sound(100, 100, 0, 50, 300);
          ctrlState.bGaitType = 0;
        }
        core->selectGait(ctrlState.bGaitType);
        printf(F("GAIT:%d\n"), ctrlState.bGaitType);
      } else {
        printf(F("GAIT can not be changed:%d\n"), ctrlState.bGaitType);
      }
    }
    // Double leg lift height
    if (BUTTON_PRESSED(dwButton, INPUT_OPT_R1)) {
      mBoolDoubleHeight = !mBoolDoubleHeight;
      if (mBoolDoubleHeight) {
        ctrlState.sLegLiftHeight = 80;
      }
      else {
        ctrlState.sLegLiftHeight = 50;
      }
      // Double Travel Length
      if (BUTTON_PRESSED(dwButton, INPUT_OPT_R2)) {
        mBoolDblTravel = !mBoolDblTravel;
      }
      if (mBoolWalkMode2) {
        ctrlState.c3dTravelLen.z = (ry - 128); //Right Stick Up/Down
      }
      else {
        ctrlState.c3dTravelLen.x = -(lx - 128);
        ctrlState.c3dTravelLen.z = (ly - 128);
      }
      if (!mBoolDblTravel) {
        ctrlState.c3dTravelLen.x = ctrlState.c3dTravelLen.x / 2;
        ctrlState.c3dTravelLen.z = ctrlState.c3dTravelLen.z / 2;
      }
      ctrlState.c3dTravelLen.y = -(rx - 128)/4; //Right Stick Left/Right
    }
  }
  */

/*
int rr(128);
void loop() {
  u32  dwButton;
  u8   lx, ly, rx, ry;
  dwButton = input->get(&lx, &ly, &rx, &ry);
  bool fAdjustLegPositions = FALSE;

  if (!dwButton) {
    return;
  }
  if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_ON_OFF)) {
    if (ctrlState.fHexOn) {
      ctrlState.fHexOn = FALSE;
      turnOff();
      printf(F("OFF\n"));
    } else {
      ctrlState.fHexOn = TRUE;
      turnOn();
      printf(F("ON\n"));
    }
  }
  else if (BUTTON_PRESSED(dwButton, PSB_CIRCLE)) {
    translate(128, 128, --rr, 128);
    printf(F("psb circle rr:%d\n"), rr);
  }
  else if (BUTTON_PRESSED(dwButton, PSB_SQUARE)) {
    travel(170, 170, 128);
    ctrlState.bGaitType = 5;
    printf(F("psb square rr:%d\n"), rr);
  }
  else {
    return;
  }
  update_servos();
}
*/
