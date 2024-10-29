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

void turnOff(void)
{
    mBodyYOffset = 0;
    mBodyYShift  = 0;
    core->initCtrl();
    Utils::sound(400, 0, 0, 100, 300);
}


void loop() {
  u32  dwButton;
  u8   lx, ly, rx, ry;
  dwButton = input->get(&lx, &ly, &rx, &ry);
  if (!dwButton) {
    goto loop_exit;
  }
  if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_ON_OFF)) {
    if (ctrlState.fHexOn) {
      ctrlState.fHexOn = FALSE;
      printf(F("OFF\n"));
    } else {
      ctrlState.fHexOn = TRUE;
      printf(F("ON\n"));
    }
  }
  if (!ctrlState.fHexOn) {
    goto loop_exit;
  }
loop_exit:
  //ret = core->loop();
  ctrlState.fHexOnOld = ctrlState.fHexOn;
}
