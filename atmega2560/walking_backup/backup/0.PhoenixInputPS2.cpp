/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 see <http://www.gnu.org/licenses/>
*/


#include <Arduino.h> // Arduino 1.0
#include <avr/pgmspace.h>
#include "utils.h"
#include "PhoenixInputPS2.h"

PhoenixInputPS2::PhoenixInputPS2(void)
{
    mSerial = &CONFIG_CTRL_SERIAL;
    mSerial->begin(CONFIG_CTRL_BAUD);
}

PhoenixInputPS2::PhoenixInputPS2(HardwareSerial *serial)
{
    mSerial = serial;
}

void PhoenixInputPS2::init(s8 (*callback)(u8 cmd, u8 *data, u8 size, u8 *res))
{
    printf(F("%s\n"), __PRETTY_FUNCTION__);
    error = mPS2.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT);
    mLX = 128;
    mLY = 128;
    mRX = 128;
    mRY = 128;
    mCallback = callback;
}

//==============================================================================
// This code reads inputs from a playstation 2 game controller (PS2) and
// then processes any commands.
//==============================================================================
u32 PhoenixInputPS2::get(u8 *lx, u8 *ly, u8 *rx, u8 *ry)
{
    int cmd;

    mPS2.read_gamepad();
    if ((ps2x.Analog(1) & 0xf0) != 0x70) {
        return 0;
    }

    cmd = mSerial->read();
    printf(F("input from serial monitor:%c\n"), cmd);

    // D-Up - Button Test
    if (ps2x.ButtonPressed(PSB_PAD_UP)) {
        //case 'x':
      return INPUT_BODY_UP;
    }
    // D-Down - Button Test
    if (ps2x.ButtonPressed(PSB_PAD_DOWN) && g_BodyYOffset) {
        //case 's':
      return INPUT_BODY_DOWN;
    }
    // D-Right - Button Test
    if (ps2x.ButtonPressed(PSB_PAD_RIGHT)) {
        //case 'a':
      return INPUT_SPEED_DOWN;
    }
    // D-Left - Button Test
    if (ps2x.ButtonPressed(PSB_PAD_LEFT)) {
        //case 'd':
      return INPUT_SPEED_UP;
    }
    if (ps2x.ButtonPressed(PSB_START)) {
        //case ' ':
      *lx = mLX = 128;
      *ly = mLY = 128;
      *rx = mRX = 128;
      *ry = mRY = 128;
      return INPUT_TOGGLE_ON_OFF; //on or off
    }
    // Select Button Test
    if (ps2x.ButtonPressed(PSB_SELECT) {
        //case 'z':
      return INPUT_OPT_SEL;
    }
    // L1 Button Test
    if (ps2x.ButtonPressed(PSB_L1)) {
        //case '1':
      return INPUT_TOGGLE_SHIFT; // translate
    }
    // L2 Button Test
    if (ps2x.ButtonPressed(PSB_L2)) {
        //case 'q':
      return INPUT_TOGGLE_ROTATE;
    }
    // R1 Button Test
    if (ps2x.ButtonPressed(PSB_R1)) {
        //case '3':
      return INPUT_OPT_R1;
    }
    // R2 Button Test
    if (ps2x.ButtonPressed(PSB_R2)) {
        //case 'e':
      return INPUT_OPT_R2;
    }
    // O - Circle Button Test
    if (ps2x.ButtonPressed(PSB_CIRCLE)) {
        //case ',':
      return INPUT_TOGGLE_SINGLE_LEG;
    }
    // CROSS Button Test
    if (ps2x.ButtonPressed(PSB_CROSS)) {
        //case '.':
      return INPUT_TOGGLE_BALANCE;
    }
    //Stand up, sit down  
    // Triangle - Button Test
    if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
        //case '/':
      return INPUT_TOGGLE_BODY_HEIGHT;
    }
    mLX = ps2x.Analog(PSS_LX);
    mLY = ps2x.Analog(PSS_RY);
    mRX = ps2x.Analog(PSS_LY);
    mRY = ps2x.Analog(PSS_RY);
    *lx = mLX;
    *ly = mLY;
    *rx = mRX;
    *ry = mRY;
    return 0;
}

u8 PhoenixInputPS2::getBodyHeight(void)
{
    return 0;
}

