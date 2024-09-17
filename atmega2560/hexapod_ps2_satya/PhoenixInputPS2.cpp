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
    //added delay to give wireless ps2 module some time to startup
    //before configuring it
    delay(300);
    int error = mPS2.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT);
    if (error == 0) {
      printf(F("Found ps2 controller\n"));
    }
    else if (error == 1) {
      printf(F("No ps2 controller found\n"));
    }
    else if (error == 2) {
      printf(F("Controller found but not accepting commands\n"));
    }
      
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
    delay(50);  
    //int cmd;

    mPS2.read_gamepad();
    if ((mPS2.Analog(1) & 0xf0) != 0x70) {
        return 0;
    }
    /*
    printf("PS2 Input: ");
    printf(F("data byte:%x\n"), mPS2.ButtonDataByte());
    printf(":");
    printf(F("lx:%d\n"), mPS2.Analog(PSS_LX));
    printf(" ");
    printf(F("ly:%d\n"), mPS2.Analog(PSS_LY));
    printf(" ");
    printf(F("rx:%d\n"), mPS2.Analog(PSS_RX));
    printf(" ");
    printf(F("ry:%d\n"), mPS2.Analog(PSS_RY));
    */

    //cmd = mSerial->read();
    //printf(F("input from serial monitor:%c\n"), cmd);
    
    /*
    mLX = mPS2.Analog(PSS_LX);
    mLY = mPS2.Analog(PSS_LY);
    mRY = mPS2.Analog(PSS_RY);
    mRX = mPS2.Analog(PSS_RX);
    */
    *lx = mLX;
    *ly = mLY;
    *rx = mRX;
    *ry = mRY;
    if (mPS2.NewButtonState()) {

    if (mPS2.Button(PSB_START)) {
      *lx = mLX = 128;
      *ly = mLY = 128;
      *rx = mRX = 128;
      *ry = mRY = 128;
      printf(F("start/stop\n"));
      return INPUT_TOGGLE_ON_OFF; //on or off
    }
    // D-Up - Button Test
    if (mPS2.Button(PSB_PAD_UP)) {
      printf(F("body up\n"));
      return INPUT_BODY_UP;
    }
    // D-Down - Button Test
    if (mPS2.Button(PSB_PAD_DOWN)) {
      printf(F("body down\n"));
      return INPUT_BODY_DOWN;
    }
    // D-Right - Button Test
    if (mPS2.Button(PSB_PAD_RIGHT)) {
      printf(F("speed down\n"));
      return INPUT_SPEED_DOWN;
    }
    // D-Left - Button Test
    if (mPS2.Button(PSB_PAD_LEFT)) {
      printf(F("speed up\n"));
      return INPUT_SPEED_UP;
    }
    // Select Button Test
    if (mPS2.Button(PSB_SELECT)) {
      printf(F("opt_sel\n"));
      return INPUT_OPT_SEL;
    }
    // L1 Button Test
    if (mPS2.Button(PSB_L1)) {
      printf(F("translate\n"));
      return INPUT_TOGGLE_SHIFT; // translate
    }
    // L2 Button Test
    if (mPS2.Button(PSB_L2)) {
      printf(F("rotate\n"));
      return INPUT_TOGGLE_ROTATE;
    }
    // R1 Button Test
    if (mPS2.Button(PSB_R1)) {
      printf(F("R1\n"));
      return INPUT_OPT_R1;
    }
    // R2 Button Test
    if (mPS2.Button(PSB_R2)) {
      printf(F("R2\n"));
      return INPUT_OPT_R2;
    }
    // O - Circle Button Test
    if (mPS2.Button(PSB_CIRCLE)) {
        //case ',':
      printf(F("single leg\n"));
      return PSB_CIRCLE;
    }
    if (mPS2.Button(PSB_SQUARE)) {
        //case ',':
      printf(F("square leg\n"));
      return PSB_SQUARE;
    }
    // CROSS Button Test
    if (mPS2.Button(PSB_CROSS)) {
        //case '.':
      printf(F("balance\n"));
      return INPUT_TOGGLE_BALANCE;
    }
    //Stand up, sit down  
    // Triangle - Button Test
    if (mPS2.Button(PSB_TRIANGLE)) {
        //case '/':
      printf(F("height\n"));
      return INPUT_TOGGLE_BODY_HEIGHT;
    }
    }
    return 0;
}

u8 PhoenixInputPS2::getBodyHeight(void)
{
    return 0;
}

