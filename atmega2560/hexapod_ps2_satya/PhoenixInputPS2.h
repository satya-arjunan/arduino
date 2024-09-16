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

#ifndef _PHOENIX_INPUT_PS2_H_
#define _PHOENIX_INPUT_PS2_H_

#include "config.h"
#include "utils.h"
#include "PhoenixInput.h"
#include <PS2X_lib.h>

#define PS2_DAT      A3
#define PS2_CMD      A2
#define PS2_SEL      A1
#define PS2_CLK      A0

class PhoenixInputPS2 : public PhoenixInput
{
private:
    // stick
    u8        mLX;
    u8        mLY;
    u8        mRX;
    u8        mRY;
    PS2X      mPS2;
    HardwareSerial  *mSerial;
public:

    PhoenixInputPS2(void);
    PhoenixInputPS2(HardwareSerial *serial);

    virtual void init(s8 (*callback)(u8 cmd, u8 *data, u8 size, u8 *res));
    virtual u32  get(u8 *lx, u8 *ly, u8 *rx, u8 *ry);
    virtual u8   getBodyHeight(void);
};

#endif
