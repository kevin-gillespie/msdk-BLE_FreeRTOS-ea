/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*
******************************************************************************/
#ifndef __HM0360_REGS_H__
#define __HM0360_REGS_H__

#define HM0360_I2C_SLAVE_ADDR   0x24

#define MODEL_H        (0x0000)
#define MODEL_L        (0x0001)
#define REVISION       (0x0002)

#define FRAME_COUNT_H  (0x0005)
#define FRAME_COUNT_L  (0x0006)
#define PIXEL_ORDER    (0x0007)

#define SW_RESET       (0x0103)
#define COMMAND_UPDATE (0x0104)

#define MONO_CTRL       (0x100A)

#define MODE_SELECT             (0x0100)
#define STANDBY_MODE            0x0
#define SW_STREAMING_MODE       0x1
#define SW_AUTO_WAKEUP_MODE     0x2
#define SW_SNAPSHOT_NFRAME_MODE 0x3
#define HW_STREAMING_MODE       0x4
#define HW_SNAPSHOT_NFRAME_MODE 0x6
#define HW_AUTO_WAKEUP_MODE     0x7

#define IMAGE_ORIENTATION   (0x0101)
#define H_MIRROR            0x1
#define V_FLIP              0x2

#define FRAME_LENGTH_H    (0x0340)  //default 532
#define FRAME_LENGTH_L    (0x0341)
#define LINE_LENGTH_H     (0x0342)  //default 768
#define LINE_LENGTH_L     (0x0343)

#define VGA_WINDOW         (0x3030)
#define VGA_RES_ENABLE     0x1

#define TEST_PATTERN_MODE  (0x0601)
#define COLOR_BAR          0x01
#define FADE_COLOR_BAR     0x11
#define WALKING_ONES       0x21
#define SOLID_PATTERN      0x31
#define PN9                0x41

#endif //__HM0360_H__
