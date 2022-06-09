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

/**
 * @file    main.c
 * @brief   SPIX example using the MX25.
 * @details Uses the MX25 on the EvKit to show the SPIX. Erases, writes, and then
 *          verifies the data. MX25_BAUD, MX25_ADDR, and MX25_SPIXFC_WIDTH
 *          can be changed to alter the communication between the devices. Refer
 *          to the schematic for the pinout and ensure that there are no switches
 *          blocking the communication to the MX25.
 */

/***** Includes *****/
#include <stdio.h>
#include <string.h>
#include "mxc_sys.h"
#include "mxc_errors.h"
#include "max32665.h"
#include "mx25.h"
#include "spixf.h"
#include "board.h"
#include "led.h"

/***** Definitions *****/

#define MX25_ADDR               0
#define MX25_SPIXFC_WIDTH		MXC_SPIXF_WIDTH_4
#define MX25_EXP_ID 			0x00C22537

int fail = 0;

/***** Functions *****/

// These are set in the linkerfile and give the starting and ending address of xip_section
#if defined ( __GNUC__)
extern uint8_t __load_start_xip, __load_length_xip;
#endif

#if defined ( __CC_ARM )
// Note: This demo has not been tested under IAR and should be considered non-functional
extern int Image$$RW_IRAM2$$Length;
extern char Image$$RW_IRAM2$$Base[];
uint8_t * __xip_addr;
#endif

/******************************************************************************/
void spixf_cfg_setup () {

    // Disable the SPIXFC before setting the SPIXF
    MXC_SPIXF_Disable();
    MXC_SPIXF_SetSPIFrequency(MX25_BAUD);
    MXC_SPIXF_SetMode(MXC_SPIXF_MODE_0);
    MXC_SPIXF_SetSSPolActiveLow();
    MXC_SPIXF_SetSSActiveTime(MXC_SPIXF_SYS_CLOCKS_2);
    MXC_SPIXF_SetSSInactiveTime(MXC_SPIXF_SYS_CLOCKS_3);

    if(MX25_SPIXFC_WIDTH == MXC_SPIXF_WIDTH_1)
    {
        MXC_SPIXF_SetCmdValue(MX25_CMD_READ);
        MXC_SPIXF_SetCmdWidth(MXC_SPIXF_SINGLE_SDIO);
        MXC_SPIXF_SetAddrWidth(MXC_SPIXF_SINGLE_SDIO);
        MXC_SPIXF_SetDataWidth(MXC_SPIXF_WIDTH_1);
        MXC_SPIXF_SetModeClk(MX25_Read_DUMMY);
    }
    else
    {
        MXC_SPIXF_SetCmdValue(MX25_CMD_QREAD);
        MXC_SPIXF_SetCmdWidth(MXC_SPIXF_SINGLE_SDIO);
        MXC_SPIXF_SetAddrWidth(MXC_SPIXF_QUAD_SDIO);
        MXC_SPIXF_SetDataWidth(MXC_SPIXF_WIDTH_4);
        MXC_SPIXF_SetModeClk(MX25_QREAD_DUMMY);
    }
    

    MXC_SPIXF_Set3ByteAddr();
    MXC_SPIXF_SCKFeedbackEnable();
    MXC_SPIXF_SetSCKNonInverted();
}

/******************************************************************************/
int main(void)
{
    uint32_t id;
    void(*func)(void);
    uint8_t rx_buf[(uint32_t)(&__load_length_xip)];
    int rx_len = sizeof(rx_buf);

    printf("\n\n********************* SPIX Example *********************\n");
    printf("This example communicates with an MX25 flash on the EvKit\n");
    printf("loads code onto it and then executes that code using the \n");
    printf("SPIX execute-in-place peripheral\n\n");

    printf("SPI Clock: %d Hz\n\n", MX25_BAUD);

    // Initialize the SPIXFC registers and set the appropriate output pins
    if (MX25_Init() != E_NO_ERROR) {
        printf("Board Init Failed\n");
        printf("Example Failed\n");
        while(1);
    }
    printf("MX25 Initialized.\n\n");

    MX25_Reset();

    // Get the ID of the MX25
    if ((id = MX25_ID()) == MX25_EXP_ID) {
	   printf("MX25 ID verified\n\n");
    } else {
	   printf("Error verifying MX25 ID: 0x%x\n", id);
       printf("Example Failed\n");
	   while (1);
    }

    int err;

    // Erase Test Sector
    printf("Erasing first 64k sector\n");
    MX25_Erase(0x00000, MX25_Erase_64K);
    printf("Erased\n\n");

    // Enable Quad mode if we are using quad
    if (MX25_SPIXFC_WIDTH == MXC_SPIXF_WIDTH_4) {
    	if(MX25_Quad(1) != E_NO_ERROR) {
	       printf("Error enabling quad mode\n\n");
            fail++;
    	}
        else{
            printf("Quad mode enabled\n\n");
        }
    } else {
        if(MX25_Quad(0) != E_NO_ERROR) {
            printf("Error disabling quad mode\n\n");
            fail++;
        }
        else{
            printf("Quad mode disabled\n\n");
        }
    }

    // Program the MX25
    printf("Programming function (%d bytes @ 0x%08x) into external MX25 flash\n", (uint32_t)(&__load_length_xip), &__load_start_xip);
    if((err = MX25_Program_Page(MX25_ADDR, &__load_start_xip, (uint32_t)(&__load_length_xip), MX25_SPIXFC_WIDTH)) != E_NO_ERROR) {
        printf("Error Programming: %d\n", err);
        fail++;
    } else {
        printf("Programmed\n\n");
    }

    printf("Verifying external flash\n");
    if((err = MX25_Read(MX25_ADDR, rx_buf, rx_len, MX25_SPIXFC_WIDTH)) != E_NO_ERROR) {
        printf("Error verifying data %d\n", err);
        fail++;
    } else {
        if(memcmp(rx_buf, &__load_start_xip, rx_len) != E_NO_ERROR) {
            printf("Error invalid data\n");
            fail++;
        } else {
            printf("Verified\n\n");
        }
    }

    // Setup SPIX
    spixf_cfg_setup();


    printf("Jumping to external flash (@ 0x%08x), watch for blinking LED.\n\n", (MXC_XIP_MEM_BASE | 0x1));
    func = (void(*)(void))(MXC_XIP_MEM_BASE | 0x1);
    func();
    printf("Returned from external flash\n\n");

    if(fail == 0) {
        printf("Example Succeeded\n\n");
    } else {
        printf("Example Failed\n\n");
    }
    return 0;
}
