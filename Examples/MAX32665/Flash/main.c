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
 * @brief   Flash Control Mass Erase & Write 32-bit enabled mode Example
 * @details This example shows how to mass erase the flash using the library
 *          and also how to Write and Verify 4 Words to the flash.
 */

/***** Includes *****/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "nvic_table.h"
#include "flc.h"
#include "icc.h"
#include "flc_regs.h"
#include "gcr_regs.h"

/***** Definitions *****/
#define TESTSIZE        8192        //2 pages worth so we can do erase functions 

#define MXC_FLASH_MEM_SIZE_TEST 2*MXC_FLASH_MEM_SIZE

/***** Globals *****/
uint32_t testaddr;
uint32_t testdata[TESTSIZE];
volatile uint32_t isr_cnt;
volatile uint32_t isr_flags;

/***** Functions *****/
int flash_verify(uint32_t address, uint32_t length, uint8_t* data)
{
    volatile uint8_t* ptr;
    
    
    for (ptr = (uint8_t*) address; ptr < (uint8_t*)(address + length); ptr++, data++) {
        if (*ptr != *data) {
            printf("Verify failed at 0x%x (0x%x != 0x%x)\n", (unsigned int) ptr, (unsigned int) *ptr, (unsigned int) *data);
            return E_UNKNOWN;
        }
    }
    
    return E_NO_ERROR;
}

//******************************************************************************
int check_mem(uint32_t startaddr, uint32_t length, uint32_t data)
{
    uint32_t* ptr;
    
    for (ptr = (uint32_t*) startaddr; ptr < (uint32_t*)(startaddr + length); ptr++) {
    
        if (*ptr != data) {
            return 0;
        }
    }
    
    return 1;
}

//******************************************************************************
int check_erased(uint32_t startaddr, uint32_t length)
{
    return check_mem(startaddr, length, 0xFFFFFFFF);
}

//******************************************************************************

int check_not_erased(uint32_t startaddr, uint32_t length)
{
    uint32_t* ptr;
    int erasedvaluefound = 0;
    
    for (ptr = (uint32_t*) startaddr; ptr < (uint32_t*)(startaddr + length); ptr++) {
        if (*ptr == 0xFFFFFFFF) {
            if (!erasedvaluefound) {
                erasedvaluefound = 1;
            }
            else {
                return 0;
            }
        }
    }
    
    return 1;
}

//******************************************************************************

void FLC0_IRQHandler(void)
{
    uint32_t temp;
    isr_cnt++;
    temp = MXC_FLC0->intr;
    
    if (temp & MXC_F_FLC_INTR_DONE) {
        MXC_FLC0->intr &= ~MXC_F_FLC_INTR_DONE;
    }
    
    if (temp & MXC_F_FLC_INTR_AF) {
        MXC_FLC0->intr &= ~MXC_F_FLC_INTR_AF;
    }
    
    isr_flags = temp;
}

void FLC1_IRQHandler(void)
{
    uint32_t temp;
    isr_cnt++;
    temp = MXC_FLC1->intr;

    if (temp & MXC_F_FLC_INTR_DONE) {
        MXC_FLC1->intr &= ~MXC_F_FLC_INTR_DONE;
    }

    if (temp & MXC_F_FLC_INTR_AF) {
        MXC_FLC1->intr &= ~MXC_F_FLC_INTR_AF;
    }

    isr_flags = temp;
}

//******************************************************************************
void flash_init(void)
{
    // Set flash clock divider to generate a 1MHz clock from the APB clock
    // APB clock is 54MHz on the real silicon
    MXC_FLC0->clkdiv = 24;
    MXC_FLC1->clkdiv = 24;
    
    // Setup and enable interrupt
    MXC_FLC_ClearFlags(0x3);
    NVIC_SetVector(FLC0_IRQn, FLC0_IRQHandler);
    NVIC_EnableIRQ(FLC0_IRQn);
    
	NVIC_SetVector(FLC1_IRQn, FLC1_IRQHandler);
	NVIC_EnableIRQ(FLC1_IRQn);
}

//******************************************************************************
void interrupt_enabler(mxc_flc_regs_t* regs)
{
    regs->intr = (MXC_F_FLC_INTR_DONEIE | MXC_F_FLC_INTR_AFIE);
    
}

//******************************************************************************
int flash_erase(uint32_t start, uint32_t end, uint32_t* buffer, unsigned length)
{
    int retval;
    uint32_t start_align, start_len, end_align, end_len, i;
    
    MXC_ASSERT(buffer);
    
    // Align start and end on page boundaries, calculate length of data to buffer
    start_align = start - (start % MXC_FLASH_PAGE_SIZE);
    start_len = (start % MXC_FLASH_PAGE_SIZE);
    end_align = end - (end % MXC_FLASH_PAGE_SIZE);
    end_len = MXC_FLASH_PAGE_SIZE - (end % MXC_FLASH_PAGE_SIZE);
    
    // Make sure the length of buffer is sufficient
    if ((length < start_len) || (length < end_len)) {
        return E_BAD_PARAM;
    }
    
    
    // Start and end address are in the same page
    if (start_align == end_align) {
        if (length < (start_len + end_len)) {
            return E_BAD_PARAM;
        }
        
        // Buffer first page data and last page data, erase and write
        memcpy(buffer, (void*) start_align, start_len);
        memcpy(&buffer[start_len], (void*) end, end_len);
        retval = MXC_FLC_PageErase(start_align);
        
        if (retval != E_NO_ERROR) {
            return retval;
        }
        
        retval = MXC_FLC_Write(start_align, start_len, buffer);
        
        if (retval != E_NO_ERROR) {
            return retval;
        }
        
        retval = MXC_FLC_Write(end, end_len, &buffer[start_len]);
        
        if (retval != E_NO_ERROR) {
            return retval;
        }
        
        return E_NO_ERROR;
    }
    
    // Buffer, erase, and write the data in the first page
    memcpy(buffer, (void*) start_align, start_len);
    retval = MXC_FLC_PageErase(start_align);
    
    if (retval != E_NO_ERROR) {
        return retval;
    }
    
    retval = MXC_FLC_Write(start_align, start_len, buffer);
    
    if (retval != E_NO_ERROR) {
        return retval;
    }
    
    // Buffer, erase, and write the data in the last page
    memcpy(buffer, (void*) end, end_len);
    retval = MXC_FLC_PageErase(end_align);
    
    if (retval != E_NO_ERROR) {
        return retval;
    }
    
    retval = MXC_FLC_Write(end, end_len, buffer);
    
    if (retval != E_NO_ERROR) {
        return retval;
    }
    
    // Erase the remaining pages. MultiPageErase will not erase if start is greater than end.
    for (i = (start_align + MXC_FLASH_PAGE_SIZE); i < (end_align - MXC_FLASH_PAGE_SIZE); i += MXC_FLASH_PAGE_SIZE) {
        retval = MXC_FLC_PageErase(i);
        
        if (retval != E_NO_ERROR) {
            break;
        }
    }
    
    return retval;
}

//******************************************************************************
int write_test(unsigned int start_addr) {
    int i = 0, fail = 0;

    // Initializing Test Data
	for (i = 0; i < TESTSIZE; i++) {
		testdata[i] = i;
	}

	// Writing TESTSIZE number of words to flash memory
	i = 0;
    for (testaddr = (start_addr); i < TESTSIZE; testaddr += 4) {
        // Clear and enable flash programming interrupts
        isr_flags = 0;
        isr_cnt = 0;

        // Write a word
        if (MXC_FLC_Write(testaddr, 4, &testdata[i]) != E_NO_ERROR) {
            printf("Failure in writing a word.\n");
            fail += 1;
            break;
        }
        else {
            //printf("Word %d : %u is written to the flash.\n", i, testdata[i]);
        }

        // Checking Interrupt
        if ((isr_cnt != 1) && (isr_flags != MXC_F_FLC_INTR_DONE)) {
            printf("Interrupt did not occur at 0x%08x\n", (unsigned int) testaddr);
            fail += 1;
        }

        // Verify that word is written properly
        if (flash_verify(testaddr, 4, (uint8_t*) &testdata[i]) != E_NO_ERROR) {
            printf("Word is not written properly.\n");
            fail += 1;
            break;
        }

        if (i < 16) {
            printf("Word %d written properly and has been verified.\n", i);
        }
        else if (i == 16) {
            printf("Continuing for %d more words...\n\n", TESTSIZE - i);
        }

        i++;
    }

    return fail;
}

//******************************************************************************
int page_erase_test(unsigned int page_addr) {
	MXC_FLC_PageErase(page_addr);
    if (check_erased(page_addr, MXC_FLASH_PAGE_SIZE)) {
        printf("Page Erase is verified\n\n");
        return 0;
    }
    else {
        printf("Page Erase failed\n\n");
        return 1;
    }
}

//******************************************************************************
int partial_erase_test(unsigned int base_addr) {
	uint32_t start, end;
	uint32_t buffer[0x1000];
	start = (base_addr + MXC_FLASH_PAGE_SIZE + 0x500);
    end = (base_addr + (2 * MXC_FLASH_PAGE_SIZE) - 0x500);
    flash_erase(start, end, buffer, 0x1000);

    if (check_erased(start, ((end - start) - 0x1000))) {
        printf("Flash Erase is verified\n\n");
        return 0;
    }
    else {
        printf("Flash Erase failed\n\n");
        return 1;
    }
}

//******************************************************************************
int main(void)
{
    int fail = 0;
    int error_status;
    
    /* Note: This example must execute out of RAM, due to MXC_FLC_MassErase() call, below */
    printf("\n\n***** Flash Control Example *****\n");
    NVIC_SetRAM();
    // Initialize the Flash
    flash_init();
    
    // Clear and enable flash programming interrupts
    interrupt_enabler(MXC_FLC0);
    interrupt_enabler(MXC_FLC1);
    isr_flags = 0;
    isr_cnt = 0;
    
    error_status = MXC_FLC_MassErase();

    if (error_status == E_NO_ERROR) {
        printf("Flash erased.\n");
    }
    else if (error_status == E_BAD_STATE) {
        printf("Flash erase operation is not allowed in this state.\n");
    }
    else {
        printf("Fail to erase flash's content.\n");
        fail += 1;
    }

    // Checking Interrupt's occurence
    if ((isr_cnt != 2) && (isr_flags != MXC_F_FLC_INTR_DONE)) {
        printf("Interrupt did not occur\n");
        fail += 1;
    }
    // Check flash's content
    if (check_erased(MXC_FLASH_MEM_BASE, MXC_FLASH_MEM_SIZE_TEST)) {
        printf("Flash mass erase is verified.\n\n");
    }
    else {
        printf("Flash mass erase failed.\n\n");
        fail += 1;
    }
    
    MXC_ICC_Disable();

    // Flash write tests
    // Flash 0
    printf("Writing %d 32-bit words to flash 0\n", TESTSIZE);
    printf("Size of testdata : %d\n", sizeof(testdata));
    fail += write_test(MXC_FLASH0_MEM_BASE);
    //Flash 1
    printf("Writing %d 32-bit words to flash 1\n", TESTSIZE);
    printf("Size of testdata : %d\n", sizeof(testdata));
    fail += write_test(MXC_FLASH1_MEM_BASE);


    //Page Erase Tests
    //Flash 0
    printf("Attempting to erase page 0 in flash bank 0\n");
    fail += page_erase_test(MXC_FLASH0_MEM_BASE);
    //Flash 1
    printf("Attempting to erase page 0 in flash bank 1\n");
    fail += page_erase_test(MXC_FLASH1_MEM_BASE);
    

    // Erase parital pages or wide range of pages and keep the data on the page not inbetween start and end.
    // Flash 0
    printf("Attempting to partially erase pages 2 and 3 in flash bank 0\n");
    fail += partial_erase_test(MXC_FLASH0_MEM_BASE);
    // Flash1
    printf("Attempting to partially erase pages 2 and 3 in flash bank 1\n");
    fail += partial_erase_test(MXC_FLASH1_MEM_BASE);
    
    
    MXC_ICC_Enable();
    if (fail == 0) {
        printf("Example Succeeded\n");
    }
    else {
        printf("Example Failed\n");
    }
    
    while (1);
    
    return 0;
}
