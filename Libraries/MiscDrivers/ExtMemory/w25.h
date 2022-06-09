/**
 * @file
 * @brief BSP driver to communicate via SPI/QPI with an W25 Serial Flash Memory.
 */
 /* ****************************************************************************
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
 *
 **************************************************************************** */

/* Define to prevent redundant inclusion */
#ifndef _W25_H_
#define _W25_H_

/* **** Includes **** */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup bsp
 * @defgroup w25_driver W25 SPI Multi-I/O Flash Memory Driver
 * @{
 */
/* **** Definitions **** */

#define W25_E_SUCCESS        0
#define W25_E_ERROR          -1
#define W25_E_BUSY           -99
#define W25_E_BAD_STATE      -98
#define W25_E_TIME_OUT       -97
#define W25_E_BAD_PARAM      -96

#define W25_EXP_ID     0xef4018

#define W25_Read_DUMMY             8       /**< Dummy byte sent on a standard read command per the W25 datasheet.         */
#define W25_DREAD_DUMMY            4       /**< Dummy data sent on a fast-read (Dual) read command per the W25 datasheet. */
#define W25_QREAD_DUMMY            6       /**< Dummy data sent on a fast-read (Quad) read command per the W25 datasheet. */

#define W25_WIP_MASK               0x01        /**< Status Reg-1: Work In Progress          */
#define W25_WEL_MASK               0x02        /**< Status Reg-1: Write Enable Latch mask   */
#define W25_QE_MASK                0x02        /**< Status Reg-2: Quad-SPI enable mask      */

#define W25_TB_POS                 5
#define W25_TB_MASK                (1 << W25_TB_POS)            /**< Top/Bottom Select mask         */
#define W25_BP_POS                 2
#define W25_BP_MASK                (0x7 << W25_BP_POS)          /**< Block Protect mask             */
#define W25_SR1_FP_MASK            (W25_TB_MASK | W25_BP_MASK)  /**< Mask of all flash block protect bits in status register 1 */
#define W25_CMP_POS                6
#define W25_CMP_MASK               (1 << W25_CMP_POS)           /**< Flash protect complement bit mask */

#define W25_GET_BP_IN_FIRST_HALF(pg)       ((pg < 4)              ? 1 :\
                                            (pg > 3 && pg < 8)    ? 2 :\
                                            (pg > 7 && pg < 16)   ? 3 :\
                                            (pg > 15 && pg < 32)  ? 4 :\
                                            (pg > 31 && pg < 64)  ? 5 :\
                                            (pg > 63 && pg < 128) ? 6 :\
                                            -1 )
#define W25_GET_BP_IN_SECOND_HALF(pg)      ((pg < 192)             ? 5 :\
                                            (pg > 191 && pg < 224) ? 4 :\
                                            (pg > 223 && pg < 240) ? 3 :\
                                            (pg > 239 && pg < 248) ? 2 :\
                                            (pg > 247 && pg < 252) ? 1 :\
                                            -1 )

#define W25_DEVICE_SIZE            0x1000000
#define W25_BLOCK_SIZE             0x10000
#define W25_PAGE_SIZE              256
/**
 * @ingroup w25_driver
 * @defgroup W25_Commands W25 SPI Command Definitions
 * @{
 */
#define W25_CMD_RST_EN             0x66         /**< Reset Enable                   */
#define W25_CMD_RST_MEM            0x99         /**< Reset Memory                   */
#define W25_CMD_ID                 0x9F         /**< ID                             */
#define W25_CMD_WRITE_EN           0x06         /**< Write Enable                   */
#define W25_CMD_WRITE_DIS          0x04         /**< Write Disable                  */

#define W25_CMD_READ               0x0B        /**< Read                           */
#define W25_CMD_DREAD              0xBB        /**< Dual SPI Read                  */
#define W25_CMD_QREAD              0xEB        /**< Quad SPI Read                  */

#define W25_CMD_READ_SR1           0x05        /**< Read Status Register 1         */
#define W25_CMD_WRITE_SR1          0x01        /**< Write Status Register 1        */

#define W25_CMD_READ_SR2           0x35        /**< Read Status Register 2         */
#define W25_CMD_WRITE_SR2          0x31        /**< Write Status Register 2        */

#define W25_CMD_READ_SR3           0x15        /**< Read Status Register 3         */
#define W25_CMD_WRITE_SR3          0x11        /**< Write Status Register 3        */

#define W25_CMD_PPROG              0x02         /**< Page Program                   */
#define W25_CMD_QUAD_PROG          0X32         /**< Quad (4 x I/O) Page Program    */

#define W25_CMD_4K_ERASE           0x20         /**< Page Erase                     */
#define W25_CMD_32K_ERASE          0x52         /**< Sector Type 2 (32KB) Erase     */
#define W25_CMD_64K_ERASE          0xD8         /**< Sector Type 3 (64KB) Erase     */
#define W25_CMD_BULK_ERASE         0xC7         /**< Bulk Erase                     */
/**@} end of group w25_commands */

/**
 * Enumeration type to select the size for an Erase command.
 */
typedef enum {
    W25_Erase_4K,      /**< 4KB Sector Erase  */
    W25_Erase_32K,     /**< 32KB Block Erase */
    W25_Erase_64K,     /**< 64KB Block Erase */
} 
W25_Erase_t;

/**
 * Enumeration type to select status register.
 */
typedef enum {
    W25_StatusReg_1,     /**< Status Register 1 */
    W25_StatusReg_2,     /**< Status Register 2 */
    W25_StatusReg_3,     /**< Status Register 3 */
}
W25_StatusReg_t;

/**
 * Enumeration type to specify data width.
 */
typedef enum {
    W25_DataLine_Single,   /**< 1 Data Line.  */
    W25_DataLine_Dual,     /**< 2 Data Lines (x2). */
    W25_DataLine_Quad      /**< 4 Data Lines (x4). */
}
W25_DataLine_t;

/**
 * Struct used to hold the start and end addresses of flash currently available to write
 */
typedef struct {
    uint32_t start_addr;    /**< Start address of flash available to write */
    uint32_t end_addr;      /**< End address of flash available to write */
} W25_Flash_Unblk_t;

/**
 * Struct definition to configure physical communication layer.
 */
typedef struct {
    int(*init)(void);
    int(*read)(uint8_t* read, unsigned len, unsigned deassert, W25_DataLine_t d_line);
    int(*write)(const uint8_t* write, unsigned len, unsigned deassert, W25_DataLine_t d_line);
    int(*clock)(unsigned len, unsigned deassert);
}
W25_Config_t;

/* *** Globals **** */

/* **** Function Prototypes **** */

/**
 * @brief      Configure W25 communication line
 * @param      cfg    Configuration structure
 * @retval     0         Success
 * @retval     Non-zero  Error condition
 */
int W25_Configure(W25_Config_t *cfg);

/**
 * @brief      Initialize SPI configuration and reset W25
 * @retval     0         Success
 * @retval     Non-zero  Error condition
 */
int W25_Init(void);

/**
 * @brief       Reset the W25 flash memory.
 * @retval      0         Success
 * @retval      Non-zero  Error condition
 */
int W25_Reset(void);

/**
 * @brief       Read manufacturer ID.
 * @retval      ID of the device, or 0 if an error occurred
 */
uint32_t W25_ID(void);

/**
 * @brief       Enable/Disable the Quad Enable(QE) bit in the status register.
 * @param       enable    @arg @b 1 enables Quad Mode. @arg @b 0 disables Quad Mode.
 * @retval      0         Success
 * @retval      Non-zero  Error condition
 */
int W25_Quad(int enable);

/**
 * @brief       Read data out by using 4-wire SPI mode.
 * @param       address         Start address to read from
 * @param       rx_buf          Pointer to the buffer of receiving data
 * @param       rx_len          Size of the data to read
 * @param       d_line          #W25_DataLine_t for how many data lines to use
 * @retval      0         Success
 * @retval      Non-zero  Error condition
 */
int W25_Read(uint32_t address, uint8_t* rx_buf, uint32_t rx_len, W25_DataLine_t d_line);

/**
 * @brief       Program the memory to @p tx_buf and length @p tx_len, applies to both SPI and QPI modes.
 * @details
 *        - SPI mode: All operations are in 4-wire SPI mode.
 *        - QPI mode: All operations are in quad SPI mode.
 * @param       address         Start address to program.
 * @param       tx_buf          Pointer to the buffer of data to write.
 * @param       tx_len          Size of the data to write.
 * @param       d_line          #W25_DataLine_t for how many data lines to use.
 * @retval      0         Success
 * @retval      Non-zero  Error condition
 */
int W25_Program_Page(uint32_t address, uint8_t* tx_buf, uint32_t tx_len, W25_DataLine_t d_line);

/**
 * @brief       Bulk erase the W25 flash memory.
 * @warning     Bulk erase typically takes between 100 to 150 seconds.
 * @retval      0         Success
 * @retval      Non-zero  Error condition
 */
int W25_Bulk_Erase(void);

/**
 * @brief       Erase memory segments
 * @param       address  Start address to begin erasing.
 * @param       size     Size to erase, see #W25_Erase_t.
 * @retval      0         Success
 * @retval      Non-zero  Error condition
 */
int W25_Erase(uint32_t address, W25_Erase_t size);

/**
 * @brief       Read status register.
 * @param       buf      Pointer to store the value of the status register.
 * @param       reg_num  Selects which status register to read (see #W25_StatusReg_t for valid values)
 */
int W25_Read_SR(uint8_t* buf, W25_StatusReg_t reg_num);

/**
 * @brief       Write status register
 * @param       value  Value to write to the status register.
 * @param       reg_num  Selects which status register to write (see #W25_StatusReg_t for valid values)
 */
int W25_Write_SR(uint8_t value, W25_StatusReg_t reg_num);

/**
 * @brief       Configures write protection scheme to protect data stored in the flash block in which "addr" is located
 * @details     The W25 write protection scheme protects sections of memory either starting at the beginning of memory 
 *              up through block selcted by "addr" or from the end of memory down through the memory block selected by "addr". 
 * 
 * @param       addr        Address to protect from being modified, passing a 0 for this argument clears all write protection 
 * @param       start       True - protect memory from start of flash up through the protected address
 *                          False - protect memory from end of flash down trhough the protected address 
 * @retval      0           Success
 * @retval      Non-zero    Error condition    
 */
int W25_Flash_Block_WP(uint32_t addr, uint32_t begin);

/**
 * @brief       Returns the start and end address of the available flash memory based on the current write protection scheme
 * 
 * @returns     Struct containing the start and addresses of available flash
 */
W25_Flash_Unblk_t W25_GetAvailableFlash(void);

/**@} end of group w25_driver */
#ifdef __cplusplus
}
#endif

#endif /* _W25_H_ */
