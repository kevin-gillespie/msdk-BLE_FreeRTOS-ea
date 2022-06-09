################################################################################
 # Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 #
 # Permission is hereby granted, free of charge, to any person obtaining a
 # copy of this software and associated documentation files (the "Software"),
 # to deal in the Software without restriction, including without limitation
 # the rights to use, copy, modify, merge, publish, distribute, sublicense,
 # and/or sell copies of the Software, and to permit persons to whom the
 # Software is furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included
 # in all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 # OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 # MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 # IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 # OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 # ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 # OTHER DEALINGS IN THE SOFTWARE.
 #
 # Except as contained in this notice, the name of Maxim Integrated
 # Products, Inc. shall not be used except as stated in the Maxim Integrated
 # Products, Inc. Branding Policy.
 #
 # The mere transfer of this software does not imply any licenses
 # of trade secrets, proprietary technology, copyrights, patents,
 # trademarks, maskwork rights, or any other form of intellectual
 # property whatsoever. Maxim Integrated Products, Inc. retains all
 # ownership rights.
 #
 # $Date: 2016-03-11 11:46:02 -0600 (Fri, 11 Mar 2016) $
 # $Revision: 21838 $
 #
 ###############################################################################

ifeq "$(BOARD_DIR)" ""
$(error BOARD_DIR must be set)
endif

PROJ_CFLAGS+=-DEMULATOR=1 -DBSP=\"EMULATOR\"
PROJ_CFLAGS+=-DHIRC_FREQ=96000000
PROJ_CFLAGS+=-DHIRC96_FREQ=96000000
# NOTE: Try not to set PeripheralClock manually.  Set the HIRC/HIRC96 frequencies
# and let SystemClockUpdate() calculate the correct frequencies.
# PROJ_CFLAGS+=-DPeripheralClock=20000000

# Source files for this test (add path to VPATH below)
SRCS += board.c
SRCS += stdio.c
SRCS += led.c
SRCS += pb.c
SRCS += rom_stub.c

# Where to find BSP source files
VPATH += $(BOARD_DIR)/Source
VPATH += $(BOARD_DIR)/../Source

# Where to find BSP header files
IPATH += $(BOARD_DIR)/Include
IPATH += $(BOARD_DIR)/../Include
