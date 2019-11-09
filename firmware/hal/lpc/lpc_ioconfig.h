/**
    \file lpc_ioconfig.h
    \brief LPC IOCONFIG header support file

    Support for the LPC Cortex-M0 IOCONFIG peripheral. Most of this work
    is derived from the LPC11C14 user manual (Rev 4. 4th March 2011)
    Chapter 7

    This file is provided under the modified BSD licence. This licence is
    compatible with the GNU GPL licence. A copy of the modified BSD licence
    is included below:

    --- Modified BSD Licence -------------------------------------------------

    Copyright (c) 2011, Brian Sidebotham
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    * Neither the name of the opencar project nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
    PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR
    ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGvoid LPCCcanInit(void)LIGENCE OR OTHERWISE) ARISING IN ANY WAY
    OUT OF THE USE OF THIS

*/


#ifndef LPC_IOCONFIG_H
#define LPC_IOCONFIG_H 1

#include "hal/lpc/lpc_common.h"

/** A definition of the base address of the ioconfig peripheral */
#define LPC_IOCONFIG_BASE_ADDRESS   ((lpc_ioconfig_t*)(APB_BASE + 0x44000))

typedef struct {
    rw uint32_t pio2_6;
    ro uint32_t reserved1;
    rw uint32_t pio2_0;
    rw uint32_t reset_pio0_0;
    rw uint32_t pio0_1;
    rw uint32_t pio1_8;
    ro uint32_t reserved2;
    rw uint32_t pio0_2;
    rw uint32_t pio2_7;
    rw uint32_t pio2_8;
    rw uint32_t pio2_1;
    rw uint32_t pio0_3;
    rw uint32_t pio0_4;
    rw uint32_t pio0_5;
    rw uint32_t pio1_9;
    rw uint32_t pio3_4;
    rw uint32_t pio2_4;
    rw uint32_t pio2_5;
    rw uint32_t pio3_5;
    rw uint32_t pio0_6;
    rw uint32_t pio0_7;
    rw uint32_t pio2_9;
    rw uint32_t pio2_10;
    rw uint32_t pio2_2;
    rw uint32_t pio0_8;
    rw uint32_t pio0_9;
    rw uint32_t swclk_pio0_10;
    rw uint32_t pio1_10;
    rw uint32_t pio2_11;
    rw uint32_t r_pio0_11;
    rw uint32_t r_pio1_0;
    rw uint32_t r_pio1_1;
    rw uint32_t r_pio1_2;
    rw uint32_t pio3_0;
    rw uint32_t pio3_1;
    rw uint32_t pio2_3;
    rw uint32_t swdio_pio1_3;
    rw uint32_t pio1_4;
    rw uint32_t pio1_11;
    rw uint32_t pio3_2;
    rw uint32_t pio1_5;
    rw uint32_t pio1_6;
    rw uint32_t pio1_7;
    rw uint32_t pio3_3;
    rw uint32_t sck_loc;
    rw uint32_t dsr_loc;
    rw uint32_t dcd_loc;
    rw uint32_t ri_loc;
    } lpc_ioconfig_t;

/** Common modes available for most (all?) pins */
#define LPC_IO_PULLUP_INACTIVE      (0 << 3)
#define LPC_IO_PULLUP_ENABLED       (1 << 3)
#define LPC_IO_PULLDN_ENABLED       (2 << 3)
#define LPC_IO_REPEATER_MODE        (3 << 3)

/** Enable io pin hysteresis */
#define LPC_IO_HYSTERESIS           (1 << 5)

/** Set the pin to analog instead of digial functions */
#define LPC_IO_ANALOG               (1 << 7)

#endif
