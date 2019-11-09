/**
    \file lpc_can.h
    \brief LPC CAN Peripheral header support file

    Support for the LPC Cortex-M0 CAN peripheral block. Most of this work
    is derived from the LPC11C14 user manual (Rev 4. 4th March 2011)

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
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
    OUT OF THE USE OF THIS

*/


#ifndef LPC_C_CAN_H
#define LPC_C_CAN_H 1

#include <stdint.h>
#include "hal/lpc/lpc_common.h"

/** A definition of the base address of the c_can peripheral */
#define LPC_CCAN_BASE_ADDRESS   ((lpc_c_can_t*)(APB_BASE + 0x50000))

/** */
typedef struct {
    rw uint32_t cntl;
    rw uint32_t stat;
    ro uint32_t ec;
    rw uint32_t bt;
    ro uint32_t canint;
    rw uint32_t test;
    rw uint32_t brpe;
    ro uint32_t reserved1;
    rw uint32_t if1_cmdreq;
    rw uint32_t if1_cmdmsk;
    rw uint32_t if1_msk1;
    rw uint32_t if1_msk2;
    rw uint32_t if1_arb1;
    rw uint32_t if1_arb2;
    rw uint32_t if1_mctrl;
    rw uint32_t if1_da1;
    rw uint32_t if1_da2;
    rw uint32_t if1_db1;
    rw uint32_t if1_db2;
    ro uint32_t reserved2[13];
    rw uint32_t if2_cmdreq;
    rw uint32_t if2_cmdmsk;
    rw uint32_t if2_msk1;
    rw uint32_t if2_msk2;
    rw uint32_t if2_arb1;
    rw uint32_t if2_arb2;
    rw uint32_t if2_mctrl;
    rw uint32_t if2_da1;
    rw uint32_t if2_da2;
    rw uint32_t if2_db1;
    rw uint32_t if2_db2;
    ro uint32_t reserved3[21];
    rw uint32_t txreq1;
    rw uint32_t txreq2;
    ro uint32_t reserved4[6];
    rw uint32_t nd1;
    rw uint32_t nd2;
    rw uint32_t reserved5[6];
    rw uint32_t ir1;
    rw uint32_t ir2;
    ro uint32_t reserved6[6];
    rw uint32_t msgv1;
    rw uint32_t msgv2;
    ro uint32_t reserved7[6];
    rw uint32_t clkdiv;
    } lpc_c_can_t;

/** Definitions for the bits in the cntl ccan register */
#define LPC_CCAN_CNTL_INIT      (1 << 0)
#define LPC_CCAN_CNTL_IE        (1 << 1)
#define LPC_CCAN_CNTL_SIE       (1 << 2)
#define LPC_CCAN_CNTL_EIE       (1 << 3)
#define LPC_CCAN_CNTL_DAR       (1 << 5)
#define LPC_CCAN_CNTL_CCE       (1 << 6)
#define LPC_CCAN_CNTL_TEST      (1 << 7)

/** Definitions for the CAN clock divider register for baud rate
    generation */
#define LPC_CCAN_CLKDIV_1       0
#define LPC_CCAN_CLKDIV_2       1
#define LPC_CCAN_CLKDIV_3       2
#define LPC_CCAN_CLKDIV_4       3
#define LPC_CCAN_CLKDIV_5       4
#define LPC_CCAN_CLKDIV_6       5
#define LPC_CCAN_CLKDIV_7       6
#define LPC_CCAN_CLKDIV_8       7
#define LPC_CCAN_CLKDIV_9       8
#define LPC_CCAN_CLKDIV_10      9
#define LPC_CCAN_CLKDIV_11      10
#define LPC_CCAN_CLKDIV_12      11
#define LPC_CCAN_CLKDIV_13      12
#define LPC_CCAN_CLKDIV_14      13
#define LPC_CCAN_CLKDIV_15      14
#define LPC_CCAN_CLKDIV_16      15

/** Definitions of the bits in the CCAN Status register */
#define LPC_CCAN_STAT_TXOK      (1 << 3)
#define LPC_CCAN_STAT_RXOK      (1 << 4)
#define LPC_CCAN_STAT_EPASS     (1 << 5)
#define LPC_CCAN_STAT_EWARN     (1 << 6)
#define LPC_CCAN_STAT_BOFF      (1 << 7)

/** Definitions of the bits in the CCAN CMDREQ register */
#define LPC_CCAN_CMDREQ_BUSY    (1 << 15)

/** Definitions of the bits in the CCAN CMDMSK register */
#define LPC_CCAN_CMDMSK_DATA_B  (1 << 0)
#define LPC_CCAN_CMDMSK_DATA_A  (1 << 1)
#define LPC_CCAN_CMDMSK_NEWDAT  (1 << 2)
#define LPC_CCAN_CMDMSK_CLRINTPND   (1 << 3)
#define LPC_CCAN_CMDMSK_CTRL    (1 << 4)
#define LPC_CCAN_CMDMSK_ARB     (1 << 5)
#define LPC_CCAN_CMDMSK_MASK    (1 << 6)
#define LPC_CCAN_CMDMSK_WR      (1 << 7)
#define LPC_CCAN_CMDMSK_RD      (0 << 7)

/* ARB1 ------------------------------------------------------------------- */

/* Definitions of the bits in the CCAN ARB1 register */

/** Message identifier mask for ARB1 (15:0 of 29-bit identifier) */
#define LPC_CCAN_ARB1_IDMSK     (0xFFFF)


/* ARB2 ------------------------------------------------------------------- */

/* Definitions of the bits in the CCAN ARB2 register */

/** Message direction, 0 = receive */
#define LPC_CCAN_ARB2_DIR       (1 << 13)

/** Extend identifier, 1 = 29-bit extended identifier */
#define LPC_CCAN_ARB2_XTD       (1 << 14)

/** Message Value, 1 = message initialise and valid - should be used by the
    message handler */
#define LPC_CCAN_ARB2_MSGVAL    (1 << 15)

/** Message identifier mask for ARB2 (either 11-bit identifier, or 18:16 of
    29-bit identifier */
#define LPC_CCAN_ARB2_IDMSK     (0x1FFF)


/* MCTRL ------------------------------------------------------------------ */

/* Definitions of the bits in the CCAN MCTRL register */

/** Data length code mask - the length of the data in the packet */
#define LPC_CCAN_MCTRL_DLC      (0xF)

/** End of buffer, 1 - single message object, or the last object in FIFO */
#define LPC_CCAN_MCTRL_EOB      (1 << 7)

/** Transmit request, 1 = transmission of this object is not yet complete */
#define LPC_CCAN_MCTRL_TXRQST   (1 << 8)

/** Remote enable, 1 = at the reception of a remote frame, TXREQST is set */
#define LPC_CCAN_MCTRL_RMTEN    (1 << 9)

#define LPC_CCAN_MCTRL_RXIE     (1 << 10)
#define LPC_CCAN_MCTRL_TXIE     (1 << 11)
#define LPC_CCAN_MCTRL_UMASK    (1 << 12)
#define LPC_CCAN_MCTRL_INTPND   (1 << 13)
#define LPC_CCAN_MCTRL_MSGLST   (1 << 14)
#define LPC_CCAN_MCTRL_NEWDAT   (1 << 15)


extern void LPCCcanInit(void);

#endif
