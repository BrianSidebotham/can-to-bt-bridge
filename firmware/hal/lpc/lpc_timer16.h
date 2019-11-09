/**
    \file lpc_timer.h
    \brief LPC Timer (16-bit and 32-bit) header support file

    Support for the LPC Cortex-M0 Timers. Most of this work
    is derived from the LPC11C14 user manual (Rev 4. 4th March 2011)
    Chapter 15

    The ARMv6-M Architecture Reference Manual should also be consulted

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


#ifndef LPC_TIMER_H
#define LPC_TIMER_H 1

#include "hal/lpc/lpc_common.h"

/** LPC Timer structure */
typedef struct {
    rw uint32_t ir;
    rw uint32_t tcr;
    rw uint32_t tc;
    rw uint32_t pr;
    rw uint32_t pc;
    rw uint32_t mcr;
    rw uint32_t mr0;
    rw uint32_t mr1;
    rw uint32_t mr2;
    rw uint32_t mr3;
    rw uint32_t ccr;
    rw uint32_t cr0;
    rw uint32_t emr;
    ro uint32_t reserved[12];
    rw uint32_t ctcr;
    rw uint32_t pwmc;
    } lpc_timer_t;

#endif
