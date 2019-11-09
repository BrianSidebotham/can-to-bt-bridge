/**
    \file lpc_nvic.h
    \brief LPC Nested Vectored Interrupt Controller header support file

    Support for the LPC Cortex-M0 NVIC peripheral block. Most of this work
    is derived from the LPC11C14 user manual (Rev 4. 4th March 2011)

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


#ifndef LPC_NVIC_H
#define LPC_NVIC_H 1

/** The LPC11C14 Interrupt vector numbers for use with the Cortex-M0 */
typedef enum {
    IV_PIO0_0,
    IV_PIO0_1,
    IV_PIO0_2,
    IV_PIO0_3,
    IV_PIO0_4,
    IV_PIO0_5,
    IV_PIO0_6,
    IV_PIO0_7,
    IV_PIO0_8,
    IV_PIO0_9,
    IV_PIO0_10,
    IV_PIO0_11,
    IV_PIO1_0,
    IV_C_CAN,
    IV_SPI_SSP1,
    IV_I2C,
    IV_CT16B0,
    IV_CT16B1,
    IV_CT32V0,
    IV_CT32B1,
    IV_SPI_SSP0,
    IV_UART,

    IV_ADC = 24,
    IV_WDT,
    IV_BOD,

    IV_PIO_3 = 28,
    IV_PIO_2,
    IV_PIO_1,
    IV_PIO_0,
    } int_vect_t;

#endif
