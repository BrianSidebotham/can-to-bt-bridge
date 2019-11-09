/**
    \file m0_nvic.c
    \brief Cortex-M0 Nested Vectored Interrupt Controller support file

    Support for the Cortex-M0 NVIC. Most of this work is derived from the
    ARMv6-M Architecture Reference Manual (Section B3.4)

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


#include "hal/arm/cortex/m0.h"

/**
    \fn void M0IntEnable(int vector)
    \param vector The interrupt vector number to enable (between 0 and 31)
    \brief Enables the interrupt whose vector number is \a vector

*/
void M0IntEnable(int vector)
{
    /* Do some basic checks, Cortex M0 only allows for 32 interrupt vectors */
    if ((vector > 31) || (vector < 0))
        return;

    /* Set the corresponding vector bit in the interrupt set en. register */
    armv6_core.armv6_nvic->iser |= (1 << vector);
}


/**
    \fn void M0IntDisable(int vector)
    \param vector The interrupt vector number to disable (between 0 and 31)
    \brief Disables the interrupt whose vector number is \a vector

*/
void M0IntDisable(int vector)
{
    /* Do some basic checks, Cortex M0 only allows for 32 interrupt vectors */
    if ((vector > 31) || (vector < 0))
        return;

    /* Set the corresponding vector bit in the interrupt clear register */
    armv6_core.armv6_nvic->icer |= (1 << vector);
}


/**
    \fn void M0IntSetPending(int vector)
    \param vector The interrupt vector number to set pending (between 0 and 31)
    \brief Set the interrupt status pending
*/
void M0IntSetPending(int vector)
{
    /* Do some basic checks, Cortex M0 only allows for 32 interrupt vectors */
    if ((vector > 31) || (vector < 0))
        return;

    /* Set the corresponding vector bit in the interrupt clear register */
    armv6_core.armv6_nvic->ispr |= (1 << vector);
}


/**
    \fn void M0IntClrPending(int vector)
    \param vector The interrupt vector number to clr pending (between 0 and 31)
    \brief Clear the interrupt pending status
*/
void M0IntClrPending(int vector)
{
    /* Do some basic checks, Cortex M0 only allows for 32 interrupt vectors */
    if ((vector > 31) || (vector < 0))
        return;

    /* Set the corresponding vector bit in the interrupt clear register */
    armv6_core.armv6_nvic->icpr |= (1 << vector);
}
