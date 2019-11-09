/**
    \file m0.h
    \brief ARM Cortex M0 definitions for the valvers opencar project

    Support file for the ARM cortex M0 processor core. Most of this work is
    derived from the ARMv6-M Architecture Reference Manual (ARM DDI 0419C)

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

/* multiple inclusion trap */
#ifndef M0_H
#define M0_H 1

/* Use the C standard types */
#include <stdint.h>
#include "hal/arm/cortex/m0_nvic.h"

/* Read and write registers are still marked as volatile! */
#ifndef rw
#define rw  volatile
#endif

/* Read only registers */
#ifndef ro
#define ro  const volatile
#endif

/* Write only registers - same as read/write as we cannot limit reading! */
#ifndef wo
#define wo  rw
#endif



/* ------------------------------------------------------------------------ */
/* ARMv6-M System Control Block ------------------------------------------- */

/** From the ARMV6-M Architecture Reference Manual ARM DDI 0419C (B3-263)
    The Cortex-M0 System Control block */
typedef struct {
    rw uint32_t actlr;  /**< SysCon Auxillary Control Register */
    ro uint32_t reserved0[829];
    ro uint32_t cpuid;  /**< SysCon CPUID Base Register (RO) */
    rw uint32_t icsr;   /**< SysCon Interrupt Control State Register */
    rw uint32_t vtor;   /**< SysCon Vector Table Offset Register */
    rw uint32_t aircr;  /**< SysCon Application Interrupt and Reset Register */
    rw uint32_t scr;    /**< SysCon System Control Register (OPTIONAL) */
    ro uint32_t ccr;    /**< SysCon Configuration Control Register (RO) */
    ro uint32_t reserved1;
    rw uint32_t shpr2;  /**< SysCon System Handler Priority Register 2 */
    rw uint32_t shpr3;  /**< SysCon System Handler Priority Register 3 */
    rw uint32_t shcsr;  /**< SysCon System Handler Control and State Reg */
    ro uint32_t reserved2;
    rw uint32_t dfsr;   /**< SysCon Debug Fault Status Register */
    } armv6_syscon_t;

/** The Cortex-M0 system control block base address

    Reference: ARMv6-M Architecture Reference Manual B3.2.2 */
#define COREM0_SYSCON_BASE_ADDRESS  (armv6_syscon_t*)0xE000E008



/* ------------------------------------------------------------------------ */
/* ARMv6-M System timer (SysTick)  ---------------------------------------- */

/** From the ARMV6-M Architecture Reference Manual ARM DDI 0419C (B3-276).
    The system tick control registers */
typedef struct {
    rw uint32_t csr;    /**< SysTick Control and Status Register */
    rw uint32_t rvr;    /**< SysTick Reload Value Register */
    rw uint32_t cvr;    /**< SysTick Current Value Register */
    rw uint32_t calib;  /**< SysTick 10ms Calibration Register */
    } armv6_systick_t;

/** Bit in SysTick CSR register - Set to one when the System Tick timer has
    reached zero since the last read of the CSR register. This bit is
    read only.

    Reference: ARMv6-M Architecture Reference Manual B3.3.3 */
#define COREM0_SYSTCSR_COUNTFLAG    (1 << 16)

/** Bit in SysTick CSR register - Clear this bit when the SysTick needs to
    use an optional external reference clock

    Reference: ARMv6-M Architecture Reference Manual B3.3.3 */
#define COREM0_SYSTCSR_CLKSOURCE    (1 << 2)

/** Bit in SysTick CSR register - Set this to cause the SysTick module to set
    the exception flag when the SysTick timer reaches zero. Effectively
    enables the SysTick exception handler

    Reference: ARMv6-M Architecture Reference Manual B3.3.3 */
#define COREM0_SYSTCSR_TICKINT      (1 << 1)

/** Bit in SysTick CSR register - Set this bit to enable the SysTick counter

    Reference: ARMv6-M Architecture Reference Manual B3.3.3 */
#define COREM0_SYSTCSR_ENABLE       (1 << 0)

/** The Cortex-M0 system timer (SysTick) base address

    Reference: ARMv6-M Architecture Reference Manual B3.3.2 */
#define COREM0_SYST_BASE_ADDRESS    (armv6_systick_t*)0xE000E010



/* ------------------------------------------------------------------------ */
/* ARMv6-M Nested Vectored Interrupt Controller --------------------------- */

/** From the ARMV6-M Architecture Reference Manual ARM DDI 0419C (B3-283).
    The Nested Vector Interrupt Controller registers */
typedef struct {
    rw uint32_t iser;   /**< NVIC Interrupt Set-Enable Register */
    ro uint32_t reserved0[31];
    rw uint32_t icer;   /**< NVIC Interrupt Clear-Enable Register */
    ro uint32_t reserved1[31];
    rw uint32_t ispr;   /**< NVIC Interrupt Set-Pending Register */
    ro uint32_t reserved2[31];
    rw uint32_t icpr;   /**< NVIC Interrupt Clear-Pending Register */
    ro uint32_t reserved3[31 + 64];
    rw uint32_t ipr[8]; /**< NVIC Interrupt Priority Registers */
    } armv6_nvic_t;

/** The Cortex-M0 nested vectored interrupt controller base address

    Reference: ARMv6-M Architecture Reference Manual B3.4.2 */
#define COREM0_NVIC_BASE_ADDRESS    (armv6_nvic_t*)0xE000E100



/* ------------------------------------------------------------------------ */
/* ARMv6-M MPU Registers (PMSAv6) ----------------------------------------- */

/** From the ARMV6-M Architecture Reference Manual ARM DDI 0419C (B3-293)
    The MPU (PMSAv6) Registers */
typedef struct {
    ro uint32_t type;   /**< MPU Type Register (IMPL DEF) */
    rw uint32_t ctrl;   /**< MPU Control Register */
    rw uint32_t rnr;    /**< MPU Region Number Register */
    rw uint32_t rbar;   /**< MPU Region Base Address Register */
    rw uint32_t rasr;   /**< MPU Region Attribute and Size Register */
    } armv6_mpu_t;

/** The Cortex-M0 PMSAv6 MPU Registers base address

    Reference: ARMv6-M Architecture Reference Manual B3.5.2 */
#define COREM0_MPUR_BASE_ADDRESS    (armv6_mpu_t*)0xE000ED90



/* ------------------------------------------------------------------------ */
/* ARMv6-M Core Definition ------------------------------------------------ */

typedef struct {
    rw uint32_t*    actlr;          /**< Auxiliary control register */
    armv6_syscon_t* armv6_syscon;   /**< System Control and ID registers */
    armv6_systick_t* armv6_systick; /**< System Tick registers */
    armv6_nvic_t* armv6_nvic;       /**< Nested Vectored Interrupt Ctrl */
    armv6_mpu_t* armv6_mpu;         /**< The MPU Registers */
    } armv6_core_t;

/* ------------------------------------------------------------------------ */
/* ARMv6-M Core Instance -------------------------------------------------- */

/* Expose the Cortex M0 core to the HAL */
extern armv6_core_t armv6_core;

#endif
