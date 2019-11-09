/**
    \file lpc_gpio.h
    \brief LPC GPIO Peripheral header support file

    Support for the LPC Cortex-M0 GPIO peripheral block. Most of this work
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


#ifndef LPC_GPIO_H
#define LPC_GPIO_H 1

#include <stdint.h>
#include "hal/lpc/lpc_common.h"

/** A definition of the base address of the GPIO (Port 0) peripheral */
#define LPC_GPIO0_BASE_ADDRESS  ((lpc_gpio_t*)(AHB_BASE + 0x00000))

/** A definition of the base address of the GPIO (Port 1) peripheral */
#define LPC_GPIO1_BASE_ADDRESS  ((lpc_gpio_t*)(AHB_BASE + 0x10000))

/** A definition of the base address of the GPIO (Port 2) peripheral */
#define LPC_GPIO2_BASE_ADDRESS  ((lpc_gpio_t*)(AHB_BASE + 0x20000))

/** A definition of the base address of the GPIO (Port 3) peripheral */
#define LPC_GPIO3_BASE_ADDRESS  ((lpc_gpio_t*)(AHB_BASE + 0x30000))

/** enum for determining a GPIO pin's direction on the LPC processor */
typedef enum {
    LPC_GPIO_INPUT,     /**< GPIO pin direction = input */
    LPC_GPIO_OUTPUT,    /**< GPIO pin direction = output */
    } lpc_gpio_dir_t;


/** enum for determining a GPIO port number */
typedef enum {
    LPC_GPIO_PORT0,     /**< GPIO Port 0 */
    LPC_GPIO_PORT1,     /**< GPIO Port 1 */
    LPC_GPIO_PORT2,     /**< GPIO Port 2 */
    LPC_GPIO_PORT3,     /**< GPIO Port 3 */
    } lpc_gpio_port_t;


/** enum for determining a GPIO port level */
typedef enum {
    LPC_GPIO_LO,        /**< GPIO pin level = low (0v) */
    LPC_GPIO_HI,        /**< GPIO pin level = high */
    } lpc_gpio_level_t;


/** struct representing a GPIO pin */
typedef struct {
    lpc_gpio_port_t port;   /**< GPIO port number */
    uint16_t bit;           /**< GPIO bit number */
    } lpc_gpio_pin_t;


/** A struct describing a GPIO port peripheral, one of these structs exists
    for each GPIO port on the processor */
typedef struct {
    rw uint32_t data[4096]; /**< Data masking register (see user manual!) */
    ro uint32_t reserved[4096];
    rw uint32_t dir;    /**< Port direction register, 1 = output */
    rw uint32_t is;     /**< Interrupt sense register */
    rw uint32_t ibe;    /**< Interrupt both edges register */
    rw uint32_t iev;    /**< Interrupt event register */
    rw uint32_t ie;     /**< Interrupt mask register */
    ro uint32_t ris;    /**< Raw interrupt status register */
    ro uint32_t mis;    /**< Masked interrupt status register */
    wo uint32_t ic;     /**< Interrupt clear register */
    } lpc_gpio_t;


extern void LpcGpioSetDirection(lpc_gpio_pin_t* pin, lpc_gpio_dir_t direction);
extern void LpcGpioSetPin(lpc_gpio_pin_t* pin, lpc_gpio_level_t level);
extern lpc_gpio_level_t LpcGpioGetPin(lpc_gpio_pin_t* pin);


#endif
