/**
    \file lpc_gpio.c
    \brief Hardware Abstraction layer for LPC GPIO Module

    (c)2011 Brian Sidebotham

    Part of the opencar ( http://www.valvers.com/opencar ) project

*/

#include "hal/lpc/lpc11c14.h"

/**
    \fn void LpcGpioSetDirection(lpc_gpio_pin_t* pin, lpc_gpio_dir_t direction)
    \brief Set the direction of a GPIO port pin
    \param pin The struct which describes the port GPIO pin
    \param direction The direction to set the port pin to

*/
void LpcGpioSetDirection(lpc_gpio_pin_t* pin, lpc_gpio_dir_t direction)
{
    /* There are only 12 bits per port, and only four ports! */
    if ((pin->bit > 11) || (pin->port > 3))
        return;

    switch (pin->port)
    {
        case LPC_GPIO_PORT0:
            if (direction == LPC_GPIO_OUTPUT)
                cpu.lpc_gpio0->dir |= (1 << pin->bit);
            else
                cpu.lpc_gpio0->dir &= ~(1 << pin->bit);
        break;

        case LPC_GPIO_PORT1:
            if (direction == LPC_GPIO_OUTPUT)
                cpu.lpc_gpio1->dir |= (1 << pin->bit);
            else
                cpu.lpc_gpio1->dir &= ~(1 << pin->bit);
        break;

        case LPC_GPIO_PORT2:
            if (direction == LPC_GPIO_OUTPUT)
                cpu.lpc_gpio2->dir |= (1 << pin->bit);
            else
                cpu.lpc_gpio2->dir &= ~(1 << pin->bit);
        break;

        case LPC_GPIO_PORT3:
            if (direction == LPC_GPIO_OUTPUT)
                cpu.lpc_gpio3->dir |= (1 << pin->bit);
            else
                cpu.lpc_gpio3->dir &= ~(1 << pin->bit);
        break;
    }
}

/**
    \fn void LpcGpioSetPin(lpc_gpio_pin_t* pin, lpc_gpio_level_t level)
    \brief Sets the level of a GPIO output pin
    \param pin The struct which  which contains the GPIO pin
    \param level The level to set the GPIO port pin to
*/
void LpcGpioSetPin(lpc_gpio_pin_t* pin, lpc_gpio_level_t level)
{
    /* There are only four ports, and twelve bits per port! */
    if ((pin->bit > 11) || (pin->port > 3))
        return;

    switch(pin->port)
    {
        case LPC_GPIO_PORT0:
            if (level == LPC_GPIO_HI)
                cpu.lpc_gpio0->data[4095] |= (1<<pin->bit);
            else
                cpu.lpc_gpio0->data[4095] &= ~(1<<pin->bit);
        break;

        case LPC_GPIO_PORT1:
            if (level == LPC_GPIO_HI)
                cpu.lpc_gpio1->data[4095] |= (1<<pin->bit);
            else
                cpu.lpc_gpio1->data[4095] &= ~(1<<pin->bit);
        break;

        case LPC_GPIO_PORT2:
            if (level == LPC_GPIO_HI)
                cpu.lpc_gpio2->data[4095] |= (1<<pin->bit);
            else
                cpu.lpc_gpio2->data[4095] &= ~(1<<pin->bit);
        break;

        case LPC_GPIO_PORT3:
            if (level == LPC_GPIO_HI)
                cpu.lpc_gpio3->data[4095] |= (1<<pin->bit);
            else
                cpu.lpc_gpio3->data[4095] &= ~(1<<pin->bit);
        break;
    }
}


/**
    \fn lpc_gpio_level_t LpcGpioGetPin(lpc_gpio_pin_t* pin)
    \param pin The pin to read
    \brief Reads the level of \a pin and returns a logic level
*/
lpc_gpio_level_t LpcGpioGetPin(lpc_gpio_pin_t* pin)
{
    lpc_gpio_level_t result = LPC_GPIO_LO;

    switch(pin->port)
    {
        case LPC_GPIO_PORT0:
            if (cpu.lpc_gpio0->data[4095] & (1 << pin->bit))
                result = LPC_GPIO_HI;
        break;

        case LPC_GPIO_PORT1:
            if (cpu.lpc_gpio1->data[4095] & (1 << pin->bit))
                result = LPC_GPIO_HI;
        break;

        case LPC_GPIO_PORT2:
            if (cpu.lpc_gpio2->data[4095] & (1 << pin->bit))
                result = LPC_GPIO_HI;
        break;

        case LPC_GPIO_PORT3:
            if (cpu.lpc_gpio3->data[4095] & (1 << pin->bit))
                result = LPC_GPIO_HI;
        break;
    }

    return result;
}
