
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "hal/hal.h"
#include "circular_buffer.h"

/** CAN message object */
typedef struct {
    uint32_t msg_id;    /**< The message ID */
    uint32_t length;    /**< Length of data receioved */
    uint32_t data[4];   /**< Data array, two bytes per uint32_t */
    } can_msg_obj_t;

/** Circular buffer for CAN received message objects */
circular_buffer_t* can_rx;

typedef struct {
    /* Engine */
    uint32_t eng_rpm;
    uint32_t eng_tps;
    uint32_t eng_water_temp;
    uint32_t eng_air_temp;
    uint32_t eng_MAP;
    uint32_t eng_lambda;
    uint32_t eng_oil_pressure;
    uint32_t eng_fuel_pressure;
    uint32_t eng_oil_temp;
    uint32_t eng_advance;
    uint32_t eng_inj_duration;

    /* Speeds */
    uint32_t kph;
    uint32_t fuel_level;

    /* Chassis */
    uint32_t battery_v;

    /* Lighting */
    uint32_t lights;

    /* Switches */
    uint32_t switches;

    } system_data_t;

/** DTA CAN message object */
can_msg_obj_t can_rx_msg;
system_data_t system_data;

int dta_led_time = 0;
int dta_led_time2 = 0;

lpc_gpio_pin_t led1 = {LPC_GPIO_PORT3, 0};
lpc_gpio_pin_t led2 = {LPC_GPIO_PORT3, 1};
lpc_gpio_pin_t led3 = {LPC_GPIO_PORT3, 2};
lpc_gpio_pin_t led4 = {LPC_GPIO_PORT3, 3};

volatile uint32_t ticker = 0;
volatile uint32_t ticker10ms = 0;

void SysTick_Handler(void)
{
    ticker++;
    ticker10ms++;

    if (dta_led_time)
    {
        dta_led_time--;

        if (dta_led_time == 0)
            LpcGpioSetPin(&led3, LPC_GPIO_HI);
    }

    if (dta_led_time2)
    {
        dta_led_time2--;

        if (dta_led_time2 == 0)
            LpcGpioSetPin(&led4, LPC_GPIO_HI);
    }
}

void CAN_IRQHandler(void)
{
    int i;

    /* Clear any interrupt source in the status register */
    cpu.lpc_c_can->stat &= ~(LPC_CCAN_STAT_BOFF |
                             LPC_CCAN_STAT_EPASS |
                             LPC_CCAN_STAT_EWARN |
                             LPC_CCAN_STAT_RXOK |
                             LPC_CCAN_STAT_TXOK);

    /* Check for messages with interrupts pending */
    for (i=0; i<32; i++)
    {
        if (cpu.lpc_c_can->canint & (1 << i))
        {
            /* Make sure the interface is not busy! */
            while(cpu.lpc_c_can->if2_cmdreq & LPC_CCAN_CMDREQ_BUSY)
            {
                /* BLANK */
            }

            /* Setup the command - reset new data flag, clear the interrupt
               pending bit, read the message object into the if2 registers,
               read databytes 0 through 7 */
            cpu.lpc_c_can->if2_cmdmsk = LPC_CCAN_CMDMSK_DATA_A |
                                        LPC_CCAN_CMDMSK_DATA_B |
                                        LPC_CCAN_CMDMSK_NEWDAT |
                                        LPC_CCAN_CMDMSK_ARB |
                                        LPC_CCAN_CMDMSK_CTRL |
                                        LPC_CCAN_CMDMSK_CLRINTPND |
                                        LPC_CCAN_CMDMSK_RD;

            /* Transfer message i + 1 (Register is 1-based and not 0-based) */
            cpu.lpc_c_can->if2_cmdreq = i + 1;

            /* Wait until the transfer is complete */
            while(cpu.lpc_c_can->if2_cmdreq & LPC_CCAN_CMDREQ_BUSY)
            {
                /* BLANK! */
            }

            /* Get the message identifier */
            can_rx_msg.msg_id = (cpu.lpc_c_can->if2_arb2 &
                                 LPC_CCAN_ARB2_IDMSK);

            /* Extended identifier or standard ? */
            if (cpu.lpc_c_can->if2_arb2 & LPC_CCAN_ARB2_XTD)
            {
                /* 29-bit extended identifier */
                can_rx_msg.msg_id <<= 16;
                can_rx_msg.msg_id |= cpu.lpc_c_can->if2_arb1;
            }
            else
            {
                /* 11-bit standard identifier */
                can_rx_msg.msg_id >>= 2;
            }

            /* Transfer data now... */
            can_rx_msg.data[0] = cpu.lpc_c_can->if2_da1;
            can_rx_msg.data[1] = cpu.lpc_c_can->if2_da2;
            can_rx_msg.data[2] = cpu.lpc_c_can->if2_db1;
            can_rx_msg.data[3] = cpu.lpc_c_can->if2_db2;
            can_rx_msg.length = cpu.lpc_c_can->if2_mctrl & LPC_CCAN_MCTRL_DLC;

            /* Copy the data to the buffer */
            Cbu_put(can_rx, &can_rx_msg);
        }
    }
}

/**
    \fn void uart_putchar(char c)
    \brief Transmits \a c via the UART
    \param c The character to transmit
*/
void uart_pc(char c)
{
    /* Wait for the transmit holding register to be empty */
    while( !(cpu.lpc_uart0->lsr & LPC_UART_LSR_THRE))
    {
        /* BLANK */
    }

    /* Transmit the character */
    cpu.lpc_uart0->thr = c;
}

int main(void)
{
    can_msg_obj_t rx;

    can_rx = Cbu_create(10, sizeof(can_msg_obj_t));

    /* Initialise the CPU */
    CpuInit();
    UartInit(0, 115200);

    LpcGpioSetDirection(&led1, LPC_GPIO_OUTPUT);
    LpcGpioSetDirection(&led2, LPC_GPIO_OUTPUT);
    LpcGpioSetDirection(&led3, LPC_GPIO_OUTPUT);
    LpcGpioSetDirection(&led4, LPC_GPIO_OUTPUT);

    /* Both LEDs start off */
    LpcGpioSetPin(&led1, LPC_GPIO_HI);
    LpcGpioSetPin(&led2, LPC_GPIO_HI);
    LpcGpioSetPin(&led3, LPC_GPIO_HI);
    LpcGpioSetPin(&led4, LPC_GPIO_HI);

    /* Enable the System timer exception (handler's above) */
    EnableSysTickException();

    /* Initialise the CAN peripheral @ 1Mb */
    LPCCcanInit();

    /* Show that there was an error with the buffer creation! */
    if (can_rx == NULL)
        LpcGpioSetPin(&led2, LPC_GPIO_LO);

    while(1)
    {
        if (can_rx->data_available)
        {
            if (Cbu_get(can_rx, &rx))
            {
                if (rx.msg_id == 0x2000)
                {
                    /* Data in this packet */
                    system_data.eng_rpm = rx.data[0];
                    system_data.eng_tps = rx.data[1];
                    system_data.eng_water_temp = rx.data[2];
                    system_data.eng_air_temp = rx.data[3];

                    dta_led_time = 5;
                    LpcGpioSetPin(&led3, LPC_GPIO_LO);
                }
                else if (rx.msg_id == 0x2001)
                {
                    system_data.eng_MAP = rx.data[0];
                    system_data.eng_lambda = rx.data[1];
                    system_data.kph = rx.data[2];
                    system_data.eng_oil_pressure = rx.data[3];

                    dta_led_time2 = 5;
                    LpcGpioSetPin(&led3, LPC_GPIO_LO);
                }
                else if (rx.msg_id == 0x2002)
                {
                    system_data.eng_fuel_pressure = rx.data[0];
                    system_data.eng_oil_temp = rx.data[1];
                    system_data.battery_v = rx.data[2];
                    system_data.eng_inj_duration = rx.data[3];

                    dta_led_time2 = 5;
                    LpcGpioSetPin(&led3, LPC_GPIO_LO);
                }
                else if (rx.msg_id == 0x2003)
                {
                    dta_led_time2 = 5;
                    LpcGpioSetPin(&led3, LPC_GPIO_LO);
                }
            }
        }

        if (ticker < 10)
            LpcGpioSetPin(&led1, LPC_GPIO_LO);
        else
            LpcGpioSetPin(&led1, LPC_GPIO_HI);

        /* Transmit every 50ms */
        if (ticker10ms > 5)
        {
            uart_putchar(0xFE);
            uart_putchar(0xFE);
            uart_putchar(0xFE);
            uart_putchar(0xFE);
            uart_putchar(0xFE);
            uart_putchar(0xFE);

            uart_putchar(system_data.eng_rpm >> 8);
            uart_putchar(system_data.eng_rpm & 0xFF);

            uart_putchar(system_data.battery_v >> 8);
            uart_putchar(system_data.battery_v & 0xFF);

            uart_putchar(system_data.kph >> 8);
            uart_putchar(system_data.kph & 0xFF);

            uart_putchar(system_data.eng_fuel_pressure >> 8);
            uart_putchar(system_data.eng_fuel_pressure & 0xFF);

            uart_putchar(system_data.eng_oil_pressure >> 8);
            uart_putchar(system_data.eng_oil_pressure & 0xFF);

            uart_putchar(system_data.eng_air_temp >> 8);
            uart_putchar(system_data.eng_air_temp & 0xFF);

            uart_putchar(system_data.eng_oil_temp >> 8);
            uart_putchar(system_data.eng_oil_temp & 0xFF);

            uart_putchar(system_data.eng_water_temp >> 8);
            uart_putchar(system_data.eng_water_temp & 0xFF);

            /* Stick a character on the end of the packet to try and ensure a bit
               of integrity! */
            uart_putchar(0x55);

            /* Reset the 10ms ticker timer */
            ticker10ms = 0;
        }

        if (ticker > 100)
        {
            ticker = 0;
        }
    }

    return 0;
}

