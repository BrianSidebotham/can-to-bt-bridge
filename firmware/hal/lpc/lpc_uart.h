/**
    \file lpc_uart.h
    \brief Functions related to the LPC UART peripheral

*/

#ifndef LPC_UART_H
#define LPC_UART_H 1

#include <stdint.h>
#include "hal/lpc/lpc_common.h"

/* Line Control Register -------------------------------------------------- */

#define LPC_UART_LCR_5BIT       0
#define LPC_UART_LCR_6BIT       1
#define LPC_UART_LCR_7BIT       2
#define LPC_UART_LCR_8BIT       3
#define LPC_UART_LCR_1STOP      (0 << 2)
#define LPC_UART_LCR_2STOP      (1 << 2)
#define LPC_UART_LCR_NOPARITY   (0 << 3)
#define LPC_UART_LCR_PARITY     (1 << 3)
#define LPC_UART_LCR_ODDPAR     (0 << 4)
#define LPC_UART_LCR_EVENPAR    (1 << 4)
#define LPC_UART_LCR_1PAR       (2 << 4)
#define LPC_UART_LCR_0PAR       (3 << 4)
#define LPC_UART_LCR_BREAK      (1 << 6)

/** Divisor latch access bit */
#define LPC_UART_LCR_DLAB       (1 << 7)


/* Modem Control Register ------------------------------------------------- */

/** Loopback Mode Select, 1 = enable modem loopback mode */
#define LPC_UART_MCR_LMS        (1 << 4)

/** RTS Flow Control Enable */
#define LPC_UART_MCR_RTSEN      (1 << 6)

/** CTS Flow Control Enable */
#define LPC_UART_MCR_CTSEN      (1 << 7)


/* Line Status Register --------------------------------------------------- */

/** Receiver Data Ready */
#define LPC_UART_LSR_RDR        (1 << 0)

/** Overrun Error */
#define LPC_UART_LSR_OE         (1 << 1)

/** Parity Error */
#define LPC_UART_LSR_PE         (1 << 2)

/** Framing Error */
#define LPC_UART_LSR_FE         (1 << 3)

/** Break Interrupt */
#define LPC_UART_LSR_BI         (1 << 4)

/** Transmitter Holding Register Empty */
#define LPC_UART_LSR_THRE       (1 << 5)

/** Transmitter Empty */
#define LPC_UART_LSR_TEMT       (1 << 6)

/** RX FIFO Error - set when any character in the FIFO had an error */
#define LPC_UART_LSR_RXFE       (1 << 7)


/* Modem Status Register -------------------------------------------------- */

#define LPC_UART_MCR_DCTS       (1 << 0)
#define LPC_UART_MCR_DDSR       (1 << 1)
#define LPC_UART_MCR_TERI       (1 << 2)
#define LPC_UART_MCR_DDCD       (1 << 3)
#define LPC_UART_MCR_CTS        (1 << 4)
#define LPC_UART_MCR_DSR        (1 << 5)
#define LPC_UART_MCR_RI         (1 << 6)
#define LPC_UART_MCR_DCD        (1 << 7)


/* Modem Control Register ------------------------------------------------- */

#define LPC_UART_FCR_FIFOEN     (1 << 0)
#define LPC_UART_FCR_RXFIFO_RST (1 << 1)
#define LPC_UART_FCR_TXFIFO_RST (1 << 2)
#define LPC_UART_FCR_TRIGGER_BIT    (1 << 6)
#define LPC_UART_FCR_TRIGGER_1CHAR (0 << 6)
#define LPC_UART_FCR_TRIGGER_4CHAR (1 << 6)
#define LPC_UART_FCR_TRIGGER_8CHAR (2 << 6)
#define LPC_UART_FCR_TRIGGER_14CHAR (3 << 6)


/* Auto-Baud Control Register --------------------------------------------- */

#define LPC_UART_ACR_START      (1 << 0)
#define LPC_UART_ACR_MODE       (1 << 1)
#define LPC_UART_ACR_AUTORES    (1 << 2)
#define LPC_UART_ACR_ABEO       (1 << 8)
#define LPC_UART_ACR_ABTO       (1 << 9)

/* Fractional Divisor Register -------------------------------------------- */

#define LPC_UART_FDR_DIVADDVAL_BIT  0
#define LPC_UART_FDR_MULVAL_BIT     4


/* Transmit Enable Register ----------------------------------------------- */

#define LPC_UART_TER_TXEN           (1 << 7)

/** A definition of the base address of the c_can peripheral */
#define LPC_UART_BASE_ADDRESS   ((lpc_uart_t*)(APB_BASE + 0x8000))

typedef struct {
    union {
        ro uint32_t rbr;        /**< Receive buffer register */
        wo uint32_t thr;        /**< Transmit holding register */
        rw uint32_t dll;        /**< Divisor latch LSB */
    };

    union {
        rw uint32_t dlm;        /**< Divisor Latch MSB register */
        rw uint32_t ier;        /**< Interrupt Enable register */
    };

    union {
        ro uint32_t iir;        /**< Interrupt ID register */
        wo uint32_t fcr;        /**< FIFO control register */
    };

    rw uint32_t lcr;            /**< Line control register */
    rw uint32_t mcr;            /**< Modem control register */
    ro uint32_t lsr;            /**< Line status register */
    ro uint32_t msr;            /**< Modem status register */
    rw uint32_t scr;            /**< Scratch Pad register */
    rw uint32_t acr;            /**< Auto-baud control register */
    ro uint32_t reserved1;
    rw uint32_t fdr;            /**< Fractional divider register */
    ro uint32_t reserved2;
    rw uint32_t ter;            /**< Transmit enable register */
    rw uint32_t reserved3[6];
    rw uint32_t rs485ctrl;      /**< RS-485 Control register */
    rw uint32_t rs485adrmatch;  /**< RS-485 Address Match Register */
    rw uint32_t rs485delay;     /**< RS-485 Direction Delay ctrl register */
    } lpc_uart_t;

extern void UartInit(int port, int baud_rate);
extern void uart_putchar(char c);

#endif
