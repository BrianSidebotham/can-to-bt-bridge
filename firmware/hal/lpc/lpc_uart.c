/**
    \file lpc_uart.c
    \brief Functions related to the LPC UART peripheral

*/

#include "hal/lpc/lpc11c14.h"

/**
    \fn void UartInit(int port, uart_baud_t baud_rate)
    \brief UART Initialisation function

    Initialises the UART port \a port with the baud rate

*/
void UartInit(int port, int baud_rate)
{
    volatile uint32_t read;

    /* Setup the IO pins required for the UART Rx and Tx lines */
    /* Pin 1.6 is UART RxD with pullup */
    cpu.lpc_ioconfig->pio1_6 = LPC_IO_PULLUP_ENABLED | (1 << 0);

    /* Pin 1.7 is UART TxD */
    cpu.lpc_ioconfig->pio1_7 = LPC_IO_PULLUP_ENABLED | (1 << 0);

    /* Enable the peripheral clock to the UART peripheral - this must
       be done after the pins have been configured (see manual page 22
       Rev 4 - 4th March 2011) */
    cpu.lpc_syscon->sysahbclkctrl |= LPC_SYSAHBCLKCTRL_ENABLE_UART;
    cpu.lpc_syscon->uartclkdiv = 6;

    /* Enable access to the divisor latches */
    cpu.lpc_uart0->lcr = LPC_UART_LCR_8BIT | LPC_UART_LCR_NOPARITY |
                        LPC_UART_LCR_1STOP | LPC_UART_LCR_DLAB;

    /* 115200 baud for 72MHz PCLK */
/*
    cpu.lpc_uart0->dll = 26;
    cpu.lpc_uart0->dlm = 0;
    cpu.lpc_uart0->fdr = (1 << LPC_UART_FDR_DIVADDVAL_BIT) |
                        (4 << LPC_UART_FDR_MULVAL_BIT);
*/
    /* 9600 baud for 72MHz PCLK */
/*
    cpu.lpc_uart0->dlm = (312 >> 8);
    cpu.lpc_uart0->dll = (312 & 0xFF);
    cpu.lpc_uart0->fdr = (1 << LPC_UART_FDR_DIVADDVAL_BIT) |
                        (4 << LPC_UART_FDR_MULVAL_BIT);
*/
    /* 115200 baud for 12MHz PCLK */
/*    cpu.lpc_uart0->dlm = 0;
    cpu.lpc_uart0->dll = 60;
    cpu.lpc_uart0->fdr = (1 << LPC_UART_FDR_DIVADDVAL_BIT) |
                        (4 << LPC_UART_FDR_MULVAL_BIT);
*/
    /* 115200 baud for 12MHz PCLK */
    cpu.lpc_uart0->dlm = 0;
    cpu.lpc_uart0->dll = 4;
    cpu.lpc_uart0->fdr = (5 << LPC_UART_FDR_DIVADDVAL_BIT) |
                        (8 << LPC_UART_FDR_MULVAL_BIT);

    /* Disable access to the divisor latches */
    /* Set to 8, n, 1 */
    cpu.lpc_uart0->lcr = LPC_UART_LCR_8BIT | LPC_UART_LCR_NOPARITY |
                        LPC_UART_LCR_1STOP;

    /* Reset the FIFO's and set the rx FIFO to 1 character */
    cpu.lpc_uart0->fcr = LPC_UART_FCR_FIFOEN |
                        LPC_UART_FCR_RXFIFO_RST |
                        LPC_UART_FCR_TXFIFO_RST |
                        LPC_UART_FCR_TRIGGER_1CHAR;

    /* Clear the line status register by reading */
    read = cpu.lpc_uart0->lsr;

    /* Wait for Tx FIFO to be clear */
    while( (cpu.lpc_uart0->lsr & (LPC_UART_LSR_THRE|LPC_UART_LSR_TEMT)) !=
           (LPC_UART_LSR_THRE|LPC_UART_LSR_TEMT) )
    {
        /* BLANK! */
    }

    /* Wait for receive buffer to be clear */
    while( cpu.lpc_uart0->lsr & LPC_UART_LSR_RDR )
    {
        read = cpu.lpc_uart0->lsr;
        read = cpu.lpc_uart0->rbr;
    }
}

/**
    \fn void uart_putchar(char c)
    \brief Transmits \a c via the UART
    \param c The character to transmit
*/
void uart_putchar(char c)
{

    /* Wait for the transmit holding register to be empty */
    while( !(cpu.lpc_uart0->lsr & LPC_UART_LSR_THRE))
    {
        /* BLANK */
    }

    /* Transmit the character */
    cpu.lpc_uart0->thr = c;
}
