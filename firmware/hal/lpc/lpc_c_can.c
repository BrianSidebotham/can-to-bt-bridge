/**
    \file lpc_c_can.c
    \brief Functions related to the C_CAN peripheral

*/

#include "hal/lpc/lpc11c14.h"


/**
    \fn void LPCCcanInit(void)
    \brief CCAN initialisation function

    Initialises the CCAN peripheral. Make sure the Pin IO config has been set
    to enable the CAN pins
*/
void LPCCcanInit(void)
{
    /* Enable the peripheral clock for the CAN module */
    cpu.lpc_syscon->sysahbclkctrl |= LPC_SYSAHBCLKCTRL_ENABLE_CAN;

    /* As per the LPC11C14 User Manual - write one to this register to
        de-assert the CAN module reset */
    cpu.lpc_syscon->presetctrl |= LPC_PRESETCTRL_CAN_RST_N;

    /* Start the initialisation process */
    cpu.lpc_c_can->cntl |= LPC_CCAN_CNTL_INIT;

    /* Divide the peripheral clock (72MHz) by 9 to get 8MHz the CCAN clock */
    cpu.lpc_c_can->clkdiv = LPC_CCAN_CLKDIV_9;

    /* Enable access to the bit timing register */
    cpu.lpc_c_can->cntl |= LPC_CCAN_CNTL_CCE;

    /* Set the bit timing register for 1Mbit operation */
    cpu.lpc_c_can->bt = 0x1400;
    cpu.lpc_c_can->brpe = 0x0000;

    /* Enable access to the bit timing register */
    cpu.lpc_c_can->cntl &= ~LPC_CCAN_CNTL_CCE;

    /* Setup the message objects to receive stuff (Message is valid!) */
    cpu.lpc_c_can->if1_cmdmsk = (1<<7) | (1<<6) | (1<<5) | (1<<4);
    cpu.lpc_c_can->if1_msk1 = 0xFFFF;
    cpu.lpc_c_can->if1_msk2 = (1<<15) | 0x1FFF;
    cpu.lpc_c_can->if1_arb1 = (0x2000 & 0xFFFF);
    cpu.lpc_c_can->if1_arb2 = (1<<15) | (1<<14) | (0x2000 >> 16);
    cpu.lpc_c_can->if1_mctrl = (1<<7) | (1<<10) | (1<<12) | 8;
    cpu.lpc_c_can->if1_cmdreq = 1;

    /* Wait until the transfer is done! */
    while (cpu.lpc_c_can->if1_cmdreq & (1<<15))
    {
        /* BLANK! */
    }

    /* Setup the message objects to receive stuff (Message is valid!) */
    cpu.lpc_c_can->if1_cmdmsk = (1<<7) | (1<<6) | (1<<5) | (1<<4);
    cpu.lpc_c_can->if1_msk1 = 0xFFFF;
    cpu.lpc_c_can->if1_msk2 = (1<<15) | 0x1FFF;
    cpu.lpc_c_can->if1_arb1 = (0x2002 & 0xFFFF);
    cpu.lpc_c_can->if1_arb2 = (1<<15) | (1<<14) | (0x2002 >> 16);
    cpu.lpc_c_can->if1_mctrl = (1<<7) | (1<<10) | (1<<12) | 8;
    cpu.lpc_c_can->if1_cmdreq = 2;

    /* Wait until the transfer is done! */
    while (cpu.lpc_c_can->if1_cmdreq & (1<<15))
    {
        /* BLANK! */
    }
#if(0)
    /* Setup the message objects to receive stuff (Message is valid!) */
    cpu.lpc_c_can->if1_cmdmsk = (1<<7) | (1<<6) | (1<<5) | (1<<4);
    cpu.lpc_c_can->if1_msk1 = 0xFFFF;
    cpu.lpc_c_can->if1_msk2 = (1<<15) | 0x1FFF;
    cpu.lpc_c_can->if1_arb1 = (0x2002 & 0xFFFF);
    cpu.lpc_c_can->if1_arb2 = (1<<15) | (1<<14) | (0x2002 >> 16);
    cpu.lpc_c_can->if1_mctrl = (1<<7) | (1<<10) | (1<<12) | 8;
    cpu.lpc_c_can->if1_cmdreq = 3;

    /* Wait until the transfer is done! */
    while (cpu.lpc_c_can->if1_cmdreq & (1<<15))
    {
        /* BLANK! */
    }

    /* Setup the message objects to receive stuff (Message is valid!) */
    cpu.lpc_c_can->if1_cmdmsk = (1<<7) | (1<<6) | (1<<5) | (1<<4);
    cpu.lpc_c_can->if1_msk1 = 0xFFFF;
    cpu.lpc_c_can->if1_msk2 = (1<<15) | 0x1FFF;
    cpu.lpc_c_can->if1_arb1 = (0x2003 & 0xFFFF);
    cpu.lpc_c_can->if1_arb2 = (1<<15) | (1<<14) | (0x2003 >> 16);
    cpu.lpc_c_can->if1_mctrl = (1<<7) | (1<<10) | (1<<12) | 8;
    cpu.lpc_c_can->if1_cmdreq = 4;
#endif
    /* Wait until the transfer is done! */
    while (cpu.lpc_c_can->if1_cmdreq & (1<<15))
    {
        /* BLANK! */
    }

    /* Finished setting up the */
    /* Finished configuring the CAN peripheral */
    cpu.lpc_c_can->cntl &= ~LPC_CCAN_CNTL_INIT;

    /* Wait here until this bit gets updated */
    while (cpu.lpc_c_can->cntl & LPC_CCAN_CNTL_INIT)
    {
        /* BLANK! */
    }

    /* Setup all the CAN interrupts */
    cpu.lpc_c_can->cntl |= (LPC_CCAN_CNTL_IE |
                            LPC_CCAN_CNTL_SIE |
                            LPC_CCAN_CNTL_EIE);

    /* Enable the CAN interrupt via the NVIC */
    M0IntEnable(IV_C_CAN);
}
