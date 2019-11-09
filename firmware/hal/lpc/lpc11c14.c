/**
    \file lpc11c14.c
    \brief Hardware Abstraction layer for LPC11C14

    (c)2011 Brian Sidebotham

    Part of the opencar ( http://www.valvers.com/opencar ) project

*/

#include "hal/lpc/lpc11c14.h"

/*! Read-only data section - initialised data source */
extern unsigned long _etext;
/*! Start of initialised data section (SRAM) */
extern unsigned long _data;
/*! End of initialised data section (SRAM) */
extern unsigned long _edata;
/*! Start of unitialised data (set to 0) (SRAM) */
extern unsigned long _bss;
/*! End of unitialised data (set to 0) (SRAM) */
extern unsigned long _ebss;

/*! Main application entry */
extern int main(void);

/** LPC11C14 cpu structure */
lpc11c14_cpu_t cpu = {
    .core = &armv6_core,
    .lpc_gpio0 = LPC_GPIO0_BASE_ADDRESS,
    .lpc_gpio1 = LPC_GPIO1_BASE_ADDRESS,
    .lpc_gpio2 = LPC_GPIO2_BASE_ADDRESS,
    .lpc_gpio3 = LPC_GPIO3_BASE_ADDRESS,
    .lpc_syscon = LPC_SYSCON_BASE_ADDRESS,
    .lpc_c_can = LPC_CCAN_BASE_ADDRESS,
    .lpc_uart0 = LPC_UART_BASE_ADDRESS,
    .lpc_ioconfig = LPC_IOCONFIG_BASE_ADDRESS,
    };

/**
    \fn void Delayms(uint32_t time)
    \brief Delay for time ms (with resolution of 10ms +10 -0 ms)
    \param time The time in ms to delay (minimum will be 10ms)

*/
void Delayms(uint32_t time)
{
    volatile uint32_t count = 0;

    /* The resolution is 10ms */
//    time = time >> 3;
    time = time / 10;

//    time /= 10;

    /* As there is no way to know that the first read will ocurr just before
       a sys clock tick, we *must* add in an additional tick. This creates
       an error which is most significant for small delays. It does however,
       ensure that we delay a minimum of 10ms */
    time += 1;

    /* First read of this register clears the countflag bit */
    count = cpu.core->armv6_systick->csr;
    count = 0;

    if (time > 51)
        time = 51;

    while(count < time)
    {
        if (cpu.core->armv6_systick->csr & COREM0_SYSTCSR_COUNTFLAG)
        {
            count++;
        }
    }
}

void CpuInit(void)
{
    volatile int scratch;

    /* Power up the system osc - notice that the bits in this register are
       active low!
       Also, we must write certain bits as */
    cpu.lpc_syscon->pdruncfg &= (~LPC_PDRUNCFG_SYSOSC_PD) & LPC_PDRUNCFG_MUSTSET;

    /* Crystal is 12MHz so set the range to 1-20MHz + Bypass is disabled */
    cpu.lpc_syscon->sysoscctrl = LPC_SYSOSCCTRL_FRANGE_1_20MHZ;

    /* Delay for a period to let things settle */
    for(scratch = 0; scratch < 500; scratch++)
        ;

    /* Select the system pll clock source */
    cpu.lpc_syscon->syspllclksel = LPC_SYSPLLCLKSEL_SYSOSC;

    /* Update the system pll clock source
       NOTE: Updating the pll clock source ocurrs on the rising edge of the
       update register */
    cpu.lpc_syscon->syspllclkuen = 0;
    cpu.lpc_syscon->syspllclkuen = LPC_SYSPLLCLKUEN_ENABLE;

    /* Set the PLL output frequency by selecting a multiplier and a post
       divider:
       (12MHz * 6) / 1 = 48MHz */
    cpu.lpc_syscon->syspllctrl = ( LPC_SYSPLLCTRL_FEEDBACKDIV_6 |
                                   LPC_SYSPLLCTRL_POSTDIV_1 );

    /* Power the system clock PLL */
    cpu.lpc_syscon->pdruncfg &= (~LPC_PDRUNCFG_SYSPLL_PD) & LPC_PDRUNCFG_MUSTSET;

    /* Wait for the PLL to lock */
    while( !(cpu.lpc_syscon->syspllstat & LPC_SYSPLLSTAT_LOCKED))
    {
        /* BLANK */
    }

    /* Set the main clock source as the PLL output */
    cpu.lpc_syscon->mainclksel = LPC_MAINCLKSEL_SYSPLL_OUT;

    /* Update the main clock source
       NOTE: Updating the clock source ocurrs on the rising edge of the
       update register */
    cpu.lpc_syscon->mainclkuen = 0;
    cpu.lpc_syscon->mainclkuen = LPC_MAINCLKUEN_ENABLE;

    /* Set the AHB clock divisor to 1 (run at 72MHz) */
    cpu.lpc_syscon->sysahbclkdiv = 1;

    /* Enable the IOCON AHB clock connection to enable peripherals */
    cpu.lpc_syscon->sysahbclkctrl |= LPC_SYSAHBCLKCTRL_ENABLE_IOCON;

    /* Setup SysTick to a 10ms ticker and enable */
    scratch = cpu.core->armv6_systick->calib;
    cpu.core->armv6_systick->rvr = 360000;
    cpu.core->armv6_systick->cvr = 0;
    cpu.core->armv6_systick->csr |= COREM0_SYSTCSR_ENABLE | COREM0_SYSTCSR_CLKSOURCE;

}


/**
    \fn void crt0(void)
    \brief C Run-Time 0 - Entry for application after reset
*/
void crt0(void)
{
    uint32_t* etext = &_etext;
    uint32_t* data = &_data;

    /* Loop through initialised data, and copy the read-only copy to SRAM */
    while(data < &_edata)
    {
        *data++ = *etext++;
    }

    /* BSS is initialised to zero */
    data = &_bss;
    while(data < &_ebss)
    {
        *data++ = 0;
    }

    /* Jump to main */
    main();

    /* Stick after exiting from main */
    while(1)
    {
        /* BLANK */
    }
}

// Declare a weak alias macro as described in the GCC manual[1][2]
#define WEAK_ALIAS(f) __attribute__ ((weak, alias (#f)));
#define SECTION(s) __attribute__ ((section(s)))

/******************************************************************************
 * Forward undefined IRQ handlers to an infinite loop function. The Handlers
 * are weakly aliased which means that (re)definitions will overide these.
 *****************************************************************************/

void irq_undefined() {
  //while(1);
}

void CAN_IRQHandler(void)       WEAK_ALIAS(irq_undefined);
void SSP1_IRQHandler(void)      WEAK_ALIAS(irq_undefined);
void I2C_IRQHandler(void)       WEAK_ALIAS(irq_undefined);
void TIMER16_0_IRQHandler(void) WEAK_ALIAS(irq_undefined);
void TIMER16_1_IRQHandler(void) WEAK_ALIAS(irq_undefined);
void TIMER32_0_IRQHandler(void) WEAK_ALIAS(irq_undefined);
void TIMER32_1_IRQHandler(void) WEAK_ALIAS(irq_undefined);
void SSP0_IRQHandler(void)      WEAK_ALIAS(irq_undefined);
void UART_IRQHandler(void)      WEAK_ALIAS(irq_undefined);
void USB_IRQHandler(void)       WEAK_ALIAS(irq_undefined);
void USB_FIQHandler(void)       WEAK_ALIAS(irq_undefined);
void ADC_IRQHandler(void)       WEAK_ALIAS(irq_undefined);
void WDT_IRQHandler(void)       WEAK_ALIAS(irq_undefined);
void BOD_IRQHandler(void)       WEAK_ALIAS(irq_undefined);
void FMC_IRQHandler(void)       WEAK_ALIAS(irq_undefined);
void PIOINT3_IRQHandler(void)   WEAK_ALIAS(irq_undefined);
void PIOINT2_IRQHandler(void)   WEAK_ALIAS(irq_undefined);
void PIOINT1_IRQHandler(void)   WEAK_ALIAS(irq_undefined);
void PIOINT0_IRQHandler(void)   WEAK_ALIAS(irq_undefined);
void WAKEUP_IRQHandler(void)    WEAK_ALIAS(irq_undefined);

/*****************************************************************************
 * Forward undefined fault handlers to an infinite loop function. The Handlers
 * are weakly aliased which means that (re)definitions will overide these.
 ****************************************************************************/

void fault_undefined() {
  // Do nothing when occured interrupt is not defined, just keep looping
  while(1);
}

void NMI_Handler(void)          WEAK_ALIAS(fault_undefined);
void HardFault_Handler(void)    WEAK_ALIAS(fault_undefined);
void MemManage_Handler(void)    WEAK_ALIAS(fault_undefined);
void BusFault_Handler(void)     WEAK_ALIAS(fault_undefined);
void UsageFault_Handler(void)   WEAK_ALIAS(fault_undefined);
void SVCall_Handler(void)       WEAK_ALIAS(fault_undefined);
void DebugMon_Handler(void)     WEAK_ALIAS(fault_undefined);
void PendSV_Handler(void)       WEAK_ALIAS(fault_undefined);
void SysTick_Handler(void)      WEAK_ALIAS(fault_undefined);

/******************************************************************************
 * Forward undefined IRQ handlers to an infinite loop function. The Handlers
 * are weakly aliased which means that (re)definitions will overide these.
 *****************************************************************************/

// Defined irq vectors using simple c code following the description in a white
// paper from ARM[3] and code example from Simonsson Fun Technologies[4].
// These vectors are placed at the memory location defined in the linker script
const void *vectors[] SECTION(".irq_vectors") =
{
  // Stack and program reset entry point
  &stack_entry,          // The initial stack pointer
  crt0,                  // The reset handler

  // Various fault handlers
  NMI_Handler,           // The NMI handler
  HardFault_Handler,     // The hard fault handler
  MemManage_Handler,     // MemManage_Handler
  BusFault_Handler,      // BusFault_Handler
  UsageFault_Handler,    // UsageFault_Handler
  0,                     // Reserved
  0,                     // Reserved
  0,                     // Reserved
  0,                     // Reserved
  SVCall_Handler,        // SVCall handler
  DebugMon_Handler,      // DebugMon_Handler
  0,                     // Reserved
  PendSV_Handler,        // The PendSV handler
  SysTick_Handler,       // The SysTick handler

  // Wakeup I/O pins handlers
  WAKEUP_IRQHandler,     // PIO0_0  Wakeup
  WAKEUP_IRQHandler,     // PIO0_1  Wakeup
  WAKEUP_IRQHandler,     // PIO0_2  Wakeup
  WAKEUP_IRQHandler,     // PIO0_3  Wakeup
  WAKEUP_IRQHandler,     // PIO0_4  Wakeup
  WAKEUP_IRQHandler,     // PIO0_5  Wakeup
  WAKEUP_IRQHandler,     // PIO0_6  Wakeup
  WAKEUP_IRQHandler,     // PIO0_7  Wakeup
  WAKEUP_IRQHandler,     // PIO0_8  Wakeup
  WAKEUP_IRQHandler,     // PIO0_9  Wakeup
  WAKEUP_IRQHandler,     // PIO0_10 Wakeup
  WAKEUP_IRQHandler,     // PIO0_11 Wakeup
  WAKEUP_IRQHandler,     // PIO1_0  Wakeup

  // Specific peripheral irq handlers
  CAN_IRQHandler,        // CAN
  SSP1_IRQHandler,       // SSP1
  I2C_IRQHandler,        // I2C0
  TIMER16_0_IRQHandler,  // CT16B0 (16-bit Timer 0)
  TIMER16_1_IRQHandler,  // CT16B1 (16-bit Timer 1)
  TIMER32_0_IRQHandler,  // CT32B0 (32-bit Timer 0)
  TIMER32_1_IRQHandler,  // CT32B1 (32-bit Timer 1)
  SSP0_IRQHandler,       // SSP0
  UART_IRQHandler,       // UART0
  USB_IRQHandler,        // USB IRQ
  USB_FIQHandler,        // USB FIQ
  ADC_IRQHandler,        // ADC (A/D Converter)
  WDT_IRQHandler,        // WDT (Watchdog Timer)
  BOD_IRQHandler,        // BOD (Brownout Detect)
  FMC_IRQHandler,        // Flash (IP2111 Flash Memory Controller)
  PIOINT3_IRQHandler,    // PIO INT3
  PIOINT2_IRQHandler,    // PIO INT2
  PIOINT1_IRQHandler,    // PIO INT1
  PIOINT0_IRQHandler,    // PIO INT0
};
