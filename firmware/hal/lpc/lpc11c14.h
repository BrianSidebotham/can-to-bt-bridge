/**
 *
 * LPC-11C14 Header File
 *
 * (c) 2011 Brian Sidebotham
 *
 * Part of the opencar ( http://www.valvers.com/opencar ) project!
 *
 *
 *
 *
 */

#ifndef LPC11C14_H
#define LPC11C14_H 1

/* Fall over if the target architecture is wrong! */
#ifndef __ARM_ARCH_6M__
  #error "The target ARM cpu must be Cortex-M0 compatible (-mcpu=cortex-m0)"
#endif

/* Include the LPC11C14 combination of core + peripherals */
#include "hal/arm/cortex/m0.h"
#include "hal/lpc/lpc_c_can.h"
#include "hal/lpc/lpc_common.h"
#include "hal/lpc/lpc_gpio.h"
#include "hal/lpc/lpc_ioconfig.h"
#include "hal/lpc/lpc_nvic.h"
#include "hal/lpc/lpc_syscon.h"
#include "hal/lpc/lpc_systick.h"
#include "hal/lpc/lpc_timer.h"
#include "hal/lpc/lpc_uart.h"

/* A stack pointer which is setup by the linker script */
extern void* stack_entry;

/** Definitions for io configs for lpc11c14 pins */
#define PIO2_6_FUNC_GPIO            0

#define PIO2_0_FUNC_GPIO            0
#define PIO2_0_FUNC_DTR             1
#define PIO2_0_FUNC_SSEL1           2

#define PIO0_0_FUNC_RESET           0
#define PIO0_0_FUNC_GPIO            1

#define PIO0_1_FUNC_GPIO            0
#define PIO0_1_FUNC_CLKOUT          1
#define PIO0_1_FUNC_CT32B0_MAT2     2

#define PIO1_8_FUNC_GPIO            0
#define PIO1_8_FUNC_CT16B1_CAP0     1

#define PIO0_2_FUNC_GPIO            0
#define PIO0_2_FUNC_SSEL0           1
#define PIO0_2_FUNC_CT16B0_CAP0     2

#define PIO2_7_FUNC_GPIO            0

#define PIO2_8_FUNC_GPIO            0

#define PIO2_1_FUNC_GPIO            0
#define PIO2_1_FUNC_DSR             1
#define PIO2_1_FUNC_SCK1            2

#define PIO0_3_FUNC_GPIO            0

#define PIO0_4_FUNC_GPIO            0
#define PIO0_4_FUNC_SCL             1

#define PIO0_5_FUNC_GPIO            0
#define PIO0_5_FUNC_SDA             1

#define PIO1_9_FUNC_GPIO            0
#define PIO1_9_FUNC_CT16B1_MAT0     1

#define PIO3_4_FUNC_GPIO            0

#define PIO2_4_FUNC_GPIO            0

#define PIO2_5_FUNC_GPIO            0

#define PIO3_5_FUNC_GPIO            0

#define PIO0_6_FUNC_GPIO            0
#define PIO0_6_FUNC_SCK0            2

#define PIO0_7_FUNC_GPIO            0
#define PIO0_7_FUNC_CTS             1

#define PIO2_9_FUNC_GPIO            0

#define PIO2_10_FUNC_GPIO           0

#define PIO2_2_FUNC_GPIO            0
#define PIO2_2_FUNC_DCD             1
#define PIO2_2_FUNC_MISO1           2

#define PIO0_8_FUNC_GPIO            0
#define PIO0_8_FUNC_MISO0           1
#define PIO0_8_FUNC_CT16B0_MAT0     2

#define PIO0_9_FUNC_GPIO            0
#define PIO0_9_FUNC_MOSI0           1
#define PIO0_9_FUNC_CT16B0_MAT1     2

#define PIO0_10_FUNC_SWCLK          0
#define PIO0_10_FUNC_GPIO           1
#define PIO0_10_FUNC_SCK0           2
#define PIO0_10_FUNC_CT16B0_MAT2    3

#define PIO1_10_FUNC_GPIO           0
#define PIO1_10_FUNC_AD6            1
#define PIO1_10_FUNC_CT16B1_MAT1    2

#define PIO2_11_FUNC_GPIO           0
#define PIO2_11_FUNC_SCK0           1

#define PIO0_11_FUNC_GPIO           1
#define PIO0_11_FUNC_AD0            2
#define PIO0_11_FUNC_CT32B0_MAT3    3

#define PIO1_0_FUNC_GPIO            1
#define PIO1_0_FUNC_AD1             2
#define PIO1_0_FUNC_CT32B1_CAP0     3

#define PIO1_1_FUNC_GPIO            1
#define PIO1_1_FUNC_AD2             2
#define PIO1_1_FUNC_CT32B1_MAT0     3

#define PIO1_2_FUNC_GPIO            1
#define PIO1_2_FUNC_AD3             2
#define PIO1_2_FUNC_CT32B1_MAT1     3

#define PIO3_0_FUNC_GPIO            0
#define PIO3_0_FUNC_DTR             1

#define PIO3_1_FUNC_GPIO            0
#define PIO3_1_FUNC_DSR             1

#define PIO2_3_FUNC_GPIO            0
#define PIO2_3_FUNC_RI              1
#define PIO2_3_FUNC_MOSI1           2

#define PIO1_3_FUNC_SWDIO           0
#define PIO1_3_FUNC_GPIO            1
#define PIO1_3_FUNC_AD4             2
#define PIO1_3_FUNC_CT32B1_MAT2     3

#define PIO1_4_FUNC_GPIO            0
#define PIO1_4_FUNC_AD5             1
#define PIO1_4_FUNC_CT32B1_MAT3     2

#define PIO1_11_FUNC_GPIO           0
#define PIO1_11_FUNC_AD7            1

#define PIO3_2_FUNC_GPIO            0
#define PIO3_2_FUNC_DCD             1

#define PIO1_5_FUNC_GPIO            0
#define PIO1_5_FUNC_RTS             1
#define PIO1_5_FUNC_CT32B0_CAP0     2

#define PIO1_6_FUNC_GPIO            0
#define PIO1_6_FUNC_RXD             1
#define PIO1_6_FUNC_CT32B0_MAT0     2

#define PIO1_7_FUNC_GPIO            0
#define PIO1_7_FUNC_TXD             1
#define PIO1_7_FUNC_CT32B0_MAT1     2

#define PIO3_3_FUNC_GPIO            0
#define PIO3_3_FUNC_RI              1

#define PIO_SCK_LOC_P0_10           0
#define PIO_SCK_LOC_P2_11           1
#define PIO_SCK_LOC_P0_6            2

#define PIO_DSR_LOC_P2_1            0
#define PIO_DSR_LOC_P3_1            1

#define PIO_DCD_LOC_P2_2            0
#define PIO_DCD_LOC_P3_2            1

#define PIO_RI_LOC_P2_3             0
#define PIO_RI_LOC_P3_3             1


/** A CPU structure which defines the various memory mapped peripherals and
    registers contained in this cpu variant */
typedef struct {
    armv6_core_t* core;
    lpc_gpio_t* lpc_gpio0;
    lpc_gpio_t* lpc_gpio1;
    lpc_gpio_t* lpc_gpio2;
    lpc_gpio_t* lpc_gpio3;
    lpc_syscon_t* lpc_syscon;
    lpc_ioconfig_t* lpc_ioconfig;
    lpc_c_can_t* lpc_c_can;
    lpc_uart_t* lpc_uart0;
    lpc_timer_t* lpc_tmr16b0;
    lpc_timer_t* lpc_tmr16b1;
    lpc_timer_t* lpc_tmr32b0;
    lpc_timer_t* lpc_tmr32b1;
    } lpc11c14_cpu_t;

/** External declaration of the cpu structure */
extern lpc11c14_cpu_t cpu;

#endif
