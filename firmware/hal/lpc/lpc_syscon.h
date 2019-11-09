/**
 *
 * LPC System control register definitions
 *
 * (c) 2011 Brian Sidebotham
 *
 * Part of the opencar ( http://www.valvers.com/opencar ) project!
 *
 *
 *
 *
 */

#ifndef LPC_SYSCON_H
#define LPC_SYSCON_H 1

#include "hal/lpc/lpc_common.h"

/** A definition of the base address of the system control peripheral */
#define LPC_SYSCON_BASE_ADDRESS ((lpc_syscon_t*)(APB_BASE + 0x48000))

/** LPC System Control Peripheral */
typedef struct {
    rw uint32_t sysmemremap;
    rw uint32_t presetctrl;
    rw uint32_t syspllctrl;
    ro uint32_t syspllstat;
    ro uint32_t reserved0[4];
    rw uint32_t sysoscctrl;
    rw uint32_t wdtoscctrl;
    rw uint32_t ircctrl;
    rw uint32_t reserved1;
    ro uint32_t sysrststat;
    ro uint32_t reserved2[3];
    rw uint32_t syspllclksel;
    rw uint32_t syspllclkuen;
    ro uint32_t reserved3[10];
    rw uint32_t mainclksel;
    rw uint32_t mainclkuen;
    rw uint32_t sysahbclkdiv;
    ro uint32_t reserved4;
    rw uint32_t sysahbclkctrl;
    ro uint32_t reserved5[4];
    rw uint32_t ssp0clkdiv;
    rw uint32_t uartclkdiv;
    rw uint32_t ssp1clkdiv;
    ro uint32_t reserved6[12];
    rw uint32_t wdtclksel;
    rw uint32_t wdtclkuen;
    rw uint32_t wdtclkdiv;
    ro uint32_t reserved7;
    rw uint32_t clkoutclksel;
    rw uint32_t clkoutuen;
    rw uint32_t clkoutclkdiv;
    ro uint32_t reserved8[5];
    ro uint32_t pioporcap0;
    ro uint32_t pioporcap1;
    ro uint32_t reserved9[18];
    rw uint32_t bodctrl;
    rw uint32_t systckcal;
    ro uint32_t reserved10[42];
    rw uint32_t startaprp0;
    rw uint32_t starterp0;
    wo uint32_t startrsrp0clr;
    ro uint32_t startsrp0;
    ro uint32_t reserved11[8];
    rw uint32_t pdsleepcfg;
    rw uint32_t pdawakecfg;
    rw uint32_t pdruncfg;
    ro uint32_t reserved12[110];
    ro uint32_t device_id;
    } lpc_syscon_t;

/** sysmemremap available values */
enum {
    LPC_SYSMEMREMAP_BOOTLOADER,
    LPC_SYSMEMREMAP_USERRAM,
    LPC_SYSMEMREMAP_USERFLASH,
    };

/** presetctrl register bits */
enum {
    LPC_PRESETCTRL_SSP0_RST_N   = (1 << 0),
    LPC_PRESETCTRL_I2C_RST_N    = (1 << 1),
    LPC_PRESETCTRL_SSP1_RST_N   = (1 << 2),
    LPC_PRESETCTRL_CAN_RST_N    = (1 << 3),
    };

#define LPC_SYSPLLCTRL_FEEDBACKDIV_1    0
#define LPC_SYSPLLCTRL_FEEDBACKDIV_2    1
#define LPC_SYSPLLCTRL_FEEDBACKDIV_3    2
#define LPC_SYSPLLCTRL_FEEDBACKDIV_4    3
#define LPC_SYSPLLCTRL_FEEDBACKDIV_5    4
#define LPC_SYSPLLCTRL_FEEDBACKDIV_6    5
#define LPC_SYSPLLCTRL_FEEDBACKDIV_7    6
#define LPC_SYSPLLCTRL_FEEDBACKDIV_8    7
#define LPC_SYSPLLCTRL_FEEDBACKDIV_9    8
#define LPC_SYSPLLCTRL_FEEDBACKDIV_10   9
#define LPC_SYSPLLCTRL_FEEDBACKDIV_11   10
#define LPC_SYSPLLCTRL_FEEDBACKDIV_12   11
#define LPC_SYSPLLCTRL_FEEDBACKDIV_13   12
#define LPC_SYSPLLCTRL_FEEDBACKDIV_14   13
#define LPC_SYSPLLCTRL_FEEDBACKDIV_15   14
#define LPC_SYSPLLCTRL_FEEDBACKDIV_16   15
#define LPC_SYSPLLCTRL_FEEDBACKDIV_17   16
#define LPC_SYSPLLCTRL_FEEDBACKDIV_18   17
#define LPC_SYSPLLCTRL_FEEDBACKDIV_19   18
#define LPC_SYSPLLCTRL_FEEDBACKDIV_20   19
#define LPC_SYSPLLCTRL_FEEDBACKDIV_21   20
#define LPC_SYSPLLCTRL_FEEDBACKDIV_22   21
#define LPC_SYSPLLCTRL_FEEDBACKDIV_23   22
#define LPC_SYSPLLCTRL_FEEDBACKDIV_24   23
#define LPC_SYSPLLCTRL_FEEDBACKDIV_25   24
#define LPC_SYSPLLCTRL_FEEDBACKDIV_26   25
#define LPC_SYSPLLCTRL_FEEDBACKDIV_27   26
#define LPC_SYSPLLCTRL_FEEDBACKDIV_28   27
#define LPC_SYSPLLCTRL_FEEDBACKDIV_29   28
#define LPC_SYSPLLCTRL_FEEDBACKDIV_30   29
#define LPC_SYSPLLCTRL_FEEDBACKDIV_31   30
#define LPC_SYSPLLCTRL_FEEDBACKDIV_32   31
#define LPC_SYSPLLCTRL_POSTDIV_1        (0 << 5)
#define LPC_SYSPLLCTRL_POSTDIV_2        (1 << 5)
#define LPC_SYSPLLCTRL_POSTDIV_4        (2 << 5)
#define LPC_SYSPLLCTRL_POSTDIV_8        (3 << 5)

#define LPC_SYSPLLSTAT_LOCKED           (1 << 0)

#define LPC_SYSOSCCTRL_BYPASS           (1 << 0)
#define LPC_SYSOSCCTRL_FRANGE_1_20MHZ   (0 << 1)
#define LPC_SYSOSCCTRL_FRANGE_15_25MHZ  (1 << 1)

#define LPC_WDTOSCCTRL_DIVSEL_2         0
#define LPC_WDTOSCCTRL_DIVSEL_4         1
#define LPC_WDTOSCCTRL_DIVSEL_6         2
#define LPC_WDTOSCCTRL_DIVSEL_8         3
#define LPC_WDTOSCCTRL_DIVSEL_10        4
#define LPC_WDTOSCCTRL_DIVSEL_12        5
#define LPC_WDTOSCCTRL_DIVSEL_14        6
#define LPC_WDTOSCCTRL_DIVSEL_16        7
#define LPC_WDTOSCCTRL_DIVSEL_18        8
#define LPC_WDTOSCCTRL_DIVSEL_20        9
#define LPC_WDTOSCCTRL_DIVSEL_22        10
#define LPC_WDTOSCCTRL_DIVSEL_24        11
#define LPC_WDTOSCCTRL_DIVSEL_26        12
#define LPC_WDTOSCCTRL_DIVSEL_28        13
#define LPC_WDTOSCCTRL_DIVSEL_30        14
#define LPC_WDTOSCCTRL_DIVSEL_32        15
#define LPC_WDTOSCCTRL_DIVSEL_34        16
#define LPC_WDTOSCCTRL_DIVSEL_36        17
#define LPC_WDTOSCCTRL_DIVSEL_38        18
#define LPC_WDTOSCCTRL_DIVSEL_40        19
#define LPC_WDTOSCCTRL_DIVSEL_42        20
#define LPC_WDTOSCCTRL_DIVSEL_44        21
#define LPC_WDTOSCCTRL_DIVSEL_46        22
#define LPC_WDTOSCCTRL_DIVSEL_48        23
#define LPC_WDTOSCCTRL_DIVSEL_50        24
#define LPC_WDTOSCCTRL_DIVSEL_52        25
#define LPC_WDTOSCCTRL_DIVSEL_54        26
#define LPC_WDTOSCCTRL_DIVSEL_56        27
#define LPC_WDTOSCCTRL_DIVSEL_58        28
#define LPC_WDTOSCCTRL_DIVSEL_60        29
#define LPC_WDTOSCCTRL_DIVSEL_62        30
#define LPC_WDTOSCCTRL_DIVSEL_64        31

#define LPC_WDTOSCCTRL_FREQSEL_0_5MHZ   (1 << 5)
#define LPC_WDTOSCCTRL_FREQSEL_0_8MHZ   (2 << 5)
#define LPC_WDTOSCCTRL_FREQSEL_1_1MHZ   (3 << 5)
#define LPC_WDTOSCCTRL_FREQSEL_1_4MHZ   (4 << 5)
#define LPC_WDTOSCCTRL_FREQSEL_1_6MHZ   (5 << 5)
#define LPC_WDTOSCCTRL_FREQSEL_1_8MHZ   (6 << 5)
#define LPC_WDTOSCCTRL_FREQSEL_2_0MHZ   (7 << 5)
#define LPC_WDTOSCCTRL_FREQSEL_2_2MHZ   (8 << 5)
#define LPC_WDTOSCCTRL_FREQSEL_2_4MHZ   (9 << 5)
#define LPC_WDTOSCCTRL_FREQSEL_2_6MHZ   (10 << 5)
#define LPC_WDTOSCCTRL_FREQSEL_2_7MHZ   (11 << 5)
#define LPC_WDTOSCCTRL_FREQSEL_2_9MHZ   (12 << 5)
#define LPC_WDTOSCCTRL_FREQSEL_3_1MHZ   (13 << 5)
#define LPC_WDTOSCCTRL_FREQSEL_3_2MHZ   (14 << 5)
#define LPC_WDTOSCCTRL_FREQSEL_3_4MHZ   (15 << 5)

#define LPC_SYSRSTSTAT_POR              (1 << 0)
#define LPC_SYSRSTSTAT_EXTRST           (1 << 1)
#define LPC_SYSRSTSTAT_WDT              (1 << 2)
#define LPC_SYSRSTSTAT_BOD              (1 << 3)
#define LPC_SYSRSTSTAT_SYSRST           (1 << 4)

#define LPC_SYSPLLCLKSEL_IRCOSC         0
#define LPC_SYSPLLCLKSEL_SYSOSC         1

#define LPC_SYSPLLCLKUEN_ENABLE         (1 << 0)

#define LPC_MAINCLKSEL_IRCOSC           0
#define LPC_MAINCLKSEL_SYSPLL_IN        1
#define LPC_MAINCLKSEL_WDTOSC           2
#define LPC_MAINCLKSEL_SYSPLL_OUT       3

#define LPC_MAINCLKUEN_ENABLE           (1 << 0)

#define LPC_SYSAHBCLKDIV_DISABLE        0

#define LPC_SYSAHBCLKCTRL_ENABLE_SYS    (1 << 0)
#define LPC_SYSAHBCLKCTRL_ENABLE_ROM    (1 << 1)
#define LPC_SYSAHBCLKCTRL_ENABLE_RAM    (1 << 2)
#define LPC_SYSAHBCLKCTRL_ENABLE_FLASHREG   (1 << 3)
#define LPC_SYSAHBCLKCTRL_ENABLE_FLASHARRAY (1 << 4)
#define LPC_SYSAHBCLKCTRL_ENABLE_I2C    (1 << 5)
#define LPC_SYSAHBCLKCTRL_ENABLE_GPIO   (1 << 6)
#define LPC_SYSAHBCLKCTRL_ENABLE_CT16B0 (1 << 7)
#define LPC_SYSAHBCLKCTRL_ENABLE_CT16B1 (1 << 8)
#define LPC_SYSAHBCLKCTRL_ENABLE_CT32B0 (1 << 9)
#define LPC_SYSAHBCLKCTRL_ENABLE_CT32B1 (1 << 10)
#define LPC_SYSAHBCLKCTRL_ENABLE_SSP0   (1 << 11)
#define LPC_SYSAHBCLKCTRL_ENABLE_UART   (1 << 12)
#define LPC_SYSAHBCLKCTRL_ENABLE_ADC    (1 << 13)
#define LPC_SYSAHBCLKCTRL_ENABLE_WDT    (1 << 15)
#define LPC_SYSAHBCLKCTRL_ENABLE_IOCON  (1 << 16)
#define LPC_SYSAHBCLKCTRL_ENABLE_CAN    (1 << 17)
#define LPC_SYSAHBCLKCTRL_ENABLE_SSP1   (1 << 18)

#define LPC_SSP0CLKDIV_DISABLE          0

#define LPC_UARTCLKDIV_DISABLE          0

#define LPC_SSP1CLKDIV_DISABLE          0

#define LPC_WDTCLKSEL_IRCOSC            0
#define LPC_WDTCLKSEL_MAINCLK           1
#define LPC_WDTCLKSEL_WDTOSC            2

#define LPC_WDTCLKUEN_ENABLE            (1 << 0)

#define LPC_WDTCLKDIV_DISABLE           0

#define LPC_CLKOUTCLKSEL_IRCOSC         0
#define LPC_CLKOUTCLKSEL_SYSOSC         1
#define LPC_CLKOUTCLKSEL_WDTOSC         2
#define LPC_CLKOUTCLKSEL_MAINOSC        3

#define LPC_CLKOUTUEN_ENABLE            (1 << 0)

#define LPC_CLKOUTCLKDIV_DISABLE        0

#define LPC_BODCTRL_BODRSTLEV_0         0
#define LPC_BODCTRL_BODRSTLEV_1         1
#define LPC_BODCTRL_BODRSTLEV_2         2
#define LPC_BODCTRL_BODRSTLEV_3         3
#define LPC_BODCTRL_BODINTVAL_0         (0 << 2)
#define LPC_BODCTRL_BODINTVAL_1         (1 << 2)
#define LPC_BODCTRL_BODINTVAL_2         (2 << 2)
#define LPC_BODCTRL_BODINTVAL_3         (3 << 2)
#define LPC_BODCTRL_BODRST_ENABLE       (1 << 4)

/** Brown-out detect power down (check user manual for the extra bits!) */
#define LPC_PDSLEEPCFG_BOD_PD           ((1 << 3) | (0x18B7))
#define LPC_PDSLEEPCFG_WDTOSC_PD        ((1 << 6) | (0x18B7))

/** Check user manual for "magic value" extra bits! */
#define LPC_PDRUNCFG_MUSTSET            (0xEDFF)
#define LPC_PDRUNCFG_IRCOUT_PD          (1 << 0)
#define LPC_PDRUNCFG_IRC_PD             (1 << 1)
#define LPC_PDRUNCFG_FLASH_PD           (1 << 2)
#define LPC_PDRUNCFG_BOD_PD             (1 << 3)
#define LPC_PDRUNCFG_ADC_PD             (1 << 4)
#define LPC_PDRUNCFG_SYSOSC_PD          (1 << 5)
#define LPC_PDRUNCFG_WDTOSC_PD          (1 << 6)
#define LPC_PDRUNCFG_SYSPLL_PD          (1 << 7)

/** Check user manual for "magic value" extra bits! */
#define LPC_PDAWAKECFG_MUSTSET
#define LPC_PDAWAKECFG_IRCOUT_PD        ((1 << 0) | (0xED00))
#define LPC_PDAWAKECFG_IRC_PD           ((1 << 1) | (0xED00))
#define LPC_PDAWAKECFG_FLASH_PD         ((1 << 2) | (0xED00))
#define LPC_PDAWAKECFG_BOD_PD           ((1 << 3) | (0xED00))
#define LPC_PDAWAKECFG_ADC_PD           ((1 << 4) | (0xED00))
#define LPC_PDAWAKECFG_SYSOSC_PD        ((1 << 5) | (0xED00))
#define LPC_PDAWAKECFG_WDTOSC_PD        ((1 << 6) | (0xED00))
#define LPC_PDAWAKECFG_SYSPLL_PD        ((1 << 7) | (0xED00))

#define LPC_DEVICEID_LPC11C14_FBD48_301 0x1440102B

#endif
