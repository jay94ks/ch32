#ifndef __CH32V00X_CORE_V00X_H__
#define __CH32V00X_CORE_V00X_H__

#include "riscv.h"

/**
 * File: ch32v00x/core/v00x.h
 * Desc: Peripheral register structures for V00x.
 * Author: jay94ks.
 */

namespace ch32 {

    /**
     * CH32V00X peripheral structures.
     * these are mapped on memory directly.
     */
    namespace v00x {
        /**
         * ADC, analog to digital converter.
         */
        typedef struct {
            RV_RW uint32_t statr;
            RV_RW uint32_t ctlr1;
            RV_RW uint32_t ctlr2;
            RV_RW uint32_t samptr1;
            RV_RW uint32_t samptr2;
            RV_RW uint32_t iofr1;
            RV_RW uint32_t iofr2;
            RV_RW uint32_t iofr3;
            RV_RW uint32_t iofr4;
            RV_RW uint32_t wdhtr;
            RV_RW uint32_t wdltr;
            RV_RW uint32_t rsqr1;
            RV_RW uint32_t rsqr2;
            RV_RW uint32_t rsqr3;
            RV_RW uint32_t isqr;
            RV_RW uint32_t idatar1;
            RV_RW uint32_t idatar2;
            RV_RW uint32_t idatar3;
            RV_RW uint32_t idatar4;
            RV_RW uint32_t rdatar;
            RV_RW uint32_t dlyr;
        } SADC;

        /**
         * Debug MCU. 
         */
        typedef struct {
            RV_RW uint32_t cfgr0;
            RV_RW uint32_t cfgr1;
        } SDebugMcu;

        /**
         * DMA channel.
         */
        typedef struct {
            RV_RW uint32_t cfgr;
            RV_RW uint32_t cntr;
            RV_RW uint32_t paddr;
            RV_RW uint32_t maddr;
        } SDMAChannel;

        /**
         * DMA.
         */
        typedef struct {
            RV_RW uint32_t intfr;
            RV_RW uint32_t intfcr;
        } SDMA;

        /**
         * EXTI.
         */
        typedef struct {
            RV_RW uint32_t intenr;
            RV_RW uint32_t evenr;
            RV_RW uint32_t rtenr;
            RV_RW uint32_t ftenr;
            RV_RW uint32_t swievr;
            RV_RW uint32_t intfr;
        } SEXTI;

        /**
         * Flash.
         */
        typedef struct {
            RV_RW uint32_t actlr;
            RV_RW uint32_t keyr;
            RV_RW uint32_t obkeyr;
            RV_RW uint32_t statr;
            RV_RW uint32_t ctlr;
            RV_RW uint32_t addr;
            RV_RW uint32_t reserved;
            RV_RW uint32_t obr;
            RV_RW uint32_t wpr;
            RV_RW uint32_t modekeyr;
            RV_RW uint32_t boot_mkr;
        } SFlash;

        /**
         * Option bytes. 
         */
        typedef struct {
            RV_RW uint16_t rdpr;
            RV_RW uint16_t user;
            RV_RW uint16_t data0;
            RV_RW uint16_t data1;
            RV_RW uint16_t wrpr0;
            RV_RW uint16_t wrpr1;
        } SOB;

        /**
         * GPIO.
         */
        typedef struct {
            RV_RW uint32_t cfglr;
            uint32_t reserved0;
            RV_RW uint32_t indr;
            RV_RW uint32_t outdr;
            RV_RW uint32_t bshr;
            RV_RW uint32_t bcr;
            RV_RW uint32_t lckr;
        } SGPIO;

        /**
         * Alternative Function.
         */
        typedef struct {
            uint32_t reserved0;
            RV_RW uint32_t pcfr1;
            RV_RW uint32_t exticr;
        } SAFIO;

        /**
         * I2C.
         */
        typedef struct {
            RV_RW uint16_t ctlr1;
            uint16_t reserved0;
            RV_RW uint16_t ctlr2;
            uint16_t reserved1;
            RV_RW uint16_t oaddr1;
            uint16_t reserved2;
            RV_RW uint16_t oaddr2;
            uint16_t reserved3;
            RV_RW uint16_t datar;
            uint16_t reserved4;
            RV_RW uint16_t star1;
            uint16_t reserved5;
            RV_RW uint16_t star2;
            uint16_t reserved6;
            RV_RW uint16_t ckcfgr;
            uint16_t reserved7;
        } SI2C;

        /**
         * Independent Watchdog. 
         */
        typedef struct {
            RV_RW uint32_t ctlr;
            RV_RW uint32_t pscr;
            RV_RW uint32_t rldr;
            RV_RW uint32_t statr;
        } SIWDG;

        /**
         * Power control. 
         */
        typedef struct {
            RV_RW uint32_t ctlr;
            RV_RW uint32_t csr;
            RV_RW uint32_t awucsr;
            RV_RW uint32_t awuwr;
            RV_RW uint32_t awupsc;
        } SPWR;

        /**
         * Reset and Clock control. 
         */
        typedef struct {
            RV_RW uint32_t ctlr;
            RV_RW uint32_t cfgr0;
            RV_RW uint32_t intr;
            RV_RW uint32_t apb2prstr;
            RV_RW uint32_t apb1prstr;
            RV_RW uint32_t ahbpcenr;
            RV_RW uint32_t apb2pcenr;
            RV_RW uint32_t apb1pcenr;
            RV_RW uint32_t reserved0;
            RV_RW uint32_t rstsckr;
        } SRCC;

        /**
         * SPI
         */
        typedef struct {
            RV_RW uint16_t ctlr1;
            uint16_t reserved0;
            RV_RW uint16_t ctlr2;
            uint16_t reserved1;
            RV_RW uint16_t statr;
            uint16_t reserved2;
            RV_RW uint16_t datar;
            uint16_t reserved3;
            RV_RW uint16_t crcr;
            uint16_t reserved4;
            RV_RW uint16_t rcrcr;
            uint16_t reserved5;
            RV_RW uint16_t tcrcr;
            uint16_t reserved6;
            uint16_t reserved7;
            uint16_t reserved8;
            RV_RW uint16_t hscr;
            uint16_t reserved9;
        } SSPI;

        /**
         * TIM.
         */
        typedef struct {
            RV_RW uint16_t ctlr1;
            uint16_t      reserved0;
            RV_RW uint16_t cltr2;
            uint16_t      reserved1;
            RV_RW uint16_t smcfgr;
            uint16_t      reserved2;
            RV_RW uint16_t dmaintenr;
            uint16_t      reserved3;
            RV_RW uint16_t intfr;
            uint16_t      reserved4;
            RV_RW uint16_t swevgr;
            uint16_t      reserved5;
            RV_RW uint16_t chctlr1;
            uint16_t      reserved6;
            RV_RW uint16_t chctlr2;
            uint16_t      reserved7;
            RV_RW uint16_t ccer;
            uint16_t      reserved8;
            RV_RW uint16_t cnt;
            uint16_t      reserved9;
            RV_RW uint16_t psc;
            uint16_t      reserved10;
            RV_RW uint16_t atrlr;
            uint16_t      reserved11;
            RV_RW uint16_t rptcr;
            uint16_t      reserved12;
            RV_RW uint32_t ch1cvr;
            RV_RW uint32_t ch2cvr;
            RV_RW uint32_t ch3cvr;
            RV_RW uint32_t ch4cvr;
            RV_RW uint16_t bdtr;
            uint16_t      reserved13;
            RV_RW uint16_t dmacfgr;
            uint16_t      reserved14;
            RV_RW uint16_t dmadr;
            uint16_t      reserved15;
        } STIM;

        /**
         * USART.
         */
        typedef struct {
            RV_RW uint16_t statr;
            uint16_t reserved0;
            RV_RW uint16_t datar;
            uint16_t reserved1;
            RV_RW uint16_t brr;
            uint16_t reserved2;
            RV_RW uint16_t ctlr1;
            uint16_t reserved3;
            RV_RW uint16_t ctlr2;
            uint16_t reserved4;
            RV_RW uint16_t ctlr3;
            uint16_t reserved5;
            RV_RW uint16_t gpr;
            uint16_t reserved6;
        } SUSART;

        /**
         * Window Watchdog.
         */
        typedef struct {
            RV_RW uint32_t ctlr;
            RV_RW uint32_t cfgr;
            RV_RW uint32_t statr;
        } SWWDG;

        /**
         * Enhanced registers.
         */
        typedef struct {
            RV_RW uint32_t exten_ctr;
        } SEXTEN;

        /**
         * Base addresses.
         */
        constexpr uint32_t flash_base = 0x08000000;
        constexpr uint32_t sram_base = 0x20000000;
        constexpr uint32_t periph_base = 0x40000000;

        constexpr uint32_t apb1_base = periph_base;
        constexpr uint32_t apb2_base = periph_base + 0x10000;
        constexpr uint32_t ahb_base = periph_base + 0x20000;

        constexpr uint32_t tim2_base = apb1_base + 0x0000;
        constexpr uint32_t wwdg_base = apb1_base + 0x2c00;
        constexpr uint32_t iwdg_base = apb1_base + 0x3000;
        constexpr uint32_t i2c1_base = apb1_base + 0x5400;
        constexpr uint32_t pwr_base = apb1_base + 0x7000;

        constexpr uint32_t afio_base = apb2_base + 0x0000;
        constexpr uint32_t exti_base = apb2_base + 0x0400;
        constexpr uint32_t gpioa_base = apb2_base + 0x0800;
        constexpr uint32_t gpioc_base = apb2_base + 0x1000;
        constexpr uint32_t gpiod_base = apb2_base + 0x1400;
        constexpr uint32_t adc1_base = apb2_base + 0x2400;
        constexpr uint32_t tim1_base = apb2_base + 0x2c00;
        constexpr uint32_t spi1_base = apb2_base + 0x3000;
        constexpr uint32_t usart1_base = apb2_base + 0x3800;

        constexpr uint32_t dma1_base = ahb_base + 0x0000;
        constexpr uint32_t dma1_1_base = ahb_base + 0x0008;
        constexpr uint32_t dma1_2_base = ahb_base + 0x001c;
        constexpr uint32_t dma1_3_base = ahb_base + 0x0030;
        constexpr uint32_t dma1_4_base = ahb_base + 0x0044;
        constexpr uint32_t dma1_5_base = ahb_base + 0x0058;
        constexpr uint32_t dma1_6_base = ahb_base + 0x006c;
        constexpr uint32_t dma1_7_base = ahb_base + 0x0080;

        constexpr uint32_t rcc_base = ahb_base + 0x1000;
        constexpr uint32_t flash_r_base = ahb_base + 0x2000;

        constexpr uint32_t ob_base = 0x1ffff800;
        constexpr uint32_t exten_base = 0x40023800;

        constexpr uint32_t vendor_cfg0_base = 0x1ffff7d4;
        constexpr uint32_t cfg0_pll_trim = vendor_cfg0_base;

        // --> define RV_* peripheral accessors.
        #define RV_V00X(type, base)     ((ch32::v00x::type*)(ch32::v00x::base))

        /**
         * peripheral register accessors.
         */
        #define RV_TIM2     RV_V00X(STIM, tim2_base)
        #define RV_WWDG     RV_V00X(SWWDG, wwdg_base)
        #define RV_IWDG     RV_V00X(SIWDG, iwdg_base)
        #define RV_I2C1     RV_V00X(SI2C, i2c1_base)
        #define RV_PWR      RV_V00X(SPWR, pwr_base)
        #define RV_AFIO     RV_V00X(SAFIO, afio_base)
        #define RV_EXTI     RV_V00X(SEXTI, exti_base)
        #define RV_GPIOA    RV_V00X(SGPIO, gpioa_base)
        #define RV_GPIOC    RV_V00X(SGPIO, gpioc_base)
        #define RV_GPIOD    RV_V00X(SGPIO, gpiod_base)
        #define RV_ADC1     RV_V00X(SADC, adc1_base)
        #define RV_TIM1     RV_V00X(STIM, tim1_base)
        #define RV_SPI1     RV_V00X(SSPI, spi1_base)
        #define RV_USART1   RV_V00X(SUSART, usart1_base)
        #define RV_DMA1     RV_V00X(SDMA, dma1_base)
        #define RV_DMA1_1   RV_V00X(SDMAChannel, dma1_1_base)
        #define RV_DMA1_2   RV_V00X(SDMAChannel, dma1_2_base)
        #define RV_DMA1_3   RV_V00X(SDMAChannel, dma1_3_base)
        #define RV_DMA1_4   RV_V00X(SDMAChannel, dma1_4_base)
        #define RV_DMA1_5   RV_V00X(SDMAChannel, dma1_5_base)
        #define RV_DMA1_6   RV_V00X(SDMAChannel, dma1_6_base)
        #define RV_DMA1_7   RV_V00X(SDMAChannel, dma1_7_base)
        #define RV_RCC      RV_V00X(SRCC, rcc_base)
        #define RV_FLASH    RV_V00X(SFlash, flash_base)
        #define RV_OB       RV_V00X(SOB, ob_base)
        #define RV_EXTEN    RV_V00X(SEXTEN, exten_base)

        /**
         * ADC_STATR
         */
        enum {
            ADC_AWD = 0x01,
            ADC_EOC = 0x02,
            ADC_JEOC = 0x04,
            ADC_JSTRT = 0x08,
            ADC_STRT = 0x10,
        };

        /**
         * ADC_CTLR1
         */
        enum {
            ADC_AWDCH = 0x0000001f,
            ADC_AWDCH_0 = 0x00000001,
            ADC_AWDCH_1 = 0x00000002,
            ADC_AWDCH_2 = 0x00000004,
            ADC_AWDCH_3 = 0x00000008,
            ADC_AWDCH_4 = 0x00000010,
            ADC_EOCIE = 0x00000020,
            ADC_AWDIE = 0x00000040,
            ADC_JEOCIE = 0x00000080,
            ADC_SCAN = 0x00000100,
            ADC_AWDSGL = 0x00000200,
            ADC_JAUTO = 0x00000400,
            ADC_DISCEN = 0x00000800,
            ADC_JDISCEN = 0x00001000,
            ADC_DISCNUM = 0x0000e000,
            ADC_DISCNUM_0 = 0x00002000,
            ADC_DISCNUM_1 = 0x00004000,
            ADC_DISCNUM_2 = 0x00008000,
            ADC_JAWDEN = 0x00400000,
            ADC_AWDEN = 0x00800000,
            ADC_CALVOLSELECT = 0x06000000,
            ADC_CALVOLSELECT_0 = 0x02000000,
            ADC_CALVOLSELECT_1 = 0x04000000,
        };

        /**
         * ADC_CTLR2
         */
        enum {
            ADC_ADON = 0x00000001,
            ADC_CONT = 0x00000002,
            ADC_CAL = 0x00000004,
            ADC_RSTCAL = 0x00000008,
            ADC_DMA = 0x00000100,
            ADC_ALIGN = 0x00000800,
            ADC_JEXTSEL = 0x00007000,
            ADC_JEXTSEL_0 = 0x00001000,
            ADC_JEXTSEL_1 = 0x00002000,
            ADC_JEXTSEL_2 = 0x00004000,
            ADC_JEXTTRIG = 0x00008000,
            ADC_EXTSEL = 0x000e0000,
            ADC_EXTSEL_0 = 0x00020000,
            ADC_EXTSEL_1 = 0x00040000,
            ADC_EXTSEL_2 = 0x00080000,
            ADC_EXTTRIG = 0x00100000,
            ADC_JSWSTART = 0x00200000,
            ADC_SWSTART = 0x00400000,
        };

        /**
         * ADC_SAMPTR1
         */
        enum {
            ADC_SMP10 = 0x00000007,
            ADC_SMP10_0 = 0x00000001,
            ADC_SMP10_1 = 0x00000002,
            ADC_SMP10_2 = 0x00000004,
            ADC_SMP11 = 0x00000038,
            ADC_SMP11_0 = 0x00000008,
            ADC_SMP11_1 = 0x00000010,
            ADC_SMP11_2 = 0x00000020,
            ADC_SMP12 = 0x000001c0,
            ADC_SMP12_0 = 0x00000040,
            ADC_SMP12_1 = 0x00000080,
            ADC_SMP12_2 = 0x00000100,
            ADC_SMP13 = 0x00000e00,
            ADC_SMP13_0 = 0x00000200,
            ADC_SMP13_1 = 0x00000400,
            ADC_SMP13_2 = 0x00000800,
            ADC_SMP14 = 0x00007000,
            ADC_SMP14_0 = 0x00001000,
            ADC_SMP14_1 = 0x00002000,
            ADC_SMP14_2 = 0x00004000,
            ADC_SMP15 = 0x00038000,
            ADC_SMP15_0 = 0x00008000,
            ADC_SMP15_1 = 0x00010000,
            ADC_SMP15_2 = 0x00020000,
        };

        /**
         * ADC_SAMPTR2
         */
        enum {
            ADC_SMP0 = 0x00000007,
            ADC_SMP0_0 = 0x00000001,
            ADC_SMP0_1 = 0x00000002,
            ADC_SMP0_2 = 0x00000004,
            ADC_SMP1 = 0x00000038,
            ADC_SMP1_0 = 0x00000008,
            ADC_SMP1_1 = 0x00000010,
            ADC_SMP1_2 = 0x00000020,
            ADC_SMP2 = 0x000001c0,
            ADC_SMP2_0 = 0x00000040,
            ADC_SMP2_1 = 0x00000080,
            ADC_SMP2_2 = 0x00000100,
            ADC_SMP3 = 0x00000e00,
            ADC_SMP3_0 = 0x00000200,
            ADC_SMP3_1 = 0x00000400,
            ADC_SMP3_2 = 0x00000800,
            ADC_SMP4 = 0x00007000,
            ADC_SMP4_0 = 0x00001000,
            ADC_SMP4_1 = 0x00002000,
            ADC_SMP4_2 = 0x00004000,
            ADC_SMP5 = 0x00038000,
            ADC_SMP5_0 = 0x00008000,
            ADC_SMP5_1 = 0x00010000,
            ADC_SMP5_2 = 0x00020000,
            ADC_SMP6 = 0x001c0000,
            ADC_SMP6_0 = 0x00040000,
            ADC_SMP6_1 = 0x00080000,
            ADC_SMP6_2 = 0x00100000,
            ADC_SMP7 = 0x00e00000,
            ADC_SMP7_0 = 0x00200000,
            ADC_SMP7_1 = 0x00400000,
            ADC_SMP7_2 = 0x00800000,
            ADC_SMP8 = 0x07000000,
            ADC_SMP8_0 = 0x01000000,
            ADC_SMP8_1 = 0x02000000,
            ADC_SMP8_2 = 0x04000000,
            ADC_SMP9 = 0x38000000,
            ADC_SMP9_0 = 0x08000000,
            ADC_SMP9_1 = 0x10000000,
            ADC_SMP9_2 = 0x20000000,
        };

        /**
         * ADC_IOFR1
         */
        enum {
            ADC_JOFFSET1 = 0x03ff,
        };

        /**
         * ADC_IOFR2
         */
        enum {
            ADC_JOFFSET2 = 0x03ff,
        };

        /**
         * ADC_IOFR3
         */
        enum {
            ADC_JOFFSET3 = 0x03ff,
        };

        /**
         * ADC_IOFR4
         */
        enum {
            ADC_JOFFSET4 = 0x03ff,
        };

        /**
         * ADC_WDHTR
         */
        enum {
            ADC_HT = 0x03ff,
        };

        /**
         * ADC_WDLTR
         */
        enum {
            ADC_LT = 0x03ff,
        };

        /**
         * ADC_RSQR1
         */
        enum {
            ADC_SQ13 = 0x0000001f,
            ADC_SQ13_0 = 0x00000001,
            ADC_SQ13_1 = 0x00000002,
            ADC_SQ13_2 = 0x00000004,
            ADC_SQ13_3 = 0x00000008,
            ADC_SQ13_4 = 0x00000010,
            ADC_SQ14 = 0x000003e0,
            ADC_SQ14_0 = 0x00000020,
            ADC_SQ14_1 = 0x00000040,
            ADC_SQ14_2 = 0x00000080,
            ADC_SQ14_3 = 0x00000100,
            ADC_SQ14_4 = 0x00000200,
            ADC_SQ15 = 0x00007c00,
            ADC_SQ15_0 = 0x00000400,
            ADC_SQ15_1 = 0x00000800,
            ADC_SQ15_2 = 0x00001000,
            ADC_SQ15_3 = 0x00002000,
            ADC_SQ15_4 = 0x00004000,
            ADC_SQ16 = 0x000f8000,
            ADC_SQ16_0 = 0x00008000,
            ADC_SQ16_1 = 0x00010000,
            ADC_SQ16_2 = 0x00020000,
            ADC_SQ16_3 = 0x00040000,
            ADC_SQ16_4 = 0x00080000,
            ADC_L = 0x00f00000,
            ADC_L_0 = 0x00100000,
            ADC_L_1 = 0x00200000,
            ADC_L_2 = 0x00400000,
            ADC_L_3 = 0x00800000,
        };

        /**
         * ADC_RSQR2
         */
        enum {
            ADC_SQ7 = 0x0000001f,
            ADC_SQ7_0 = 0x00000001,
            ADC_SQ7_1 = 0x00000002,
            ADC_SQ7_2 = 0x00000004,
            ADC_SQ7_3 = 0x00000008,
            ADC_SQ7_4 = 0x00000010,
            ADC_SQ8 = 0x000003e0,
            ADC_SQ8_0 = 0x00000020,
            ADC_SQ8_1 = 0x00000040,
            ADC_SQ8_2 = 0x00000080,
            ADC_SQ8_3 = 0x00000100,
            ADC_SQ8_4 = 0x00000200,
            ADC_SQ9 = 0x00007c00,
            ADC_SQ9_0 = 0x00000400,
            ADC_SQ9_1 = 0x00000800,
            ADC_SQ9_2 = 0x00001000,
            ADC_SQ9_3 = 0x00002000,
            ADC_SQ9_4 = 0x00004000,
            ADC_SQ10 = 0x000f8000,
            ADC_SQ10_0 = 0x00008000,
            ADC_SQ10_1 = 0x00010000,
            ADC_SQ10_2 = 0x00020000,
            ADC_SQ10_3 = 0x00040000,
            ADC_SQ10_4 = 0x00080000,
            ADC_SQ11 = 0x01f00000,
            ADC_SQ11_0 = 0x00100000,
            ADC_SQ11_1 = 0x00200000,
            ADC_SQ11_2 = 0x00400000,
            ADC_SQ11_3 = 0x00800000,
            ADC_SQ11_4 = 0x01000000,
            ADC_SQ12 = 0x3e000000,
            ADC_SQ12_0 = 0x02000000,
            ADC_SQ12_1 = 0x04000000,
            ADC_SQ12_2 = 0x08000000,
            ADC_SQ12_3 = 0x10000000,
            ADC_SQ12_4 = 0x20000000,
        };

        /**
         * ADC_RSQR3
         */
        enum {
            ADC_SQ1 = 0x0000001f,
            ADC_SQ1_0 = 0x00000001,
            ADC_SQ1_1 = 0x00000002,
            ADC_SQ1_2 = 0x00000004,
            ADC_SQ1_3 = 0x00000008,
            ADC_SQ1_4 = 0x00000010,
            ADC_SQ2 = 0x000003e0,
            ADC_SQ2_0 = 0x00000020,
            ADC_SQ2_1 = 0x00000040,
            ADC_SQ2_2 = 0x00000080,
            ADC_SQ2_3 = 0x00000100,
            ADC_SQ2_4 = 0x00000200,
            ADC_SQ3 = 0x00007c00,
            ADC_SQ3_0 = 0x00000400,
            ADC_SQ3_1 = 0x00000800,
            ADC_SQ3_2 = 0x00001000,
            ADC_SQ3_3 = 0x00002000,
            ADC_SQ3_4 = 0x00004000,
            ADC_SQ4 = 0x000f8000,
            ADC_SQ4_0 = 0x00008000,
            ADC_SQ4_1 = 0x00010000,
            ADC_SQ4_2 = 0x00020000,
            ADC_SQ4_3 = 0x00040000,
            ADC_SQ4_4 = 0x00080000,
            ADC_SQ5 = 0x01f00000,
            ADC_SQ5_0 = 0x00100000,
            ADC_SQ5_1 = 0x00200000,
            ADC_SQ5_2 = 0x00400000,
            ADC_SQ5_3 = 0x00800000,
            ADC_SQ5_4 = 0x01000000,
            ADC_SQ6 = 0x3e000000,
            ADC_SQ6_0 = 0x02000000,
            ADC_SQ6_1 = 0x04000000,
            ADC_SQ6_2 = 0x08000000,
            ADC_SQ6_3 = 0x10000000,
            ADC_SQ6_4 = 0x20000000,
        };

        /**
         * ADC_ISQR
         */
        enum {
            ADC_JSQ1 = 0x0000001f,
            ADC_JSQ1_0 = 0x00000001,
            ADC_JSQ1_1 = 0x00000002,
            ADC_JSQ1_2 = 0x00000004,
            ADC_JSQ1_3 = 0x00000008,
            ADC_JSQ1_4 = 0x00000010,
            ADC_JSQ2 = 0x000003e0,
            ADC_JSQ2_0 = 0x00000020,
            ADC_JSQ2_1 = 0x00000040,
            ADC_JSQ2_2 = 0x00000080,
            ADC_JSQ2_3 = 0x00000100,
            ADC_JSQ2_4 = 0x00000200,
            ADC_JSQ3 = 0x00007c00,
            ADC_JSQ3_0 = 0x00000400,
            ADC_JSQ3_1 = 0x00000800,
            ADC_JSQ3_2 = 0x00001000,
            ADC_JSQ3_3 = 0x00002000,
            ADC_JSQ3_4 = 0x00004000,
            ADC_JSQ4 = 0x000f8000,
            ADC_JSQ4_0 = 0x00008000,
            ADC_JSQ4_1 = 0x00010000,
            ADC_JSQ4_2 = 0x00020000,
            ADC_JSQ4_3 = 0x00040000,
            ADC_JSQ4_4 = 0x00080000,
            ADC_JL = 0x00300000,
            ADC_JL_0 = 0x00100000,
            ADC_JL_1 = 0x00200000,
        };

        /**
         * ADC_IDATAR1
         */
        enum {
            ADC_IDATAR1_JDATA = 0xffff,
        };

        /**
         * ADC_IDATAR2
         */
        enum {
            ADC_IDATAR2_JDATA = 0xffff,
        };

        /**
         * ADC_IDATAR3
         */
        enum {
            ADC_IDATAR3_JDATA = 0xffff,
        };

        /**
         * ADC_IDATAR4
         */
        enum {
            ADC_IDATAR4_JDATA = 0xffff,
        };

        /**
         * ADC_RDATAR
         */
        enum {
            ADC_RDATAR_DATA = 0xffffffff,
        };

        /**
         * ADC_DLYR
         */
        enum {
            ADC_DLYR_DLYVLU = 0x1ff,
            ADC_DLYR_DLYSRC = 0x200,
        };

        /**
         * DMA_INTFR
         */
        enum {
            DMA_GIF1 = 0x00000001,
            DMA_TCIF1 = 0x00000002,
            DMA_HTIF1 = 0x00000004,
            DMA_TEIF1 = 0x00000008,
            DMA_GIF2 = 0x00000010,
            DMA_TCIF2 = 0x00000020,
            DMA_HTIF2 = 0x00000040,
            DMA_TEIF2 = 0x00000080,
            DMA_GIF3 = 0x00000100,
            DMA_TCIF3 = 0x00000200,
            DMA_HTIF3 = 0x00000400,
            DMA_TEIF3 = 0x00000800,
            DMA_GIF4 = 0x00001000,
            DMA_TCIF4 = 0x00002000,
            DMA_HTIF4 = 0x00004000,
            DMA_TEIF4 = 0x00008000,
            DMA_GIF5 = 0x00010000,
            DMA_TCIF5 = 0x00020000,
            DMA_HTIF5 = 0x00040000,
            DMA_TEIF5 = 0x00080000,
            DMA_GIF6 = 0x00100000,
            DMA_TCIF6 = 0x00200000,
            DMA_HTIF6 = 0x00400000,
            DMA_TEIF6 = 0x00800000,
            DMA_GIF7 = 0x01000000,
            DMA_TCIF7 = 0x02000000,
            DMA_HTIF7 = 0x04000000,
            DMA_TEIF7 = 0x08000000,
        };

        /**
         * DMA_INTFCR
         */
        enum {
            DMA_CGIF1 = 0x00000001,
            DMA_CTCIF1 = 0x00000002,
            DMA_CHTIF1 = 0x00000004,
            DMA_CTEIF1 = 0x00000008,
            DMA_CGIF2 = 0x00000010,
            DMA_CTCIF2 = 0x00000020,
            DMA_CHTIF2 = 0x00000040,
            DMA_CTEIF2 = 0x00000080,
            DMA_CGIF3 = 0x00000100,
            DMA_CTCIF3 = 0x00000200,
            DMA_CHTIF3 = 0x00000400,
            DMA_CTEIF3 = 0x00000800,
            DMA_CGIF4 = 0x00001000,
            DMA_CTCIF4 = 0x00002000,
            DMA_CHTIF4 = 0x00004000,
            DMA_CTEIF4 = 0x00008000,
            DMA_CGIF5 = 0x00010000,
            DMA_CTCIF5 = 0x00020000,
            DMA_CHTIF5 = 0x00040000,
            DMA_CTEIF5 = 0x00080000,
            DMA_CGIF6 = 0x00100000,
            DMA_CTCIF6 = 0x00200000,
            DMA_CHTIF6 = 0x00400000,
            DMA_CTEIF6 = 0x00800000,
            DMA_CGIF7 = 0x01000000,
            DMA_CTCIF7 = 0x02000000,
            DMA_CHTIF7 = 0x04000000,
            DMA_CTEIF7 = 0x08000000,
        };

        /**
         * DMA_CFGR1
         */
        enum {
            DMA_CFGR1_EN = 0x0001,
            DMA_CFGR1_TCIE = 0x0002,
            DMA_CFGR1_HTIE = 0x0004,
            DMA_CFGR1_TEIE = 0x0008,
            DMA_CFGR1_DIR = 0x0010,
            DMA_CFGR1_CIRC = 0x0020,
            DMA_CFGR1_PINC = 0x0040,
            DMA_CFGR1_MINC = 0x0080,
            DMA_CFGR1_PSIZE = 0x0300,
            DMA_CFGR1_PSIZE_0 = 0x0100,
            DMA_CFGR1_PSIZE_1 = 0x0200,
            DMA_CFGR1_MSIZE = 0x0c00,
            DMA_CFGR1_MSIZE_0 = 0x0400,
            DMA_CFGR1_MSIZE_1 = 0x0800,
            DMA_CFGR1_PL = 0x3000,
            DMA_CFGR1_PL_0 = 0x1000,
            DMA_CFGR1_PL_1 = 0x2000,
            DMA_CFGR1_MEM2MEM = 0x4000,
        };

        /**
         * DMA_CFGR2
         */
        enum {
            DMA_CFGR2_EN = 0x0001,
            DMA_CFGR2_TCIE = 0x0002,
            DMA_CFGR2_HTIE = 0x0004,
            DMA_CFGR2_TEIE = 0x0008,
            DMA_CFGR2_DIR = 0x0010,
            DMA_CFGR2_CIRC = 0x0020,
            DMA_CFGR2_PINC = 0x0040,
            DMA_CFGR2_MINC = 0x0080,
            DMA_CFGR2_PSIZE = 0x0300,
            DMA_CFGR2_PSIZE_0 = 0x0100,
            DMA_CFGR2_PSIZE_1 = 0x0200,
            DMA_CFGR2_MSIZE = 0x0c00,
            DMA_CFGR2_MSIZE_0 = 0x0400,
            DMA_CFGR2_MSIZE_1 = 0x0800,
            DMA_CFGR2_PL = 0x3000,
            DMA_CFGR2_PL_0 = 0x1000,
            DMA_CFGR2_PL_1 = 0x2000,
            DMA_CFGR2_MEM2MEM = 0x4000,
        };

        /**
         * DMA_CFGR3
         */
        enum {
            DMA_CFGR3_EN = 0x0001,
            DMA_CFGR3_TCIE = 0x0002,
            DMA_CFGR3_HTIE = 0x0004,
            DMA_CFGR3_TEIE = 0x0008,
            DMA_CFGR3_DIR = 0x0010,
            DMA_CFGR3_CIRC = 0x0020,
            DMA_CFGR3_PINC = 0x0040,
            DMA_CFGR3_MINC = 0x0080,
            DMA_CFGR3_PSIZE = 0x0300,
            DMA_CFGR3_PSIZE_0 = 0x0100,
            DMA_CFGR3_PSIZE_1 = 0x0200,
            DMA_CFGR3_MSIZE = 0x0c00,
            DMA_CFGR3_MSIZE_0 = 0x0400,
            DMA_CFGR3_MSIZE_1 = 0x0800,
            DMA_CFGR3_PL = 0x3000,
            DMA_CFGR3_PL_0 = 0x1000,
            DMA_CFGR3_PL_1 = 0x2000,
            DMA_CFGR3_MEM2MEM = 0x4000,
        };

        /**
         * DMA_CFG4
         */
        enum {
            DMA_CFG4_EN = 0x0001,
            DMA_CFG4_TCIE = 0x0002,
            DMA_CFG4_HTIE = 0x0004,
            DMA_CFG4_TEIE = 0x0008,
            DMA_CFG4_DIR = 0x0010,
            DMA_CFG4_CIRC = 0x0020,
            DMA_CFG4_PINC = 0x0040,
            DMA_CFG4_MINC = 0x0080,
            DMA_CFG4_PSIZE = 0x0300,
            DMA_CFG4_PSIZE_0 = 0x0100,
            DMA_CFG4_PSIZE_1 = 0x0200,
            DMA_CFG4_MSIZE = 0x0c00,
            DMA_CFG4_MSIZE_0 = 0x0400,
            DMA_CFG4_MSIZE_1 = 0x0800,
            DMA_CFG4_PL = 0x3000,
            DMA_CFG4_PL_0 = 0x1000,
            DMA_CFG4_PL_1 = 0x2000,
            DMA_CFG4_MEM2MEM = 0x4000,
        };

        /**
         * DMA_CFG5
         */
        enum {
            DMA_CFG5_EN = 0x0001,
            DMA_CFG5_TCIE = 0x0002,
            DMA_CFG5_HTIE = 0x0004,
            DMA_CFG5_TEIE = 0x0008,
            DMA_CFG5_DIR = 0x0010,
            DMA_CFG5_CIRC = 0x0020,
            DMA_CFG5_PINC = 0x0040,
            DMA_CFG5_MINC = 0x0080,
            DMA_CFG5_PSIZE = 0x0300,
            DMA_CFG5_PSIZE_0 = 0x0100,
            DMA_CFG5_PSIZE_1 = 0x0200,
            DMA_CFG5_MSIZE = 0x0c00,
            DMA_CFG5_MSIZE_0 = 0x0400,
            DMA_CFG5_MSIZE_1 = 0x0800,
            DMA_CFG5_PL = 0x3000,
            DMA_CFG5_PL_0 = 0x1000,
            DMA_CFG5_PL_1 = 0x2000,
            DMA_CFG5_MEM2MEM = 0x4000,
        };

        /**
         * DMA_CFG6
         */
        enum {
            DMA_CFG6_EN = 0x0001,
            DMA_CFG6_TCIE = 0x0002,
            DMA_CFG6_HTIE = 0x0004,
            DMA_CFG6_TEIE = 0x0008,
            DMA_CFG6_DIR = 0x0010,
            DMA_CFG6_CIRC = 0x0020,
            DMA_CFG6_PINC = 0x0040,
            DMA_CFG6_MINC = 0x0080,
            DMA_CFG6_PSIZE = 0x0300,
            DMA_CFG6_PSIZE_0 = 0x0100,
            DMA_CFG6_PSIZE_1 = 0x0200,
            DMA_CFG6_MSIZE = 0x0c00,
            DMA_CFG6_MSIZE_0 = 0x0400,
            DMA_CFG6_MSIZE_1 = 0x0800,
            DMA_CFG6_PL = 0x3000,
            DMA_CFG6_PL_0 = 0x1000,
            DMA_CFG6_PL_1 = 0x2000,
            DMA_CFG6_MEM2MEM = 0x4000,
        };

        /**
         * DMA_CFG7
         */
        enum {
            DMA_CFG7_EN = 0x0001,
            DMA_CFG7_TCIE = 0x0002,
            DMA_CFG7_HTIE = 0x0004,
            DMA_CFG7_TEIE = 0x0008,
            DMA_CFG7_DIR = 0x0010,
            DMA_CFG7_CIRC = 0x0020,
            DMA_CFG7_PINC = 0x0040,
            DMA_CFG7_MINC = 0x0080,
            DMA_CFG7_PSIZE = 0x0300,
            DMA_CFG7_PSIZE_0 = 0x0100,
            DMA_CFG7_PSIZE_1 = 0x0200,
            DMA_CFG7_MSIZE = 0x0c00,
            DMA_CFG7_MSIZE_0 = 0x0400,
            DMA_CFG7_MSIZE_1 = 0x0800,
            DMA_CFG7_PL = 0x3000,
            DMA_CFG7_PL_0 = 0x1000,
            DMA_CFG7_PL_1 = 0x2000,
            DMA_CFG7_MEM2MEM = 0x4000,
        };

        /**
         * DMA_CNTR1
         */
        enum {
            DMA_CNTR1_NDT = 0xffff,
        };

        /**
         * DMA_CNTR2
         */
        enum {
            DMA_CNTR2_NDT = 0xffff,
        };

        /**
         * DMA_CNTR3
         */
        enum {
            DMA_CNTR3_NDT = 0xffff,
        };

        /**
         * DMA_CNTR4
         */
        enum {
            DMA_CNTR4_NDT = 0xffff,
        };

        /**
         * DMA_CNTR5
         */
        enum {
            DMA_CNTR5_NDT = 0xffff,
        };

        /**
         * DMA_CNTR6
         */
        enum {
            DMA_CNTR6_NDT = 0xffff,
        };

        /**
         * DMA_CNTR7
         */
        enum {
            DMA_CNTR7_NDT = 0xffff,
        };

        /**
         * DMA_PADDR1
         */
        enum {
            DMA_PADDR1_PA = 0xffffffff,
        };

        /**
         * DMA_PADDR2
         */
        enum {
            DMA_PADDR2_PA = 0xffffffff,
        };

        /**
         * DMA_PADDR3
         */
        enum {
            DMA_PADDR3_PA = 0xffffffff,
        };

        /**
         * DMA_PADDR4
         */
        enum {
            DMA_PADDR4_PA = 0xffffffff,
        };

        /**
         * DMA_PADDR5
         */
        enum {
            DMA_PADDR5_PA = 0xffffffff,
        };

        /**
         * DMA_PADDR6
         */
        enum {
            DMA_PADDR6_PA = 0xffffffff,
        };

        /**
         * DMA_PADDR7
         */
        enum {
            DMA_PADDR7_PA = 0xffffffff,
        };

        /**
         * DMA_MADDR1
         */
        enum {
            DMA_MADDR1_MA = 0xffffffff,
        };

        /**
         * DMA_MADDR2
         */
        enum {
            DMA_MADDR2_MA = 0xffffffff,
        };

        /**
         * DMA_MADDR3
         */
        enum {
            DMA_MADDR3_MA = 0xffffffff,
        };

        /**
         * DMA_MADDR4
         */
        enum {
            DMA_MADDR4_MA = 0xffffffff,
        };

        /**
         * DMA_MADDR5
         */
        enum {
            DMA_MADDR5_MA = 0xffffffff,
        };

        /**
         * DMA_MADDR6
         */
        enum {
            DMA_MADDR6_MA = 0xffffffff,
        };

        /**
         * DMA_MADDR7
         */
        enum {
            DMA_MADDR7_MA = 0xffffffff,
        };

        /**
         * EXTI_INTENR
         */
        enum {
            EXTI_INTENR_MR0 = 0x00000001,
            EXTI_INTENR_MR1 = 0x00000002,
            EXTI_INTENR_MR2 = 0x00000004,
            EXTI_INTENR_MR3 = 0x00000008,
            EXTI_INTENR_MR4 = 0x00000010,
            EXTI_INTENR_MR5 = 0x00000020,
            EXTI_INTENR_MR6 = 0x00000040,
            EXTI_INTENR_MR7 = 0x00000080,
            EXTI_INTENR_MR8 = 0x00000100,
            EXTI_INTENR_MR9 = 0x00000200,
        };

        /**
         * EXTI_EVENR
         */
        enum {
            EXTI_EVENR_MR0 = 0x00000001,
            EXTI_EVENR_MR1 = 0x00000002,
            EXTI_EVENR_MR2 = 0x00000004,
            EXTI_EVENR_MR3 = 0x00000008,
            EXTI_EVENR_MR4 = 0x00000010,
            EXTI_EVENR_MR5 = 0x00000020,
            EXTI_EVENR_MR6 = 0x00000040,
            EXTI_EVENR_MR7 = 0x00000080,
            EXTI_EVENR_MR8 = 0x00000100,
            EXTI_EVENR_MR9 = 0x00000200,
        };

        /**
         * EXTI_RTENR
         */
        enum {
            EXTI_RTENR_TR0 = 0x00000001,
            EXTI_RTENR_TR1 = 0x00000002,
            EXTI_RTENR_TR2 = 0x00000004,
            EXTI_RTENR_TR3 = 0x00000008,
            EXTI_RTENR_TR4 = 0x00000010,
            EXTI_RTENR_TR5 = 0x00000020,
            EXTI_RTENR_TR6 = 0x00000040,
            EXTI_RTENR_TR7 = 0x00000080,
            EXTI_RTENR_TR8 = 0x00000100,
            EXTI_RTENR_TR9 = 0x00000200,
        };

        /**
         * EXTI_FTENR
         */
        enum {
            EXTI_FTENR_TR0 = 0x00000001,
            EXTI_FTENR_TR1 = 0x00000002,
            EXTI_FTENR_TR2 = 0x00000004,
            EXTI_FTENR_TR3 = 0x00000008,
            EXTI_FTENR_TR4 = 0x00000010,
            EXTI_FTENR_TR5 = 0x00000020,
            EXTI_FTENR_TR6 = 0x00000040,
            EXTI_FTENR_TR7 = 0x00000080,
            EXTI_FTENR_TR8 = 0x00000100,
            EXTI_FTENR_TR9 = 0x00000200,
        };

        /**
         * EXTI_SWIEVR
         */
        enum {
            EXTI_SWIEVR_SWIEVR0 = 0x00000001,
            EXTI_SWIEVR_SWIEVR1 = 0x00000002,
            EXTI_SWIEVR_SWIEVR2 = 0x00000004,
            EXTI_SWIEVR_SWIEVR3 = 0x00000008,
            EXTI_SWIEVR_SWIEVR4 = 0x00000010,
            EXTI_SWIEVR_SWIEVR5 = 0x00000020,
            EXTI_SWIEVR_SWIEVR6 = 0x00000040,
            EXTI_SWIEVR_SWIEVR7 = 0x00000080,
            EXTI_SWIEVR_SWIEVR8 = 0x00000100,
            EXTI_SWIEVR_SWIEVR9 = 0x00000200,
        };

        /**
         * EXTI_INTFR
         */
        enum {
            EXTI_INTF_INTF0 = 0x00000001,
            EXTI_INTF_INTF1 = 0x00000002,
            EXTI_INTF_INTF2 = 0x00000004,
            EXTI_INTF_INTF3 = 0x00000008,
            EXTI_INTF_INTF4 = 0x00000010,
            EXTI_INTF_INTF5 = 0x00000020,
            EXTI_INTF_INTF6 = 0x00000040,
            EXTI_INTF_INTF7 = 0x00000080,
            EXTI_INTF_INTF8 = 0x00000100,
            EXTI_INTF_INTF9 = 0x00000200,
        };

        /**
         * FLASH_ACTLR
         */
        enum {
            FLASH_ACTLR_LATENCY = 0x03,
            FLASH_ACTLR_LATENCY_0 = 0x00,
            FLASH_ACTLR_LATENCY_1 = 0x01,
            FLASH_ACTLR_LATENCY_2 = 0x02,
        };

        /**
         * FLASH_KEYR
         */
        enum {
            FLASH_KEYR_FKEYR = 0xffffffff,
        };

        /**
         * FLASH_OBKEYR
         */
        enum {
            FLASH_OBKEYR_OBKEYR = 0xffffffff,
        };

        /**
         * FLASH_STATR
         */
        enum {
            FLASH_STATR_BSY = 0x01,
            FLASH_STATR_WRPRTERR = 0x10,
            FLASH_STATR_EOP = 0x20,
            FLASH_STATR_MODE = 0x4000,
            FLASH_STATR_LOCK = 0x8000,
        };

        /**
         * FLASH_CTLR
         */
        enum {
            FLASH_CTLR_PG = 0x0001,
            FLASH_CTLR_PER = 0x0002,
            FLASH_CTLR_MER = 0x0004,
            FLASH_CTLR_OPTPG = 0x0010,
            FLASH_CTLR_OPTER = 0x0020,
            FLASH_CTLR_STRT = 0x0040,
            FLASH_CTLR_LOCK = 0x0080,
            FLASH_CTLR_OPTWRE = 0x0200,
            FLASH_CTLR_ERRIE = 0x0400,
            FLASH_CTLR_EOPIE = 0x1000,
            FLASH_CTLR_FLOCK = 0x8000,
            FLASH_CTLR_PAGE_PG = 0x00010000,
            FLASH_CTLR_PAGE_ER = 0x00020000,
            FLASH_CTLR_BUF_LOAD = 0x00040000,
            FLASH_CTLR_BUF_RST = 0x00080000,
        };

        /**
         * FLASH_ADDR
         */
        enum {
            FLASH_ADDR_FAR = 0xffffffff,
        };

        /**
         * FLASH_OBR
         */
        enum {
            FLASH_OBR_OPTERR = 0x0001,
            FLASH_OBR_RDPRT = 0x0002,
            FLASH_OBR_USER = 0x03fc,
            FLASH_OBR_WDG_SW = 0x0004,
            FLASH_OBR_nRST_STOP = 0x0008,
            FLASH_OBR_STANDY_RST = 0x0010,
            FLASH_OBR_nRST_STDBY = 0x0010,
            FLASH_OBR_RST_MODE = 0x0060,
            FLASH_OBR_STATR_MODE = 0x0080,
            FLASH_OBR_FIX_11 = 0x0300,
        };

        /**
         * FLASH_WPR
         */
        enum {
            FLASH_WPR_WRP = 0xffffffff,
        };

        /**
         * FLASH_RDPR
         */
        enum {
            FLASH_RDPR_RDPR = 0x000000ff,
            FLASH_RDPR_nRDPR = 0x0000ff00,
        };

        /**
         * FLASH_USER
         */
        enum {
            FLASH_USER_USER = 0x00ff0000,
            FLASH_USER_nUSER = 0xff000000,
        };

        /**
         * FLASH_Data0
         */
        enum {
            FLASH_Data0_Data0 = 0x000000ff,
            FLASH_Data0_nData0 = 0x0000ff00,
        };

        /**
         * FLASH_Data1
         */
        enum {
            FLASH_Data1_Data1 = 0x00ff0000,
            FLASH_Data1_nData1 = 0xff000000,
        };

        /**
         * FLASH_WRPR0
         */
        enum {
            FLASH_WRPR0_WRPR0 = 0x000000ff,
            FLASH_WRPR0_nWRPR0 = 0x0000ff00,
        };

        /**
         * FLASH_WRPR1
         */
        enum {
            FLASH_WRPR1_WRPR1 = 0x00ff0000,
            FLASH_WRPR1_nWRPR1 = 0xff000000,
        };

        /**
         * FLASH_MODEKEYR
         */
        enum {
            FLASH_MODEKEYR_KEY1 = 0x45670123,
            FLASH_MODEKEYR_KEY2 = 0xcdef89ab,
        };

        /**
         * FLASH__BOOT_MODEKEYR
         */
        enum {
            FLASH_BOOT_MODEKEYR_KEY1 = 0x45670123,
            FLASH_BOOT_MODEKEYR_KEY2 = 0xcdef89ab,
        };

        /**
         * GPIO_CFGLR
         */
        enum {
            GPIO_CFGLR_MODE = 0x33333333,
            GPIO_CFGLR_MODE0 = 0x00000003,
            GPIO_CFGLR_MODE0_0 = 0x00000001,
            GPIO_CFGLR_MODE0_1 = 0x00000002,
            GPIO_CFGLR_MODE1 = 0x00000030,
            GPIO_CFGLR_MODE1_0 = 0x00000010,
            GPIO_CFGLR_MODE1_1 = 0x00000020,
            GPIO_CFGLR_MODE2 = 0x00000300,
            GPIO_CFGLR_MODE2_0 = 0x00000100,
            GPIO_CFGLR_MODE2_1 = 0x00000200,
            GPIO_CFGLR_MODE3 = 0x00003000,
            GPIO_CFGLR_MODE3_0 = 0x00001000,
            GPIO_CFGLR_MODE3_1 = 0x00002000,
            GPIO_CFGLR_MODE4 = 0x00030000,
            GPIO_CFGLR_MODE4_0 = 0x00010000,
            GPIO_CFGLR_MODE4_1 = 0x00020000,
            GPIO_CFGLR_MODE5 = 0x00300000,
            GPIO_CFGLR_MODE5_0 = 0x00100000,
            GPIO_CFGLR_MODE5_1 = 0x00200000,
            GPIO_CFGLR_MODE6 = 0x03000000,
            GPIO_CFGLR_MODE6_0 = 0x01000000,
            GPIO_CFGLR_MODE6_1 = 0x02000000,
            GPIO_CFGLR_MODE7 = 0x30000000,
            GPIO_CFGLR_MODE7_0 = 0x10000000,
            GPIO_CFGLR_MODE7_1 = 0x20000000,
            GPIO_CFGLR_CNF = 0xcccccccc,
            GPIO_CFGLR_CNF0 = 0x0000000c,
            GPIO_CFGLR_CNF0_0 = 0x00000004,
            GPIO_CFGLR_CNF0_1 = 0x00000008,
            GPIO_CFGLR_CNF1 = 0x000000c0,
            GPIO_CFGLR_CNF1_0 = 0x00000040,
            GPIO_CFGLR_CNF1_1 = 0x00000080,
            GPIO_CFGLR_CNF2 = 0x00000c00,
            GPIO_CFGLR_CNF2_0 = 0x00000400,
            GPIO_CFGLR_CNF2_1 = 0x00000800,
            GPIO_CFGLR_CNF3 = 0x0000c000,
            GPIO_CFGLR_CNF3_0 = 0x00004000,
            GPIO_CFGLR_CNF3_1 = 0x00008000,
            GPIO_CFGLR_CNF4 = 0x000c0000,
            GPIO_CFGLR_CNF4_0 = 0x00040000,
            GPIO_CFGLR_CNF4_1 = 0x00080000,
            GPIO_CFGLR_CNF5 = 0x00c00000,
            GPIO_CFGLR_CNF5_0 = 0x00400000,
            GPIO_CFGLR_CNF5_1 = 0x00800000,
            GPIO_CFGLR_CNF6 = 0x0c000000,
            GPIO_CFGLR_CNF6_0 = 0x04000000,
            GPIO_CFGLR_CNF6_1 = 0x08000000,
            GPIO_CFGLR_CNF7 = 0xc0000000,
            GPIO_CFGLR_CNF7_0 = 0x40000000,
            GPIO_CFGLR_CNF7_1 = 0x80000000,
        };

        /**
         * GPIO_INDR
         */
        enum {
            GPIO_INDR_IDR0 = 0x0001,
            GPIO_INDR_IDR1 = 0x0002,
            GPIO_INDR_IDR2 = 0x0004,
            GPIO_INDR_IDR3 = 0x0008,
            GPIO_INDR_IDR4 = 0x0010,
            GPIO_INDR_IDR5 = 0x0020,
            GPIO_INDR_IDR6 = 0x0040,
            GPIO_INDR_IDR7 = 0x0080,
        };

        /**
         * GPIO_OUTDR
         */
        enum {
            GPIO_OUTDR_ODR0 = 0x0001,
            GPIO_OUTDR_ODR1 = 0x0002,
            GPIO_OUTDR_ODR2 = 0x0004,
            GPIO_OUTDR_ODR3 = 0x0008,
            GPIO_OUTDR_ODR4 = 0x0010,
            GPIO_OUTDR_ODR5 = 0x0020,
            GPIO_OUTDR_ODR6 = 0x0040,
            GPIO_OUTDR_ODR7 = 0x0080,
        };

        /**
         * GPIO_BSHR
         */
        enum {
            GPIO_BSHR_BS0 = 0x00000001,
            GPIO_BSHR_BS1 = 0x00000002,
            GPIO_BSHR_BS2 = 0x00000004,
            GPIO_BSHR_BS3 = 0x00000008,
            GPIO_BSHR_BS4 = 0x00000010,
            GPIO_BSHR_BS5 = 0x00000020,
            GPIO_BSHR_BS6 = 0x00000040,
            GPIO_BSHR_BS7 = 0x00000080,
            GPIO_BSHR_BR0 = 0x00010000,
            GPIO_BSHR_BR1 = 0x00020000,
            GPIO_BSHR_BR2 = 0x00040000,
            GPIO_BSHR_BR3 = 0x00080000,
            GPIO_BSHR_BR4 = 0x00100000,
            GPIO_BSHR_BR5 = 0x00200000,
            GPIO_BSHR_BR6 = 0x00400000,
            GPIO_BSHR_BR7 = 0x00800000,
        };

        /**
         * GPIO_BCR
         */
        enum {
            GPIO_BCR_BR0 = 0x0001,
            GPIO_BCR_BR1 = 0x0002,
            GPIO_BCR_BR2 = 0x0004,
            GPIO_BCR_BR3 = 0x0008,
            GPIO_BCR_BR4 = 0x0010,
            GPIO_BCR_BR5 = 0x0020,
            GPIO_BCR_BR6 = 0x0040,
            GPIO_BCR_BR7 = 0x0080,
        };

        /**
         * GPIO_LCKR
         */
        enum {
            GPIO_LCK0 = 0x00000001,
            GPIO_LCK1 = 0x00000002,
            GPIO_LCK2 = 0x00000004,
            GPIO_LCK3 = 0x00000008,
            GPIO_LCK4 = 0x00000010,
            GPIO_LCK5 = 0x00000020,
            GPIO_LCK6 = 0x00000040,
            GPIO_LCK7 = 0x00000080,
            GPIO_LCKK = 0x00000100,
        };

        /**
         * AFIO_PCFR1register
         */
        enum {
            AFIO_PCFR1_SPI1_REMAP = 0x00000001,
            AFIO_PCFR1_I2C1_REMAP = 0x00000002,
            AFIO_PCFR1_USART1_REMAP = 0x00000004,
            AFIO_PCFR1_TIM1_REMAP = 0x000000c0,
            AFIO_PCFR1_TIM1_REMAP_0 = 0x00000040,
            AFIO_PCFR1_TIM1_REMAP_1 = 0x00000080,
            AFIO_PCFR1_TIM1_REMAP_NOREMAP = 0x00000000,
            AFIO_PCFR1_TIM1_REMAP_PARTIALREMAP = 0x00000040,
            AFIO_PCFR1_TIM1_REMAP_PARTIALREMAP_1 = 0x00000080,
            AFIO_PCFR1_TIM1_REMAP_FULLREMAP = 0x000000c0,
            AFIO_PCFR1_TIM2_REMAP = 0x00000300,
            AFIO_PCFR1_TIM2_REMAP_0 = 0x00000100,
            AFIO_PCFR1_TIM2_REMAP_1 = 0x00000200,
            AFIO_PCFR1_TIM2_REMAP_NOREMAP = 0x00000000,
            AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP1 = 0x00000100,
            AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP2 = 0x00000200,
            AFIO_PCFR1_TIM2_REMAP_FULLREMAP = 0x00000300,
            AFIO_PCFR1_PA12_REMAP = 0x00008000,
            AFIO_PCFR1_ADC1_ETRGINJ_REMAP = 0x00020000,
            AFIO_PCFR1_ADC1_ETRGREG_REMAP = 0x00040000,
            AFIO_PCFR1_USART1_HIGH_BIT_REMAP = 0x00200000,
            AFIO_PCFR1_I2C1_HIGH_BIT_REMAP = 0x00400000,
            AFIO_PCFR1_TIM1_1_RM = 0x00800000,
            AFIO_PCFR1_SWJ_CFG = 0x07000000,
            AFIO_PCFR1_SWJ_CFG_0 = 0x01000000,
            AFIO_PCFR1_SWJ_CFG_1 = 0x02000000,
            AFIO_PCFR1_SWJ_CFG_2 = 0x04000000,
            AFIO_PCFR1_SWJ_CFG_RESET = 0x00000000,
            AFIO_PCFR1_SWJ_CFG_NOJNTRST = 0x01000000,
            AFIO_PCFR1_SWJ_CFG_JTAGDISABLE = 0x02000000,
            AFIO_PCFR1_SWJ_CFG_DISABLE = 0x04000000,
        };

        /**
         * AFIO_EXTICR1
         */
        enum {
            AFIO_EXTICR1_EXTI0 = 0x0003,
            AFIO_EXTICR1_EXTI1 = 0x000c,
            AFIO_EXTICR1_EXTI2 = 0x0030,
            AFIO_EXTICR1_EXTI3 = 0x00c0,
            AFIO_EXTICR1_EXTI4 = 0x0300,
            AFIO_EXTICR1_EXTI5 = 0x0c00,
            AFIO_EXTICR1_EXTI6 = 0x3000,
            AFIO_EXTICR1_EXTI7 = 0xc000,
            AFIO_EXTICR1_EXTI0_PA = 0x0000,
            AFIO_EXTICR1_EXTI0_PC = 0x0002,
            AFIO_EXTICR1_EXTI0_PD = 0x0003,
            AFIO_EXTICR1_EXTI1_PA = 0x0000,
            AFIO_EXTICR1_EXTI1_PC = 0x0008,
            AFIO_EXTICR1_EXTI1_PD = 0x000c,
            AFIO_EXTICR1_EXTI2_PA = 0x0000,
            AFIO_EXTICR1_EXTI2_PC = 0x0020,
            AFIO_EXTICR1_EXTI2_PD = 0x0030,
            AFIO_EXTICR1_EXTI3_PA = 0x0000,
            AFIO_EXTICR1_EXTI3_PC = 0x0080,
            AFIO_EXTICR1_EXTI3_PD = 0x00c0,
            AFIO_EXTICR1_EXTI4_PA = 0x0000,
            AFIO_EXTICR1_EXTI4_PC = 0x0200,
            AFIO_EXTICR1_EXTI4_PD = 0x0300,
            AFIO_EXTICR1_EXTI5_PA = 0x0000,
            AFIO_EXTICR1_EXTI5_PC = 0x0800,
            AFIO_EXTICR1_EXTI5_PD = 0x0c00,
            AFIO_EXTICR1_EXTI6_PA = 0x0000,
            AFIO_EXTICR1_EXTI6_PC = 0x2000,
            AFIO_EXTICR1_EXTI6_PD = 0x3000,
            AFIO_EXTICR1_EXTI7_PA = 0x0000,
            AFIO_EXTICR1_EXTI7_PC = 0x8000,
            AFIO_EXTICR1_EXTI7_PD = 0xc000,
        };

        /**
         * IWDG_CTLR
         */
        enum {
            IWDG_KEY = 0xffff,
        };

        /**
         * IWDG_PSCR
         */
        enum {
            IWDG_PR = 0x07,
            IWDG_PR_0 = 0x01,
            IWDG_PR_1 = 0x02,
            IWDG_PR_2 = 0x04,
        };

        /**
         * IWDG_RLDR
         */
        enum {
            IWDG_RL = 0x0fff,
        };

        /**
        * IWDG_STATR
        */
        enum {
            IWDG_PVU = 0x01,
            IWDG_RVU = 0x02,
        };

        /**
        * I2C_CTLR1
        */
        enum {
            I2C_CTLR1_PE = 0x0001,
            I2C_CTLR1_ENPEC = 0x0020,
            I2C_CTLR1_ENGC = 0x0040,
            I2C_CTLR1_NOSTRETCH = 0x0080,
            I2C_CTLR1_START = 0x0100,
            I2C_CTLR1_STOP = 0x0200,
            I2C_CTLR1_ACK = 0x0400,
            I2C_CTLR1_POS = 0x0800,
            I2C_CTLR1_PEC = 0x1000,
            I2C_CTLR1_SWRST = 0x8000,
        };

        /**
        * I2C_CTLR2
        */
        enum {
            I2C_CTLR2_FREQ = 0x003f,
            I2C_CTLR2_FREQ_0 = 0x0001,
            I2C_CTLR2_FREQ_1 = 0x0002,
            I2C_CTLR2_FREQ_2 = 0x0004,
            I2C_CTLR2_FREQ_3 = 0x0008,
            I2C_CTLR2_FREQ_4 = 0x0010,
            I2C_CTLR2_FREQ_5 = 0x0020,
            I2C_CTLR2_ITERREN = 0x0100,
            I2C_CTLR2_ITEVTEN = 0x0200,
            I2C_CTLR2_ITBUFEN = 0x0400,
            I2C_CTLR2_DMAEN = 0x0800,
            I2C_CTLR2_LAST = 0x1000,
        };

        /**
        * I2C_OADDR1
        */
        enum {
            I2C_OADDR1_ADD1_7 = 0x00fe,
            I2C_OADDR1_ADD8_9 = 0x0300,
            I2C_OADDR1_ADD0 = 0x0001,
            I2C_OADDR1_ADD1 = 0x0002,
            I2C_OADDR1_ADD2 = 0x0004,
            I2C_OADDR1_ADD3 = 0x0008,
            I2C_OADDR1_ADD4 = 0x0010,
            I2C_OADDR1_ADD5 = 0x0020,
            I2C_OADDR1_ADD6 = 0x0040,
            I2C_OADDR1_ADD7 = 0x0080,
            I2C_OADDR1_ADD8 = 0x0100,
            I2C_OADDR1_ADD9 = 0x0200,
            I2C_OADDR1_ADDMODE = 0x8000,
        };

        /**
        * I2C_OADDR2
        */
        enum {
            I2C_OADDR2_ENDUAL = 0x01,
            I2C_OADDR2_ADD2 = 0xfe,
        };

        /**
        * I2C_DATAR
        */
        enum {
            I2C_DR_DATAR = 0xff,
        };

        /**
        * I2C_STAR1
        */
        enum {
            I2C_STAR1_SB = 0x0001,
            I2C_STAR1_ADDR = 0x0002,
            I2C_STAR1_BTF = 0x0004,
            I2C_STAR1_ADD10 = 0x0008,
            I2C_STAR1_STOPF = 0x0010,
            I2C_STAR1_RXNE = 0x0040,
            I2C_STAR1_TXE = 0x0080,
            I2C_STAR1_BERR = 0x0100,
            I2C_STAR1_ARLO = 0x0200,
            I2C_STAR1_AF = 0x0400,
            I2C_STAR1_OVR = 0x0800,
            I2C_STAR1_PECERR = 0x1000,
        };

        /**
        * I2C_STAR2
        */
        enum {
            I2C_STAR2_MSL = 0x0001,
            I2C_STAR2_BUSY = 0x0002,
            I2C_STAR2_TRA = 0x0004,
            I2C_STAR2_GENCALL = 0x0010,
            I2C_STAR2_DUALF = 0x0080,
            I2C_STAR2_PEC = 0xff00,
        };

        /**
        * I2C_CKCFGR
        */
        enum {
            I2C_CKCFGR_CCR = 0x0fff,
            I2C_CKCFGR_DUTY = 0x4000,
            I2C_CKCFGR_FS = 0x8000,
        };

        /**
        * PWR_CTLR
        */
        enum {
            PWR_CTLR_PDDS = 0x0002,
            PWR_CTLR_PVDE = 0x0010,
            PWR_CTLR_PLS = 0x00e0,
            PWR_CTLR_PLS_0 = 0x0020,
            PWR_CTLR_PLS_1 = 0x0040,
            PWR_CTLR_PLS_2 = 0x0080,
            PWR_PVDLevel_0 = 0x0000,
            PWR_PVDLevel_1 = 0x0020,
            PWR_PVDLevel_2 = 0x0040,
            PWR_PVDLevel_3 = 0x0060,
            PWR_PVDLevel_4 = 0x0080,
            PWR_PVDLevel_5 = 0x00a0,
            PWR_PVDLevel_6 = 0x00c0,
            PWR_PVDLevel_7 = 0x00e0,
        };

        /**
        * PWR_AWUCSR
        */
        enum {
            PWR_AWUCSR_AWUEN = 0x0002,
        };

        /**
        * PWR_CSR
        */
        enum {
            PWR_CSR_PVDO = 0x0004,
        };

        /**
        * PWR_AWUWR
        */
        enum {
            PWR_AWUWR = 0x003f,
            PWR_AWUPSC = 0x000f,
            PWR_AWUPSC_0 = 0x0000,
            PWR_AWUPSC_2 = 0x0002,
            PWR_AWUPSC_4 = 0x0003,
            PWR_AWUPSC_8 = 0x0004,
            PWR_AWUPSC_16 = 0x0005,
            PWR_AWUPSC_32 = 0x0006,
            PWR_AWUPSC_64 = 0x0007,
            PWR_AWUPSC_128 = 0x0008,
            PWR_AWUPSC_256 = 0x0009,
            PWR_AWUPSC_512 = 0x000a,
            PWR_AWUPSC_1024 = 0x000b,
            PWR_AWUPSC_2048 = 0x000c,
            PWR_AWUPSC_4096 = 0x000d,
            PWR_AWUPSC_10240 = 0x000e,
            PWR_AWUPSC_61440 = 0x000f,
        };

        /**
        * RCC_CTLR
        */
        enum {
            RCC_HSION = 0x00000001,
            RCC_HSIRDY = 0x00000002,
            RCC_HSITRIM = 0x000000f8,
            RCC_HSICAL = 0x0000ff00,
            RCC_HSEON = 0x00010000,
            RCC_HSERDY = 0x00020000,
            RCC_HSEBYP = 0x00040000,
            RCC_CSSON = 0x00080000,
            RCC_PLLON = 0x01000000,
            RCC_PLLRDY = 0x02000000,
        };

        /**
        * RCC_CFGR0
        */
        enum {
            RCC_SW = 0x00000003,
            RCC_SW_0 = 0x00000001,
            RCC_SW_1 = 0x00000002,
            RCC_SW_HSI = 0x00000000,
            RCC_SW_HSE = 0x00000001,
            RCC_SW_PLL = 0x00000002,
            RCC_SWS = 0x0000000c,
            RCC_SWS_0 = 0x00000004,
            RCC_SWS_1 = 0x00000008,
            RCC_SWS_HSI = 0x00000000,
            RCC_SWS_HSE = 0x00000004,
            RCC_SWS_PLL = 0x00000008,
            RCC_HPRE = 0x000000f0,
            RCC_HPRE_0 = 0x00000010,
            RCC_HPRE_1 = 0x00000020,
            RCC_HPRE_2 = 0x00000040,
            RCC_HPRE_3 = 0x00000080,
            RCC_HPRE_DIV1 = 0x00000000,
            RCC_HPRE_DIV2 = 0x00000010,
            RCC_HPRE_DIV3 = 0x00000020,
            RCC_HPRE_DIV4 = 0x00000030,
            RCC_HPRE_DIV5 = 0x00000040,
            RCC_HPRE_DIV6 = 0x00000050,
            RCC_HPRE_DIV7 = 0x00000060,
            RCC_HPRE_DIV8 = 0x00000070,
            RCC_HPRE_DIV16 = 0x000000b0,
            RCC_HPRE_DIV32 = 0x000000c0,
            RCC_HPRE_DIV64 = 0x000000d0,
            RCC_HPRE_DIV128 = 0x000000e0,
            RCC_HPRE_DIV256 = 0x000000f0,
            RCC_ADCPRE = 0x0000f800,
            RCC_ADCPRE_0 = 0x00000800,
            RCC_ADCPRE_1 = 0x00001000,
            RCC_ADCPRE_2 = 0x00002000,
            RCC_ADCPRE_3 = 0x00004000,
            RCC_ADCPRE_4 = 0x00008000,
            RCC_ADCPRE_DIV2 = 0x00000000,
            RCC_ADCPRE_DIV4 = 0x00004000,
            RCC_ADCPRE_DIV6 = 0x00008000,
            RCC_ADCPRE_DIV8 = 0x0000c000,
            RCC_PLLSRC = 0x00010000,
            RCC_PLLSRC_HSI_Mul2 = 0x00000000,
            RCC_PLLSRC_HSE_Mul2 = 0x00010000,
            RCC_CFGR0_MCO = 0x07000000,
            RCC_MCO_0 = 0x01000000,
            RCC_MCO_1 = 0x02000000,
            RCC_MCO_2 = 0x04000000,
            RCC_MCO_NOCLOCK = 0x00000000,
            RCC_CFGR0_MCO_SYSCLK = 0x04000000,
            RCC_CFGR0_MCO_HSI = 0x05000000,
            RCC_CFGR0_MCO_HSE = 0x06000000,
            RCC_CFGR0_MCO_PLL = 0x07000000,
        };

        /**
        * RCC_INTR
        */
        enum {
            RCC_LSIRDYF = 0x00000001,
            RCC_HSIRDYF = 0x00000004,
            RCC_HSERDYF = 0x00000008,
            RCC_PLLRDYF = 0x00000010,
            RCC_CSSF = 0x00000080,
            RCC_LSIRDYIE = 0x00000100,
            RCC_HSIRDYIE = 0x00000400,
            RCC_HSERDYIE = 0x00000800,
            RCC_PLLRDYIE = 0x00001000,
            RCC_LSIRDYC = 0x00010000,
            RCC_HSIRDYC = 0x00040000,
            RCC_HSERDYC = 0x00080000,
            RCC_PLLRDYC = 0x00100000,
            RCC_CSSC = 0x00800000,
        };

        /**
        * RCC_APB2PRSTR
        */
        enum {
            RCC_AFIORST = 0x00000001,
            RCC_IOPARST = 0x00000004,
            RCC_IOPCRST = 0x00000010,
            RCC_IOPDRST = 0x00000020,
            RCC_ADC1RST = 0x00000200,
            RCC_TIM1RST = 0x00000800,
            RCC_SPI1RST = 0x00001000,
            RCC_USART1RST = 0x00004000,
        };

        /**
        * RCC_APB1PRSTR
        */
        enum {
            RCC_TIM2RST = 0x00000001,
            RCC_WWDGRST = 0x00000800,
            RCC_I2C1RST = 0x00200000,
            RCC_PWRRST = 0x10000000,
        };

        /**
        * RCC_AHBPCENR
        */
        enum {
            RCC_DMA1EN = 0x0001,
            RCC_SRAMEN = 0x0004,
        };

        /**
        * RCC_APB2PCENR
        */
        enum {
            RCC_AFIOEN = 0x00000001,
            RCC_IOPAEN = 0x00000004,
            RCC_IOPCEN = 0x00000010,
            RCC_IOPDEN = 0x00000020,
            RCC_ADC1EN = 0x00000200,
            RCC_TIM1EN = 0x00000800,
            RCC_SPI1EN = 0x00001000,
            RCC_USART1EN = 0x00004000,
        };

        /**
        * RCC_APB1PCENR
        */
        enum {
            RCC_TIM2EN = 0x00000001,
            RCC_WWDGEN = 0x00000800,
            RCC_I2C1EN = 0x00200000,
            RCC_PWREN = 0x10000000,
        };

        /**
        * RCC_RSTSCKR
        */
        enum {
            RCC_LSION = 0x00000001,
            RCC_LSIRDY = 0x00000002,
            RCC_RMVF = 0x01000000,
            RCC_PINRSTF = 0x04000000,
            RCC_PORRSTF = 0x08000000,
            RCC_SFTRSTF = 0x10000000,
            RCC_IWDGRSTF = 0x20000000,
            RCC_WWDGRSTF = 0x40000000,
            RCC_LPWRRSTF = 0x80000000,
        };

        /**
        * SPI_CTLR1
        */
        enum {
            SPI_CTLR1_CPHA = 0x0001,
            SPI_CTLR1_CPOL = 0x0002,
            SPI_CTLR1_MSTR = 0x0004,
            SPI_CTLR1_BR = 0x0038,
            SPI_CTLR1_BR_0 = 0x0008,
            SPI_CTLR1_BR_1 = 0x0010,
            SPI_CTLR1_BR_2 = 0x0020,
            SPI_CTLR1_SPE = 0x0040,
            SPI_CTLR1_LSBFIRST = 0x0080,
            SPI_CTLR1_SSI = 0x0100,
            SPI_CTLR1_SSM = 0x0200,
            SPI_CTLR1_RXONLY = 0x0400,
            SPI_CTLR1_DFF = 0x0800,
            SPI_CTLR1_CRCNEXT = 0x1000,
            SPI_CTLR1_CRCEN = 0x2000,
            SPI_CTLR1_BIDIOE = 0x4000,
            SPI_CTLR1_BIDIMODE = 0x8000,
        };

        /**
        * SPI_CTLR2
        */
        enum {
            SPI_CTLR2_RXDMAEN = 0x01,
            SPI_CTLR2_TXDMAEN = 0x02,
            SPI_CTLR2_SSOE = 0x04,
            SPI_CTLR2_ERRIE = 0x20,
            SPI_CTLR2_RXNEIE = 0x40,
            SPI_CTLR2_TXEIE = 0x80,
        };

        /**
        * SPI_STATR
        */
        enum {
            SPI_STATR_RXNE = 0x01,
            SPI_STATR_TXE = 0x02,
            SPI_STATR_CHSIDE = 0x04,
            SPI_STATR_UDR = 0x08,
            SPI_STATR_CRCERR = 0x10,
            SPI_STATR_MODF = 0x20,
            SPI_STATR_OVR = 0x40,
            SPI_STATR_BSY = 0x80,
        };

        /**
        * SPI_DATAR
        */
        enum {
            SPI_DATAR_DR = 0xffff,
        };

        /**
        * SPI_CRCR
        */
        enum {
            SPI_CRCR_CRCPOLY = 0xffff,
        };

        /**
        * SPI_RCRCR
        */
        enum {
            SPI_RCRCR_RXCRC = 0xffff,
        };

        /**
        * SPI_TCRCR
        */
        enum {
            SPI_TCRCR_TXCRC = 0xffff,
        };

        /**
        * SPI_HSCR
        */
        enum {
            SPI_HSCR_HSRXEN = 0x0001,
        };

        /**
        * TIM_CTLR1
        */
        enum {
            TIM_CEN = 0x0001,
            TIM_UDIS = 0x0002,
            TIM_URS = 0x0004,
            TIM_OPM = 0x0008,
            TIM_DIR = 0x0010,
            TIM_CMS = 0x0060,
            TIM_CMS_0 = 0x0020,
            TIM_CMS_1 = 0x0040,
            TIM_ARPE = 0x0080,
            TIM_CTLR1_CKD = 0x0300,
            TIM_CKD_0 = 0x0100,
            TIM_CKD_1 = 0x0200,
        };

        /**
        * TIM_CTLR2
        */
        enum {
            TIM_CCPC = 0x0001,
            TIM_CCUS = 0x0004,
            TIM_CCDS = 0x0008,
            TIM_MMS = 0x0070,
            TIM_MMS_0 = 0x0010,
            TIM_MMS_1 = 0x0020,
            TIM_MMS_2 = 0x0040,
            TIM_TI1S = 0x0080,
            TIM_OIS1 = 0x0100,
            TIM_OIS1N = 0x0200,
            TIM_OIS2 = 0x0400,
            TIM_OIS2N = 0x0800,
            TIM_OIS3 = 0x1000,
            TIM_OIS3N = 0x2000,
            TIM_OIS4 = 0x4000,
        };

        /**
        * TIM_SMCFGR
        */
        enum {
            TIM_SMS = 0x0007,
            TIM_SMS_0 = 0x0001,
            TIM_SMS_1 = 0x0002,
            TIM_SMS_2 = 0x0004,
            TIM_TS = 0x0070,
            TIM_TS_0 = 0x0010,
            TIM_TS_1 = 0x0020,
            TIM_TS_2 = 0x0040,
            TIM_MSM = 0x0080,
            TIM_ETF = 0x0f00,
            TIM_ETF_0 = 0x0100,
            TIM_ETF_1 = 0x0200,
            TIM_ETF_2 = 0x0400,
            TIM_ETF_3 = 0x0800,
            TIM_ETPS = 0x3000,
            TIM_ETPS_0 = 0x1000,
            TIM_ETPS_1 = 0x2000,
            TIM_ECE = 0x4000,
            TIM_ETP = 0x8000,
        };

        /**
        * TIM_DMAINTENR
        */
        enum {
            TIM_UIE = 0x0001,
            TIM_CC1IE = 0x0002,
            TIM_CC2IE = 0x0004,
            TIM_CC3IE = 0x0008,
            TIM_CC4IE = 0x0010,
            TIM_COMIE = 0x0020,
            TIM_TIE = 0x0040,
            TIM_BIE = 0x0080,
            TIM_UDE = 0x0100,
            TIM_CC1DE = 0x0200,
            TIM_CC2DE = 0x0400,
            TIM_CC3DE = 0x0800,
            TIM_CC4DE = 0x1000,
            TIM_COMDE = 0x2000,
            TIM_TDE = 0x4000,
        };

        /**
        * TIM_INTFR
        */
        enum {
            TIM_UIF = 0x0001,
            TIM_CC1IF = 0x0002,
            TIM_CC2IF = 0x0004,
            TIM_CC3IF = 0x0008,
            TIM_CC4IF = 0x0010,
            TIM_COMIF = 0x0020,
            TIM_TIF = 0x0040,
            TIM_BIF = 0x0080,
            TIM_CC1OF = 0x0200,
            TIM_CC2OF = 0x0400,
            TIM_CC3OF = 0x0800,
            TIM_CC4OF = 0x1000,
        };

        /**
        * TIM_SWEVGR
        */
        enum {
            TIM_UG = 0x01,
            TIM_CC1G = 0x02,
            TIM_CC2G = 0x04,
            TIM_CC3G = 0x08,
            TIM_CC4G = 0x10,
            TIM_COMG = 0x20,
            TIM_TG = 0x40,
            TIM_BG = 0x80,
        };

        /**
        * TIM_CHCTLR1
        */
        enum {
            TIM_CC1S = 0x0003,
            TIM_CC1S_0 = 0x0001,
            TIM_CC1S_1 = 0x0002,
            TIM_OC1FE = 0x0004,
            TIM_OC1PE = 0x0008,
            TIM_OC1M = 0x0070,
            TIM_OC1M_0 = 0x0010,
            TIM_OC1M_1 = 0x0020,
            TIM_OC1M_2 = 0x0040,
            TIM_OC1CE = 0x0080,
            TIM_CC2S = 0x0300,
            TIM_CC2S_0 = 0x0100,
            TIM_CC2S_1 = 0x0200,
            TIM_OC2FE = 0x0400,
            TIM_OC2PE = 0x0800,
            TIM_OC2M = 0x7000,
            TIM_OC2M_0 = 0x1000,
            TIM_OC2M_1 = 0x2000,
            TIM_OC2M_2 = 0x4000,
            TIM_OC2CE = 0x8000,
            TIM_IC1PSC = 0x000c,
            TIM_IC1PSC_0 = 0x0004,
            TIM_IC1PSC_1 = 0x0008,
            TIM_IC1F = 0x00f0,
            TIM_IC1F_0 = 0x0010,
            TIM_IC1F_1 = 0x0020,
            TIM_IC1F_2 = 0x0040,
            TIM_IC1F_3 = 0x0080,
            TIM_IC2PSC = 0x0c00,
            TIM_IC2PSC_0 = 0x0400,
            TIM_IC2PSC_1 = 0x0800,
            TIM_IC2F = 0xf000,
            TIM_IC2F_0 = 0x1000,
            TIM_IC2F_1 = 0x2000,
            TIM_IC2F_2 = 0x4000,
            TIM_IC2F_3 = 0x8000,
        };

        /**
        * TIM_CHCTLR2
        */
        enum {
            TIM_CC3S = 0x0003,
            TIM_CC3S_0 = 0x0001,
            TIM_CC3S_1 = 0x0002,
            TIM_OC3FE = 0x0004,
            TIM_OC3PE = 0x0008,
            TIM_OC3M = 0x0070,
            TIM_OC3M_0 = 0x0010,
            TIM_OC3M_1 = 0x0020,
            TIM_OC3M_2 = 0x0040,
            TIM_OC3CE = 0x0080,
            TIM_CC4S = 0x0300,
            TIM_CC4S_0 = 0x0100,
            TIM_CC4S_1 = 0x0200,
            TIM_OC4FE = 0x0400,
            TIM_OC4PE = 0x0800,
            TIM_OC4M = 0x7000,
            TIM_OC4M_0 = 0x1000,
            TIM_OC4M_1 = 0x2000,
            TIM_OC4M_2 = 0x4000,
            TIM_OC4CE = 0x8000,
            TIM_IC3PSC = 0x000c,
            TIM_IC3PSC_0 = 0x0004,
            TIM_IC3PSC_1 = 0x0008,
            TIM_IC3F = 0x00f0,
            TIM_IC3F_0 = 0x0010,
            TIM_IC3F_1 = 0x0020,
            TIM_IC3F_2 = 0x0040,
            TIM_IC3F_3 = 0x0080,
            TIM_IC4PSC = 0x0c00,
            TIM_IC4PSC_0 = 0x0400,
            TIM_IC4PSC_1 = 0x0800,
            TIM_IC4F = 0xf000,
            TIM_IC4F_0 = 0x1000,
            TIM_IC4F_1 = 0x2000,
            TIM_IC4F_2 = 0x4000,
            TIM_IC4F_3 = 0x8000,
        };

        /**
        * TIM_CCER
        */
        enum {
            TIM_CC1E = 0x0001,
            TIM_CC1P = 0x0002,
            TIM_CC1NE = 0x0004,
            TIM_CC1NP = 0x0008,
            TIM_CC2E = 0x0010,
            TIM_CC2P = 0x0020,
            TIM_CC2NE = 0x0040,
            TIM_CC2NP = 0x0080,
            TIM_CC3E = 0x0100,
            TIM_CC3P = 0x0200,
            TIM_CC3NE = 0x0400,
            TIM_CC3NP = 0x0800,
            TIM_CC4E = 0x1000,
            TIM_CC4P = 0x2000,
        };

        /**
        * TIM_CNT
        */
        enum {
            TIM_CNT = 0xffff,
        };

        /**
        * TIM_PSC
        */
        enum {
            TIM_PSC = 0xffff,
        };

        /**
        * TIM_ATRLR
        */
        enum {
            TIM_ARR = 0xffff,
        };

        /**
        * TIM_RPTCR
        */
        enum {
            TIM_REP = 0xff,
        };

        /**
        * TIM_CH1CVR
        */
        enum {
            TIM_CCR1 = 0xffff,
        };

        /**
        * TIM_CH2CVR
        */
        enum {
            TIM_CCR2 = 0xffff,
        };

        /**
        * TIM_CH3CVR
        */
        enum {
            TIM_CCR3 = 0xffff,
        };

        /**
        * TIM_CH4CVR
        */
        enum {
            TIM_CCR4 = 0xffff,
        };

        /**
        * TIM_BDTR
        */
        enum {
            TIM_DTG = 0x00ff,
            TIM_DTG_0 = 0x0001,
            TIM_DTG_1 = 0x0002,
            TIM_DTG_2 = 0x0004,
            TIM_DTG_3 = 0x0008,
            TIM_DTG_4 = 0x0010,
            TIM_DTG_5 = 0x0020,
            TIM_DTG_6 = 0x0040,
            TIM_DTG_7 = 0x0080,
            TIM_LOCK = 0x0300,
            TIM_LOCK_0 = 0x0100,
            TIM_LOCK_1 = 0x0200,
            TIM_OSSI = 0x0400,
            TIM_OSSR = 0x0800,
            TIM_BKE = 0x1000,
            TIM_BKP = 0x2000,
            TIM_AOE = 0x4000,
            TIM_MOE = 0x8000,
        };

        /**
        * TIM_DMACFGR
        */
        enum {
            TIM_DBA = 0x001f,
            TIM_DBA_0 = 0x0001,
            TIM_DBA_1 = 0x0002,
            TIM_DBA_2 = 0x0004,
            TIM_DBA_3 = 0x0008,
            TIM_DBA_4 = 0x0010,
            TIM_DBL = 0x1f00,
            TIM_DBL_0 = 0x0100,
            TIM_DBL_1 = 0x0200,
            TIM_DBL_2 = 0x0400,
            TIM_DBL_3 = 0x0800,
            TIM_DBL_4 = 0x1000,
        };

        /**
        * TIM_DMAADR
        */
        enum {
            TIM_DMAR_DMAB = 0xffff,
        };

        /**
        * USART_STATR
        */
        enum {
            USART_STATR_PE = 0x0001,
            USART_STATR_FE = 0x0002,
            USART_STATR_NE = 0x0004,
            USART_STATR_ORE = 0x0008,
            USART_STATR_IDLE = 0x0010,
            USART_STATR_RXNE = 0x0020,
            USART_STATR_TC = 0x0040,
            USART_STATR_TXE = 0x0080,
            USART_STATR_LBD = 0x0100,
            USART_STATR_CTS = 0x0200,
        };

        /**
        * USART_DATAR
        */
        enum {
            USART_DATAR_DR = 0x01ff,
        };

        /**
        * USART_BRR
        */
        enum {
            USART_BRR_DIV_Fraction = 0x000f,
            USART_BRR_DIV_Mantissa = 0xfff0,
        };

        /**
        * USART_CTLR1
        */
        enum {
            USART_CTLR1_SBK = 0x0001,
            USART_CTLR1_RWU = 0x0002,
            USART_CTLR1_RE = 0x0004,
            USART_CTLR1_TE = 0x0008,
            USART_CTLR1_IDLEIE = 0x0010,
            USART_CTLR1_RXNEIE = 0x0020,
            USART_CTLR1_TCIE = 0x0040,
            USART_CTLR1_TXEIE = 0x0080,
            USART_CTLR1_PEIE = 0x0100,
            USART_CTLR1_PS = 0x0200,
            USART_CTLR1_PCE = 0x0400,
            USART_CTLR1_WAKE = 0x0800,
            USART_CTLR1_M = 0x1000,
            USART_CTLR1_UE = 0x2000,
        };

        /**
        * USART_CTLR2
        */
        enum {
            USART_CTLR2_ADD = 0x000f,
            USART_CTLR2_LBDL = 0x0020,
            USART_CTLR2_LBDIE = 0x0040,
            USART_CTLR2_LBCL = 0x0100,
            USART_CTLR2_CPHA = 0x0200,
            USART_CTLR2_CPOL = 0x0400,
            USART_CTLR2_CLKEN = 0x0800,
            USART_CTLR2_STOP = 0x3000,
            USART_CTLR2_STOP_0 = 0x1000,
            USART_CTLR2_STOP_1 = 0x2000,
            USART_CTLR2_LINEN = 0x4000,
        };

        /**
        * USART_CTLR3
        */
        enum {
            USART_CTLR3_EIE = 0x0001,
            USART_CTLR3_IREN = 0x0002,
            USART_CTLR3_IRLP = 0x0004,
            USART_CTLR3_HDSEL = 0x0008,
            USART_CTLR3_NACK = 0x0010,
            USART_CTLR3_SCEN = 0x0020,
            USART_CTLR3_DMAR = 0x0040,
            USART_CTLR3_DMAT = 0x0080,
            USART_CTLR3_RTSE = 0x0100,
            USART_CTLR3_CTSE = 0x0200,
            USART_CTLR3_CTSIE = 0x0400,
        };

        /**
        * USART_GPR
        */
        enum {
            USART_GPR_PSC = 0x00ff,
            USART_GPR_PSC_0 = 0x0001,
            USART_GPR_PSC_1 = 0x0002,
            USART_GPR_PSC_2 = 0x0004,
            USART_GPR_PSC_3 = 0x0008,
            USART_GPR_PSC_4 = 0x0010,
            USART_GPR_PSC_5 = 0x0020,
            USART_GPR_PSC_6 = 0x0040,
            USART_GPR_PSC_7 = 0x0080,
            USART_GPR_GT = 0xff00,
        };

        /**
        * WWDG_CTLR
        */
        enum {
            WWDG_CTLR_T = 0x7f,
            WWDG_CTLR_T0 = 0x01,
            WWDG_CTLR_T1 = 0x02,
            WWDG_CTLR_T2 = 0x04,
            WWDG_CTLR_T3 = 0x08,
            WWDG_CTLR_T4 = 0x10,
            WWDG_CTLR_T5 = 0x20,
            WWDG_CTLR_T6 = 0x40,
            WWDG_CTLR_WDGA = 0x80,
        };

        /**
        * WWDG_CFGR
        */
        enum {
            WWDG_CFGR_W = 0x007f,
            WWDG_CFGR_W0 = 0x0001,
            WWDG_CFGR_W1 = 0x0002,
            WWDG_CFGR_W2 = 0x0004,
            WWDG_CFGR_W3 = 0x0008,
            WWDG_CFGR_W4 = 0x0010,
            WWDG_CFGR_W5 = 0x0020,
            WWDG_CFGR_W6 = 0x0040,
            WWDG_CFGR_WDGTB = 0x0180,
            WWDG_CFGR_WDGTB0 = 0x0080,
            WWDG_CFGR_WDGTB1 = 0x0100,
            WWDG_CFGR_EWI = 0x0200,
        };

        /**
        * WWDG_STATR
        */
        enum {
            WWDG_STATR_EWIF = 0x01,
        };

        /**
        * Enhanced
        */
        enum {
            EXTEN_LOCKUP_EN = 0x00000040,
            EXTEN_LOCKUP_RSTF = 0x00000080,
            EXTEN_LDO_TRIM = 0x00000400,
            EXTEN_OPA_EN = 0x00010000,
            EXTEN_OPA_NSEL = 0x00020000,
            EXTEN_OPA_PSEL = 0x00040000,
        };




    }
}

#endif