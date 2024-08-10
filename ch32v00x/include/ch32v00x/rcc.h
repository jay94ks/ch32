#ifndef __CH32V00X_RCC_H__
#define __CH32V00X_RCC_H__

#include "core/riscv.h"
#include "core/v00x.h"

namespace ch32 {

    /**
     * RCC.
     */
    namespace rcc {
        /**
         * External oscillator speed in HZ. 
         */
        constexpr uint32_t HseHz = 24000000;

        /**
         * Internal oscillator speed in HZ. 
         */
        constexpr uint32_t HsiHz = 24000000;

        /**
         * HSE startup timeout.
         */
        constexpr uint32_t HseStartupTimeout = 0x2000;

        /**
         * RCC Clock information.
         * This is exported from RCC registers.
         */
        typedef struct {
            uint32_t sysclk;
            uint32_t hclk;
            uint32_t pclk1;
            uint32_t pclk2;
            uint32_t adcclk;
        } RccClockInfo;

        /**
         * HSE configuration.
         */
        typedef enum {
            RCCHSE_OFF      = 0,
            RCCHSE_ON       = 0x10000,
            RCCHSE_BYPASS   = 0x40000
        } RccHse;

        /**
         * PLL entry clock source.
         */
        typedef enum {
            RCCPLL_HSI_MUL2 = 0,
            RCCPLL_HSE_MUL2 = 0x10000,
        } RccPllSrc;

        /**
         * System clock source. 
         */
        typedef enum {
            RCCSYS_HSI      = 0,
            RCCSYS_HSE      = 1,
            RCCSYS_PLLCLK   = 2,
        } RccSysSrc;

        /**
         * AHB clock source.
         */
        typedef enum {
            RCCAHB_SYSCLK_DIV_1    = 0,
            RCCAHB_SYSCLK_DIV_2    = 0x10,
            RCCAHB_SYSCLK_DIV_3    = 0x20,
            RCCAHB_SYSCLK_DIV_4    = 0x30,
            RCCAHB_SYSCLK_DIV_5    = 0x40,
            RCCAHB_SYSCLK_DIV_6    = 0x50,
            RCCAHB_SYSCLK_DIV_7    = 0x60,
            RCCAHB_SYSCLK_DIV_8    = 0x70,
            RCCAHB_SYSCLK_DIV_16   = 0xb0,
            RCCAHB_SYSCLK_DIV_32   = 0xc0,
            RCCAHB_SYSCLK_DIV_64   = 0xd0,
            RCCAHB_SYSCLK_DIV_128  = 0xe0,
            RCCAHB_SYSCLK_DIV_256  = 0xf0,
        } RccAhbSrc;

        /**
         * RCC interrupt source.
         */
        typedef enum {
            RCCINT_LSI = 0x01,
            RCCINT_HSI = 0x04,
            RCCINT_HSE = 0x08,
            RCCINT_PLL = 0x10,
            RCSINT_CSS = 0x80,
        } RccInt;

        /**
         * ADC clock source.
         */
        typedef enum {
            RCCADC_PCLK2_DIV2   = 0,
            RCCADC_PCLK2_DIV4   = 0x4000,
            RCCADC_PCLK2_DIV6   = 0x8000,
            RCCADC_PCLK2_DIV8   = 0xc000,
            RCCADC_PCLK2_DIV12  = 0xa000,
            RCCADC_PCLK2_DIV16  = 0xe000,
            RCCADC_PCLK2_DIV24  = 0xa800,
            RCCADC_PCLK2_DIV32  = 0xe800,
            RCCADC_PCLK2_DIV48  = 0xb000,
            RCCADC_PCLK2_DIV64  = 0xf000,
            RCCADC_PCLK2_DIV96  = 0xb800,
            RCCADC_PCLK2_DIV128 = 0xf800,
        } RccAdcSrc;

        /**
         * AHB peripherals.
         */
        typedef enum {
            RCCAHB_PERI_DMA1    = 0x01,
            RCCAHB_PERI_SRAM    = 0x04,
        } RccAhbPeri;

        /**
         * APB2 peripherals.
         */
        typedef enum {
            RCCAPB2_PERI_AFIO   = 0x0001,
            RCCAPB2_PERI_GPIOA  = 0x0004,
            RCCAPB2_PERI_GPIOC  = 0x0010,
            RCCAPB2_PERI_GPIOD  = 0x0020,
            RCCAPB2_PERI_ADC1   = 0x0200,
            RCCAPB2_PERI_TIM1   = 0x0800,
            RCCAPB2_PERI_SPI1   = 0x1000,
            RCCAPB2_PERI_USART1 = 0x4000,
        } RccApb2Peri;

        /**
         * APB1 peripherials.
         */
        typedef enum {
            RCCAPB1_PERI_TIM2   = 0x00000001,
            RCCAPB1_PERI_WWDG   = 0x00000800,
            RCCAPB1_PERI_I2C1   = 0x00200000,
            RCCAPB1_PERI_PWR    = 0x10000000
        } RccApb1Peri;

        /**
         * RCC -> MCO pin clock output.
         */
        typedef enum {
            RCCMCO_NO_CLOCK     = 0,
            RCCMCO_SYSCLK       = 0x04,
            RCCMCO_HSI          = 0x05,
            RCCMCO_HSE          = 0x06,
            RCCMCO_PLL          = 0x07,
        } RccMco;
        
        /**
         * RCC flags.
         */
        typedef enum {
            RCCFL_HSI_READY = 0x21,
            RCCFL_HSE_READY = 0x31,
            RCCFL_PLL_READY = 0x39,
            RCCFL_LSI_READY = 0x61,
            RCCFL_PIN_RESET = 0x7a,
            RCCFL_POR_RESET = 0x7b,
            RCCFL_SFT_RESET = 0x7c,
            RCCFL_IWDG_RESET = 0x7d,
            RCCFL_WWDG_RESET = 0x7e,
            RCCFL_LPWR_RESET = 0x7f,
        } RccFlags;

        /**
         * RCC SysTick clock source.
         */
        typedef enum {
            RCC_SYSTICK_HCLK_DIV8   = 0xfffffffb,
            RCC_SYSTICK_HCLK        = 0x00000004,
        } RccSysTick;

        /**
         * De-initialize the RCC.
         * Aka, Resets the RCC clock configuration to the default reset state.
         */
        void deinit();

        /**
         * Configure the external HSE oscillator.
         */
        void hse(RccHse hse);

        /**
         * Wait for HSE start-up.
         * @return true / false: HSE is stable and ready / not available or not ready.
         */
        bool waitHse();

        /**
         * Adjust the internal HSI calibration. 
         * @param calibration calibration trimming value: 0x00 ~ 0x1f.
         */
        void adjust(uint8_t calibration);

        /**
         * Enables or disables the HSI oscillator.
         * @param state RVFN_ENABLE or RVFN_DISABLE.
         */
        void hsiCtl(RvFunc state);

        /**
         * Configure the PLL clock source and multiplication factor.
         */
        void pll(RccPllSrc pll);

        /**
         * Enables or disables the HSI oscillator.
         * @param state RVFN_ENABLE or RVFN_DISABLE.
         */
        void pllCtl(RvFunc state);

        /**
         * Configure the SYSCLK.
         */
        void sysclk(RccSysSrc sys);

        /**
         * Get the system clock source.
         */
        RccSysSrc sysclk();

        /**
         * Configures the AHB clock (HCLK).
         */
        void hclk(RccAhbSrc ahb);

        /**
         * Configures the AHB clock (HCLK).
         * --> alias of `hclk(...)` function.
         */
        inline void ahbclk(RccAhbSrc ahb) {
            hclk(ahb);
        }

        /**
         * Enables or disables the specified RCC interrupts.
         * @param state RVFN_ENABLE or RVFN_DISABLE.
         */
        void intr(RccInt it, RvFunc state);

        /**
         * Configures the ADC clock.
         */
        void adcclk(RccAdcSrc adc);
        
        /**
         * Configures the ADC clock.
         * --> alias of `adcclk(...)` function.
         */
        inline void pclk2(RccAdcSrc adc) {
            adcclk(adc);
        }

        /**
         * Enables or disables the LSI oscillator.
         * @param state RVFN_ENABLE or RVFN_DISABLE.
         */
        void lsiCtl(RvFunc state);

        /**
         * Get the clock informations.
         * When HSE crystal uses fractional value, the result may be incorrect. 
         */
        void clocks(RccClockInfo* out);

        /**
         * Enables or disables the AHB peripheral clock.
         * @param state RVFN_ENABLE or RVFN_DISABLE.
         */
        void ahb(RccAhbPeri ahb, RvFunc state);

        /**
         * Enables or disables the APB2 peripheral clock.
         * @param state RVFN_ENABLE or RVFN_DISABLE.
         */
        void apb2(RccApb2Peri apb2, RvFunc state);

        /**
         * Enables or disables the APB1 peripheral clock.
         * @param state RVFN_ENABLE or RVFN_DISABLE.
         */
        void apb1(RccApb1Peri apb1, RvFunc state);

        /**
         * Forces or releases APB2 peripheral reset.
         * @param state RVFN_ENABLE or RVFN_DISABLE.
         */
        void resetApb2(RccApb2Peri apb2, RvFunc state);

        /**
         * Forces or releases APB1 peripheral reset.
         * @param state RVFN_ENABLE or RVFN_DISABLE.
         */
        void resetApb1(RccApb1Peri apb1, RvFunc state);

        /**
         * Enables or disables the clock security system.
         * @param state RVFN_ENABLE or RVFN_DISABLE.
         */
        void security(RvFunc state);

        /**
         * Selects the clock source to bypass to MCO pin.
         */
        void mco(RccMco mco);

        /**
         * Checks whether the specified RCC flag is set or not.
         */
        bool hasFlag(RccFlags flag);

        /**
         * Clears the RCC reset flags.
         */
        void clearFlag();

        /**
         * Checks whether the specified RCC interrupt has been occurred or not.
         */
        RvFlag intr(RccInt intr);

        /**
         * Clears the RCC's interrupt pending bits.
         */
        void clearIntr(RccInt intr);
    }
}

#endif