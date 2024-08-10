#include <ch32v00x/rcc.h>

using namespace ch32::v00x;

namespace ch32 {
namespace rcc {

    enum {
        CTLR_HSE_BYPASS_RESET   = 0xFFFBFFFF,
        CTLR_HSE_BYPASS_SET     = 0x00040000,
        CTLR_HSE_ON_RESET       = 0xfffeffff,
        CTLR_HSE_ON_SET         = 0x00010000,
        CTLR_HSI_TRIM_MASK      = 0xffffff07,
    };

    enum {
        CFGR0_SWS_MASK          = 0x0000000C,
        CFGR0_SW_MASK           = 0xFFFFFFFC,
        CFGR0_PLLSRC_MASK       = 0x00010000,
        CFGR0_HPRE_RESET_MASK   = 0xFFFFFF0F,
        CFGR0_HPRE_SET_MASK     = 0x000000F0,
        CFGR0_ADCPRE_RESET_MASK = 0xFFFF07FF,
        CFGR0_ADCPRE_SET_MASK   = 0x0000F800,
    };

    enum {
        HSI_ADJUST_MAX_CALIBR = 0x1f
    };

    void deinit() {
        RV_RCC->ctlr |= 0x00000001;
        RV_RCC->cfgr0 &= 0xf8ff0000;
        RV_RCC->ctlr &= 0xfef6ffff;
        RV_RCC->ctlr &= 0xfffbffff;
        RV_RCC->cfgr0 &= 0xfffeffff;
        RV_RCC->intr = 0x009f0000;

        adjust(0x10);
    }

    void hse(RccHse hse) {
        RV_RCC->ctlr &= CTLR_HSE_ON_RESET;
        RV_RCC->ctlr &= CTLR_HSE_BYPASS_RESET;

        switch (hse) {
            case RCCHSE_ON:
                RV_RCC->ctlr |= CTLR_HSE_ON_SET;
                break;

            case RCCHSE_BYPASS:
                RV_RCC->ctlr |= CTLR_HSE_BYPASS_SET | CTLR_HSE_ON_SET;
                break;

            default:
                break;
        }
    }

    bool waitHse() {
        RV_RW uint32_t count = 0;

        do {
            if (hasFlag(RCCFL_HSE_READY)) {
                break;
            }

            count++;
        }

        while(count < HseStartupTimeout);
        return hasFlag(RCCFL_HSE_READY);
    }

    void adjust(uint8_t calibration) {
        if (calibration > HSI_ADJUST_MAX_CALIBR) {
            calibration = HSI_ADJUST_MAX_CALIBR;
        }

        uint32_t tmp = RV_RCC->ctlr;
        tmp &= CTLR_HSI_TRIM_MASK;
        tmp |= uint32_t(calibration) << 3;
        RV_RCC->ctlr = tmp;
    }

    void hsiCtl(RvFunc state) {
        if (state) {
            RV_RCC->ctlr |= 1 << 0;
            return;
        }

        RV_RCC->ctlr &= ~(1 << 0);
    }

    void pll(RccPllSrc pll) {
        constexpr uint32_t PLL_MASK = 0xFFFEFFFF;
        uint32_t tmp = RV_RCC->cfgr0;

        tmp &= PLL_MASK;
        tmp |= uint32_t(pll);

        RV_RCC->cfgr0 = tmp;
    }

    void pllCtl(RvFunc state) {
        if (state) {
            RV_RCC->ctlr |= 1 << 24;
            return;
        }

        RV_RCC->ctlr &= ~(1 << 24);
    }

    void sysclk(RccSysSrc sys) {
        const uint8_t calibr = *((RV_RD uint8_t*) cfg0_pll_trim);

        if (calibr != 0xff) {
            if (sys == RCCSYS_PLLCLK && (RV_RCC->cfgr0 & (1 << 16)) == RCC_PLLSRC_HSI_Mul2) {
                adjust(calibr & HSI_ADJUST_MAX_CALIBR);
            } else {
                adjust(0x10);
            }
        }

        uint32_t tmp = RV_RCC->cfgr0;
        tmp &= CFGR0_SW_MASK;
        tmp |= uint32_t(sys);
        RV_RCC->cfgr0 = tmp;
    }

    RccSysSrc sysclk() {
        return RccSysSrc(RV_RCC->cfgr0 & CFGR0_SW_MASK);
    }

    void hclk(RccAhbSrc ahb) {
        uint32_t tmp = RV_RCC->cfgr0;

        tmp &= CFGR0_HPRE_RESET_MASK;
        tmp |= uint32_t(ahb);

        RV_RCC->cfgr0 = tmp;
    }

    void intr(RccInt it, RvFunc state) {
        if (state) {
            *((RV_RW uint8_t*)0x40021009) |= uint8_t(it);
            return;
        }
        
        *((RV_RW uint8_t*)0x40021009) &= ~uint8_t(it);
    }
    
    void adcclk(RccAdcSrc adc) {
        uint32_t tmp = RV_RCC->cfgr0;

        tmp &= CFGR0_ADCPRE_RESET_MASK;
        tmp |= uint32_t(adc);

        RV_RCC->cfgr0 = tmp;
    }

    void lsiCtl(RvFunc state) {
        if (state) {
            RV_RCC->rstsckr |= (1 << 0);
            return;
        }

        RV_RCC->rstsckr &= ~(1 << 0);
    }

    void clocks(RccClockInfo* out) {
        uint32_t tmp = RV_RCC->cfgr0 & CFGR0_SWS_MASK;
        switch (tmp) {
            case 0x04: out->sysclk = HseHz; break;
            case 0x08:
                if ((RV_RCC->cfgr0 & CFGR0_PLLSRC_MASK) != 0) {
                    out->sysclk = HseHz << 1;
                    break;
                }

                out->sysclk = HsiHz << 1;
                break;
                
            case 0x00: default: 
                out->sysclk = HsiHz;
                break;
        }

        constexpr uint8_t BIT_3 = 0x08;
        constexpr uint8_t BIT_210 = 0x07;

        tmp = ((RV_RCC->cfgr0 & CFGR0_HPRE_SET_MASK) >> 4) & 0x0f;
        uint8_t prescaler = (tmp & BIT_210) + 1;  // --> 1 ~ 8.
        
        if (((RV_RCC->cfgr0 & CFGR0_HPRE_SET_MASK) >> 4) < 8) {
            out->hclk = out->sysclk / prescaler;
        } else {
            out->hclk = out->sysclk >> prescaler;
        }

        out->pclk1 = out->hclk;
        out->pclk2 = out->hclk;

        tmp = (RV_RCC->cfgr0 & CFGR0_ADCPRE_SET_MASK) >> 11;
        tmp = ((tmp & 0x18) >> 3) | ((tmp & 0x07) << 2);
        tmp = (tmp & 0x13) >= 4 ? (tmp - 12) : (tmp & 0x03);

        if (tmp <= 3) { // 2 4 6 8
            prescaler = 2 * (tmp + 1);
        }

        else if (tmp <= 7) { // 4 8 12 16
            prescaler = 4 * (tmp - 3);
        }

        else if (tmp <= 15) { // 16 32 48 64.
            prescaler = 16 * (tmp - 11);
        }

        else { // 32 64 96 128.
            prescaler = 32 * (tmp - 15);
        }

        out->adcclk = out->pclk2 / prescaler;
    }

    void ahb(RccAhbPeri ahb, RvFunc state) {
        if (state) {
            RV_RCC->ahbpcenr |= uint32_t(ahb);
            return;
        }
        
        RV_RCC->ahbpcenr &= ~uint32_t(ahb);
    }

    void apb2(RccApb2Peri apb2, RvFunc state) {
        if (state) {
            RV_RCC->apb2pcenr |= uint32_t(apb2);
            return;
        }
        
        RV_RCC->apb2pcenr &= ~uint32_t(apb2);
    }

    void apb1(RccApb1Peri apb1, RvFunc state) {
        if (state) {
            RV_RCC->apb1pcenr |= uint32_t(apb1);
            return;
        }
        
        RV_RCC->apb1pcenr &= ~uint32_t(apb1);
    }

    void resetApb2(RccApb2Peri apb2, RvFunc state) {
        if (state) {
            RV_RCC->apb2prstr |= uint32_t(apb2);
            return;
        }

        RV_RCC->apb2prstr &= ~uint32_t(apb2);
    }

    void resetApb1(RccApb1Peri apb1, RvFunc state) {
        if (state) {
            RV_RCC->apb1prstr |= uint32_t(apb1);
            return;
        }

        RV_RCC->apb1prstr &= ~uint32_t(apb1);
    }

    void security(RvFunc state) {
        if (state) {
            RV_RCC->ctlr |= (1 << 19);
            return;
        }

        RV_RCC->ctlr &= ~(1 << 19);
    }

    void mco(RccMco mco) {
        *((RV_RW uint8_t*)0x40021007) = mco;
    }

    bool hasFlag(RccFlags flag) {
        constexpr uint8_t FLAG_MASK = 0x1f;
        uint32_t reg;

        if ((flag >> 5) == 1) {
            reg = RV_RCC->ctlr;
        } else {
            reg = RV_RCC->rstsckr;
        }

        return (reg & (1 << (flag & FLAG_MASK))) != 0;
    }

    void clearFlag() {
        RV_RCC->rstsckr |= 0x01000000; // RSTSCKR_RMVF_Set.
    }
    
    RvFlag intr(RccInt intr) {
        if ((RV_RCC->intr & intr) != 0) {
            return RVFL_SET;
        }

        return RVFL_RESET;
    }

    void clearIntr(RccInt intr) {
        *((RV_RW uint8_t*) 0x4002100A) = intr;
    }
}
}
