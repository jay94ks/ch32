#include <ch32v00x/power.h>
#include <ch32v00x/rcc.h>

using namespace ch32::v00x;

namespace ch32 {
namespace power {

    enum {
        CTLR_DS_MASK = 0xFFFFFFFD,
        CTLR_PLS_MASK = 0xFFFFFF1F,
    };

    enum {
        AWUPSC_MASK = 0xFFFFFFF0,
        AWUWR_MASK = 0xFFFFFFC0,
    };

    void deinit() {
        rcc::resetApb1(rcc::RCCAPB1_PERI_PWR, RVFN_ENABLE);
        rcc::resetApb1(rcc::RCCAPB1_PERI_PWR, RVFN_DISABLE);
    }

    void pvd(RvFunc state) {
        if (state) {
            RV_PWR->ctlr |= (1 << 4);
            return;
        }

        RV_PWR->ctlr &= ~(1 << 4);
    }

    void pvdSet(PwrPvdLevel pvd) {
        uint32_t tmp = RV_PWR->ctlr;

        tmp &= CTLR_PLS_MASK;
        tmp |= uint32_t (pvd);

        RV_PWR->ctlr = tmp;
    }

    void autoWakeUp(RvFunc state) {
        if (state) {
            RV_PWR->awucsr |= (1 << 1);
            return;
        }

        RV_PWR->awucsr &= ~(1 << 1);
    }

    void awuPrescalerSet(PwrAwu prescaler) {
        uint32_t tmp = RV_PWR->awupsc;
        
        tmp &= AWUPSC_MASK;
        tmp |= prescaler;

        RV_PWR->awupsc = tmp;
    }

    void awuWindowSet(uint8_t window) {
        uint32_t tmp = RV_PWR->awuwr;

        tmp &= AWUWR_MASK;
        tmp |= window;

        RV_PWR->awuwr = tmp;
    }

    void enter(PwrStopMode mode) {
        RV_PWR->ctlr &= CTLR_DS_MASK;
        RV_PWR->ctlr |= PWR_CTLR_PDDS;

        if (mode == PWRSTOP_WFI) {
            nvic::wfi();
        } else {
            nvic::wfe();
        }

        RV_NVIC->sctlr &= ~(1 << 2);
    }

    bool check() {
        const uint32_t PWR_FLAG_PVDO = 4;
        return (RV_PWR->csr & PWR_FLAG_PVDO) != 0;
    }
}
}