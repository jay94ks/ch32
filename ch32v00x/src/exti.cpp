#include <ch32v00x/exti.h>
#include <ch32v00x/rcc.h>

using namespace ch32::v00x;

namespace ch32 {
namespace exti {

    void deinit() {
        RV_EXTI->intenr = 0;
        RV_EXTI->evenr = 0;
        RV_EXTI->rtenr = 0;
        RV_EXTI->ftenr = 0;
        RV_EXTI->intfr = 0x000fffff;
    }

    void init(ExtiLine line, ExtiMode mode, ExtiTrigger trig, RvFunc state) {
        if (!state) {
            *((RV_RW uint32_t*) (exti_base + mode)) &= ~uint32_t(line);
            return;
        }

        RV_EXTI->intenr &= ~line;
        RV_EXTI->evenr &= ~line;
        
        *((RV_RW uint32_t*) (exti_base + mode)) |= uint32_t(line);

        RV_EXTI->rtenr &= ~line;
        RV_EXTI->ftenr &= ~line;

        if (trig != EXTI_TRIG_RISEFALL) {
            *((RV_RW uint32_t*)(exti_base + trig)) |= line;
        }

        else {
            RV_EXTI->rtenr |= line;
            RV_EXTI->ftenr |= line;
        }
    }

    void generate(ExtiLine line) {
        RV_EXTI->swievr |= line;
    }
    
    bool check(ExtiLine line) {
        return (RV_EXTI->intfr & line) != 0;
    }

    void clear(ExtiLine line) {
        RV_EXTI->intfr &= ~line;
    }

    bool intr(ExtiLine line) {
        if ((RV_EXTI->intenr & line) == 0) {
            return false;
        }

        return (RV_EXTI->intfr & line) != 0;
    }
}
}