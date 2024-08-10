#include <ch32v00x/core/riscv.h>

namespace ch32 {
namespace riscv {

#ifdef RV_GET_REG
#undef RV_GET_REG
#endif

#ifdef RV_SET_REG
#undef RV_SET_REG
#endif

#define RV_GET_REG(name, ...) \
    uint32_t name() { \
        uint32_t val; \
        RV_ASM(__VA_ARGS__: "=r"(val)); \
        return val; \
    }

#define RV_SET_REG(name, ...) \
    void name(uint32_t val) { \
        RV_ASM(__VA_ARGS__ : : "r"(val)); \
    }

    RV_GET_REG(mstatus, "csrr %0," "mstatus");
    RV_SET_REG(mstatus, "csrw mstatus, %0");
    
    RV_GET_REG(misa, "csrr %0," "misa");
    RV_SET_REG(misa, "csrw misa, %0");
    
    RV_GET_REG(mtvec, "csrr %0," "misa");
    RV_SET_REG(mtvec, "csrw misa, %0");
    
    RV_GET_REG(mscratch, "csrr %0," "misa");
    RV_SET_REG(mscratch, "csrw misa, %0");
    
    RV_GET_REG(mepc, "csrr %0," "misa");
    RV_SET_REG(mepc, "csrw misa, %0");
    
    RV_GET_REG(mcause, "csrr %0," "mcause");
    RV_SET_REG(mcause, "csrw mcause, %0");
    
    RV_GET_REG(mvendorid, "csrr %0," "mvendorid");
    RV_GET_REG(marchid, "csrr %0," "marchid");
    RV_GET_REG(mimpid, "csrr %0," "mimpid");
    RV_GET_REG(mhartid, "csrr %0," "mhartid");
    
    RV_GET_REG(sp, "mv %0,""sp");

}
}
