#ifndef __CH32V00X_CORE_RISCV_H__
#define __CH32V00X_CORE_RISCV_H__

/**
 * File: ch32v00x/core/riscv.h
 * Desc: RISCV V2 core peripheral access layer.
 * Author: jay94ks.
 */

#ifndef __cplusplus
#error "this libary requires C++"
#endif

/**
 * Intelisense helper for VSCode.
 */
#if defined(__INTELLISENSE__) && __INTELLISENSE__
#define __asm
#define __attribute__(...)
#define RV_ASM(...)
#endif

/**
 * IO definitions.
 * RV_RD: read-only.
 * RV_WR: write-only.
 * RV_RW: read-write. 
 */
#define RV_RD   volatile const
#define RV_RW   volatile
#define RV_WR   volatile

/**
 * Force-inline.
 */
#define RV_FORCEINLINE __attribute__(( always_inline ))
#ifndef RV_ASM
#define RV_ASM(...) __asm volatile (__VA_ARGS__)
#endif

/**
 * uses standard integers.
 */
#include <stdint.h>
#include "../nvic.h"

/**
 * CH32 namespace.
 */
namespace ch32 {

    /**
     * RISCV ready status.
     */
    typedef enum {
        RVERR_NOT_READY = 0,
        RVERR_READY = !RVERR_NOT_READY
    } RvReady;

    /**
     * RISCV functional state. 
     */
    typedef enum {
        RVFN_DISABLE = 0,
        RVFN_ENABLE = !RVFN_DISABLE
    } RvFunc;

    /**
     * RISCV flag state. 
     */
    typedef enum {
        RVFL_RESET = 0,
        RVFL_SET = !RVFL_RESET
    } RvFlag;

    /**
     * RISCV core features.
     */
    namespace riscv {
        /**
         * PFIC, program fast interrupt controller.
         * A structure that mapped on memory.
         */
        typedef struct {
            RV_RD uint32_t isr[8];
            RV_RD uint32_t ipr[8];
            RV_RW uint32_t ithresdr;
            RV_RW uint32_t reserved;
            RV_RW uint32_t cfgr;
            RV_RD uint32_t gisr;
            RV_RW uint8_t vtfidr[4];
            uint8_t reserved0[12];
            RV_RW uint32_t vtfaddr[4];
            uint8_t reserved1[0x90];
            RV_WR uint32_t ienr[8];
            uint8_t reserved2[0x60];
            RV_WR uint32_t irer[8];
            uint8_t reserved3[0x60];
            RV_WR uint32_t ipsr[8];
            uint8_t reserved4[0x60];
            RV_WR uint32_t iprr[8];
            uint8_t reserved5[0x60];
            RV_RW uint32_t iactr[8];
            uint8_t reserved6[0xe0];
            RV_RW uint8_t iprior[256];
            uint8_t reserved7[0x810];
            RV_RW uint32_t  sctlr;
        } SPFIC;
        
        /**
         * SysTick.
         * A structure that mapped on memory.
         */
        typedef struct {
            RV_RW uint32_t ctlr;
            RV_RW uint32_t sr;
            RV_RW uint32_t cnt;
            uint32_t reserved0;
            RV_RW uint32_t cmp;
            uint32_t reserved1;
        } SSysTick;

        /**
         * Enable the global interrupt.
         * This function is only used for Machine mode.
         */
        RV_FORCEINLINE static inline void enableIrq() {
            RV_ASM ("csrs mstatus, %0" : : "r" (0x88) );
        }

        /**
         * Disable the global interrupt.
         * This function is only used for Machine mode.
         */
        RV_FORCEINLINE static inline void disableIrq() {
            RV_ASM ("csrc mstatus, %0" : : "r" (0x88) );
        }

        /**
         * No-operation.
         * This consume a opcode cycle once.
         */
        RV_FORCEINLINE static inline void nop() {
            RV_ASM ("nop");
        }

        /**
         * Get the machine status register. 
         */
        uint32_t mstatus();

        /**
         * Set the machine status register.
         */
        void mstatus(uint32_t val);

        /**
         * Get the machine ISA register. 
         */
        uint32_t misa();

        /**
         * Set the machine ISA register. 
         */
        void misa(uint32_t val);

        /**
         * Get the machine trap-vector base-address register.
         */
        uint32_t mtvec();

        /**
         * Set the machine trap-vector base-address register.
         */
        void mtvec(uint32_t val);

        /**
         * Get the machine scratch register. 
         */
        uint32_t mscratch();

        /**
         * Set the machine scratch register. 
         */
        void mscratch(uint32_t val);

        /**
         * Get the machine exception program register.
         */
        uint32_t mepc();

        /**
         * Set the machine exception program register.
         */
        void mepc(uint32_t val);

        /**
         * Get the machine cause register.
         */
        uint32_t mcause();

        /**
         * Set the machine cause register.
         */
        void mcause(uint32_t val);

        /**
         * Get the machine vendor id register.
         */
        uint32_t mvendorid();

        /**
         * Get the machine architecture id register.
         */
        uint32_t marchid();
        
        /**
         * Get the machine implementation id register.
         */
        uint32_t mimpid();
        
        /**
         * Get the machine hart id register.
         */
        uint32_t mhartid();
        
        /**
         * Get the SP register.
         */
        uint32_t sp();
    }

}

#endif