#ifndef __CH32V00X_IRQ_H__
#define __CH32V00X_IRQ_H__

/**
 * File: ch32v00x/nvic.h
 * Desc: NVIC features.
 * Author: jay94ks.
 */

#include "core/riscv.h"

namespace ch32 {
    /**
     * interrupt numbers.
     */
    typedef enum {
        IRQ_NMI = 2,            // --> Non-Maskable interrupt.
        IRQ_EXC = 3,            // --> Exception.
        IRQ_HardFault = IRQ_EXC,// --> Hard fault.

        IRQ_SYSTICK = 12,       // --> System Timer.
        IRQ_SOFTWARE = 14,      // --> Software

        IRQ_WWDG = 16,          // --> Window Watchdog.
        IRQ_PVD = 17,           // --> PVD through EXTI
        IRQ_FLASH = 18,         // --> Flash
        IRQ_RCC = 19,           // --> RCC
        IRQ_EXTI7_0 = 20,       // --> EXTI [7...0]
        IRQ_AWU = 21,           // --> AWU

        IRQ_DMA1_1 = 22,        // --> DMA1 channel 1
        IRQ_DMA1_2 = 23,        // --> DMA1 channel 2
        IRQ_DMA1_3 = 24,        // --> DMA1 channel 3
        IRQ_DMA1_4 = 25,        // --> DMA1 channel 4
        IRQ_DMA1_5 = 26,        // --> DMA1 channel 5
        IRQ_DMA1_6 = 27,        // --> DMA1 channel 6
        IRQ_DMA1_7 = 28,        // --> DMA1 channel 7

        IRQ_ADC = 29,           // --> ADC
        IRQ_I2C1_EV = 30,       // --> I2C event
        IRQ_I2C1_ER = 31,       // --> I2C error
        IRQ_USART1 = 32,        // --> USART1
        IRQ_SPI1 = 33,          // --> SPI1

        IRQ_TIM1_BRK = 34,      // --> TIM1 Break
        IRQ_TIM1_UP = 35,       // --> TIM1 Update
        IRQ_TIM1_TRG_COM = 36,  // --> TIM1 Trigger, Commutation
        IRQ_TIM1_CC = 37,       // --> TIM1 Capture Compare

        IRQ_TIM2 = 38,          // --> TIM2
    } EIrq;
    
    /**
     * NVIC.
     */
    namespace nvic {
        /**
         * RV_PFIC: E000 E000.
         * Alias: RV_NVIC.
         */
        #define RV_PFIC ((ch32::riscv::SPFIC*) 0xE000E000)
        #define RV_NVIC RV_PFIC
        #define RV_NVIC_KEY1 uint32_t(0xFA050000)
        #define RV_NVIC_KEY2 uint32_t(0xBCAF0000)
        #define RV_NVIC_KEY3 uint32_t(0xBEEF0000)

        /**
         * RV_SYSTICK: E000 F000. 
         */
        #define RV_SYSTICK  ((ch32::riscv::SSysTick*) 0xE000F000)

        // --
        #define RV_NVIC_REG(irq)  uint32_t((irq) >> 5)
        #define RV_NVIC_FLAG(irq) (1 << uint32_t((irq) & 0x1f))

        /**
         * Enable interrupt for NVIC.
         */
        RV_FORCEINLINE static inline void enable(EIrq irq) {
            RV_NVIC->ienr[RV_NVIC_REG(irq)] = RV_NVIC_FLAG(irq);
        }

        /**
         * Disable interrupt for NVIC.
         */
        RV_FORCEINLINE static inline void disable(EIrq irq) {
            RV_NVIC->irer[RV_NVIC_REG(irq)] = RV_NVIC_FLAG(irq);
        }

        /**
         * Get interrupt enable status.
         * @return false: interrupt pending disable.
         * @return true: interrupt pending enable.
         */
        RV_FORCEINLINE static inline bool isEnabled(EIrq irq) {
            return (RV_NVIC->isr[RV_NVIC_REG(irq)] & RV_NVIC_FLAG(irq)) != 0;
        }

        /**
         * Get interrupt pending status.
         * @return false: interrupt pending disable.
         * @return true: interrupt pending enable.
         */
        RV_FORCEINLINE static inline bool isPending(EIrq irq) {
            return (RV_NVIC->ipr[RV_NVIC_REG(irq)] & RV_NVIC_FLAG(irq)) != 0;
        }

        /**
         * Set interrupt pending.
         */
        RV_FORCEINLINE static inline void setPending(EIrq irq) {
            RV_NVIC->ipsr[RV_NVIC_REG(irq)] = RV_NVIC_FLAG(irq);
        }

        /**
         * Clear interrupt pending.
         */
        RV_FORCEINLINE static inline void clearPending(EIrq irq) {
            RV_NVIC->iprr[RV_NVIC_REG(irq)] = RV_NVIC_FLAG(irq);
        }

        /**
         * Get the interrupt active state.
         * @return true or false: active or not.
         */
        RV_FORCEINLINE static inline bool isActive(EIrq irq) {
            return (RV_NVIC->iactr[RV_NVIC_REG(irq)] & RV_NVIC_FLAG(irq)) != 0;
        }

        /**
         * Set interrupt priority. 
         * @param priority (csr, 0x84, bit1 = 1) nesting enable: bit 7 preemption, bit 6 sub priority. bit 5~0 reserved.
         * @param priority (csr, 0x84, bit1 = 0) nesting disable: bit 7 preemption, bit 6 sub priority.
         */
        RV_FORCEINLINE static inline void priority(EIrq irq, uint8_t priority) {
            RV_NVIC->iprior[RV_NVIC_REG(irq)] = priority;
        }

        /**
         * WFI, Wait for interrupt.
         */
        RV_FORCEINLINE static inline void wfi() {
            RV_NVIC->sctlr &= ~(1 << 3);
            RV_ASM("wfi");
        }

        /**
         * SEV, Set event.
         */
        RV_FORCEINLINE static inline void sev() {
            uint32_t t;

            t = RV_NVIC->sctlr;
            RV_NVIC->sctlr |= (1 << 3) | (1 << 5);
            RV_NVIC->sctlr = (RV_NVIC->sctlr & ~(1<<5)) | ( t & (1<<5));
        }

        /**
         * WFE, wait for event once. 
         */
        RV_FORCEINLINE static inline void wfe() {
            RV_NVIC->sctlr |= (1<<3);
            asm volatile ("wfi");
        }

        /**
         * WFE, wait for event twice.
         */
        RV_FORCEINLINE static inline void wfe2() {
            sev();
            wfe();
            wfe();
        }

        /**
         * Set VTF interrupt.
         */
        RV_FORCEINLINE static inline void vtf(uint32_t addr, EIrq irq, uint8_t n, RvFunc state) {
            const uint32_t ADDR_MASK = 0xfffffffe;
            if (n > 1) {
                return;
            }

            addr &= ADDR_MASK;
            if (state != RVFN_DISABLE) {
                addr |= 0x01;
            }

            RV_NVIC->vtfidr[n] = irq;
            RV_NVIC->vtfaddr[n] = addr;
        }

        /**
         * Initiate a system reset request.
         */
        RV_FORCEINLINE static inline void systemReset() {
            RV_NVIC->cfgr = RV_NVIC_KEY3 | (1 << 7);
        }
    }
}

#endif