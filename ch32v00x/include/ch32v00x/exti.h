#ifndef __CH32V00X_EXTI_H__
#define __CH32V00X_EXTI_H__

#include "core/riscv.h"
#include "core/v00x.h"

namespace ch32 {

    /**
     * External interrupts.
     */
    namespace exti {
        /**
         * EXTI line number. 
         */
        typedef enum {
            EXTI_LINE_0 = 0x001,
            EXTI_LINE_1 = 0x002,
            EXTI_LINE_2 = 0x004,
            EXTI_LINE_3 = 0x008,
            EXTI_LINE_4 = 0x010,
            EXTI_LINE_5 = 0x020,
            EXTI_LINE_6 = 0x040,
            EXTI_LINE_7 = 0x080,
            EXTI_LINE_8 = 0x100,
            EXTI_LINE_9 = 0x200,
        } ExtiLine;

        /**
         * EXTI mode.
         */
        typedef enum {
            EXTI_MODE_INTR = 0x00,
            EXTI_MODE_EVENT = 0x04,
        } ExtiMode;

        /**
         * EXTI trigger.
         */
        typedef enum {
            EXTI_TRIG_RISE = 0x08,
            EXTI_TRIG_FALL = 0x0c,
            EXTI_TRIG_RISEFALL = 0x10
        } ExtiTrigger;

        /**
         * De-initializes the EXTI registers to default value.
         */
        void deinit();

        /**
         * Initializes the EXTI line.
         * @param line EXTI_LINE_0 ~ 9.
         * @param mode default: EXTI_MODE_INTR.
         * @param trig default: EXTI_TRIG_FALL.
         * @param state default: RVFN_DISABLE.
         */
        void init(ExtiLine line, ExtiMode mode, ExtiTrigger trig, RvFunc state);

        /**
         * Generates software interrupt for the EXTI line.
         */
        void generate(ExtiLine line);

        /**
         * Checks whether the specified eXTI line flag is set or not.
         */
        bool check(ExtiLine line);

        /**
         * Clears the EXTI's line pending flags.
         */
        void clear(ExtiLine line);

        /**
         * Checks whether the specified EXTI line is asserted or not.
         */
        bool intr(ExtiLine line);
    }
}

#endif