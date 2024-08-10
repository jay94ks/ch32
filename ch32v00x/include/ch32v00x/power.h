#ifndef __CH32V00X_POWER_H__
#define __CH32V00X_POWER_H__

#include "core/riscv.h"
#include "core/v00x.h"

namespace ch32 {
    /**
     * Power (PWR).
     */
    namespace power {
        /**
         * Power detection level.
         */
        typedef enum {
            PWRPVD_2V9  = 0x00,
            PWRPVD_3V1  = 0x20,
            PWRPVD_3V3  = 0x40,
            PWRPVD_3V5  = 0x60,
            PWRPVD_3V7  = 0x80,
            PWRPVD_3V9  = 0xa0,
            PWRPVD_4V1  = 0xc0,
            PWRPVD_4V4  = 0xe0,
        } PwrPvdLevel;

        /**
         * Power auto-wake up clock prescaler.
         */
        typedef enum {
            PWRAWU_1        = 0,
            PWRAWU_2        = 0x02,
            PWRAWU_4        = 0x03,
            PWRAWU_8        = 0x04,
            PWRAWU_16       = 0x05,
            PWRAWU_32       = 0x06,
            PWRAWU_64       = 0x07,
            PWRAWU_128      = 0x08,
            PWRAWU_256      = 0x09,
            PWRAWU_512      = 0x0a,
            PWRAWU_1024     = 0x0b,
            PWRAWU_2048     = 0x0c,
            PWRAWU_4096     = 0x0d,
            PWRAWU_10240    = 0x0e,
            PWRAWU_61440    = 0x0f,
        } PwrAwu;

        /**
         * Power stop mode.
         */
        typedef enum {
            PWRSTOP_WFI     = 0x01,
            PWRSTOP_WFE     = 0x02,
        } PwrStopMode;
        
        /**
         * De-initializes the power peripheral registers to default. 
         */
        void deinit();

        /**
         * Enables or disables the power voltage detector.
         * @param state RVFN_ENABLE or RVFN_DISABLE. 
         */
        void pvd(RvFunc state);

        /**
         * Configures the voltage threshold detected by PVD.
         */
        void pvdSet(PwrPvdLevel pvd);

        /**
         * Enables or disables the auto wake-up.
         * @param state RVFN_ENABLE or RVFN_DISABLE. 
         */
        void autoWakeUp(RvFunc state);

        /**
         * Sets the auto wake up prescaler.
         */
        void awuPrescalerSet(PwrAwu prescaler);

        /**
         * Sets the WWDG window value. 
         */
        void awuWindowSet(uint8_t window);

        /**
         * Enter to stand-by mode with the specified stop mode.
         */
        void enter(PwrStopMode mode);

        /**
         * Checks whether the PVDO flag is set or not.
         */
        bool check();
    }
}

#endif