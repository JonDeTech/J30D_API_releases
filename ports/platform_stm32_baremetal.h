#ifndef J30D_PLATFORM_STM32_BAREMETAL_H_
#define J30D_PLATFORM_STM32_BAREMETAL_H_

#include "j30d_platform.h"

/* Fills `platform` in ready to hand to j30d_init(). lock/unlock are left
 * NULL -- correct for single-threaded bare-metal, see platform_stm32_baremetal.c.
 * Also safe to call, and to invoke platform->return_to_iic()/i2c_read()/
 * i2c_write() from, BEFORE the RTOS scheduler has started (uses HAL_Delay,
 * not vTaskDelay) -- e.g. from a boot-time presence probe. */
void j30d_platform_stm32_baremetal_init(j30d_platform_t *platform);

/* Example: heat flux from every new sample, single-threaded.
 *
 *     #include <stdio.h>
 *     #include "j30d.h"
 *     #include "platform_stm32_baremetal.h"
 *
 *     void heat_flux_loop(void)
 *     {
 *         j30d_platform_t platform;
 *         j30d_handle_t   h;
 *
 *         j30d_platform_stm32_baremetal_init(&platform);
 *         if (j30d_init(&h, &platform, NULL) != J30D_OK)
 *         {
 *             return;
 *         }
 *
 *         for (;;)
 *         {
 *             j30d_data_t d;
 *             int32_t     hf_x1000;
 *
 *             if (j30d_get_data_blocking(&h, &d, 5000) == J30D_OK &&
 *                 j30d_heat_flux_x1000(&h, &d.raw_ch1, &hf_x1000) == J30D_OK)
 *             {
 *                 int32_t mag = (hf_x1000 < 0) ? -hf_x1000 : hf_x1000;
 *                 printf("%s%ld.%03ld mW/cm2\n", (hf_x1000 < 0) ? "-" : "",
 *                        (long)(mag / 1000), (long)(mag % 1000));
 *             }
 *         }
 *     }
 *
 * j30d_get_data_blocking() waits in HAL_Delay() for the next sample -- about
 * 1.8 s at the defaults -- and returns J30D_OK only once raw_ch1 is fresh.
 * j30d_heat_flux_x1000() needs no FPU, which the STM32L0's Cortex-M0+ does
 * not have; its result is mW/cm2 x 1000, so 12345 prints as 12.345.
 * j30d_heat_flux() gives the same as a float in mW/cm2. */

#endif /* J30D_PLATFORM_STM32_BAREMETAL_H_ */
