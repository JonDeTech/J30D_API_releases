#ifndef J30D_PLATFORM_STM32_FREERTOS_H_
#define J30D_PLATFORM_STM32_FREERTOS_H_

#include "j30d_platform.h"

/* Fills `platform` in ready to hand to j30d_init(). Creates its mutex on
 * first call. Requires the RTOS scheduler to already be running (uses
 * vTaskDelay/xSemaphoreTake) -- for anything called before
 * osKernelStart(), use platform_stm32_baremetal.h instead. */
void j30d_platform_stm32_freertos_init(j30d_platform_t *platform);

/* Example: a task reading heat flux from every new sample.
 *
 *     #include "FreeRTOS.h"
 *     #include "task.h"
 *     #include "j30d.h"
 *     #include "platform_stm32_freertos.h"
 *
 *     static void heat_flux_task(void *arg)
 *     {
 *         j30d_platform_t platform;
 *         j30d_handle_t   h;
 *         (void)arg;
 *
 *         j30d_platform_stm32_freertos_init(&platform);
 *         if (j30d_init(&h, &platform, NULL) != J30D_OK)
 *         {
 *             vTaskDelete(NULL);
 *         }
 *
 *         for (;;)
 *         {
 *             j30d_data_t d;
 *             float       hf_mw_cm2;
 *
 *             if (j30d_get_data_blocking(&h, &d, 5000) == J30D_OK &&
 *                 j30d_heat_flux(&h, &d.raw_ch1, &hf_mw_cm2) == J30D_OK)
 *             {
 *                 use_heat_flux(hf_mw_cm2);
 *             }
 *         }
 *     }
 *
 * j30d_get_data_blocking() sleeps in vTaskDelay() until the next sample --
 * about 1.8 s at the defaults -- without holding the driver's mutex, so
 * other tasks keep both the CPU and the bus meanwhile. It returns J30D_OK
 * only once raw_ch1 is fresh. On a part without an FPU,
 * j30d_heat_flux_x1000() gives the same as an integer in mW/cm2 x 1000. */

#endif /* J30D_PLATFORM_STM32_FREERTOS_H_ */
