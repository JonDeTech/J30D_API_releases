#ifndef J30D_PLATFORM_STM32_FREERTOS_H_
#define J30D_PLATFORM_STM32_FREERTOS_H_

#include "j30d_platform.h"

/* Fills `platform` in ready to hand to j30d_init(). Creates its mutex on
 * first call. Requires the RTOS scheduler to already be running (uses
 * vTaskDelay/xSemaphoreTake) -- for anything called before
 * osKernelStart(), use platform_stm32_baremetal.h instead. */
void j30d_platform_stm32_freertos_init(j30d_platform_t *platform);

#endif /* J30D_PLATFORM_STM32_FREERTOS_H_ */
