#ifndef J30D_PLATFORM_STM32_BAREMETAL_H_
#define J30D_PLATFORM_STM32_BAREMETAL_H_

#include "j30d_platform.h"

/* Fills `platform` in ready to hand to j30d_init(). lock/unlock are left
 * NULL -- correct for single-threaded bare-metal, see platform_stm32_baremetal.c.
 * Also safe to call, and to invoke platform->return_to_iic()/i2c_read()/
 * i2c_write() from, BEFORE the RTOS scheduler has started (uses HAL_Delay,
 * not vTaskDelay) -- e.g. from a boot-time presence probe. */
void j30d_platform_stm32_baremetal_init(j30d_platform_t *platform);

#endif /* J30D_PLATFORM_STM32_BAREMETAL_H_ */
