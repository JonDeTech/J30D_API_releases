/*
 * j30d_config.c
 *
 * Definition of j30d_default_config. Customer-editable -- these
 * are the values applied whenever j30d_init() is passed a NULL config, or a
 * field is marked J30D_FIELD_DEFAULT. (A field marked J30D_FIELD_UNCHANGED
 * is left alone entirely and never reads this table.)
 */

#include "j30d_config.h"   /* the field types, and the values used below */

const j30d_config_t j30d_default_config = {

	/* Left alone by default: 0x7F answers on every unit whatever this holds,
	 * so there is nothing the driver needs here. Set it only if your product
	 * wants a specific address on the bus -- change BOTH fields:
	 *
	 *     .value  = 0x10,                  <- your 7-bit address
	 *     .status = J30D_FIELD_UPDATED,    <- without this, .value is ignored
	 *
	 * or, in one go, replace the whole initializer with J30D_SET_INIT(0x10).
	 * It is then written at every j30d_init(), like every other field here.
	 * Nothing is stored in EEPROM, so nothing can be corrupted. */
	.chip_address = {
		.value  = 0x00,
		.status = J30D_FIELD_UNCHANGED,
	},

	.command = {
		.value  = J30D_COMMAND_MODE_EN
		        | J30D_COMMAND_MODE_SEL_SINGLE
		        | J30D_COMMAND_SLEEP_EN
		        | J30D_COMMAND_CLK_1200KHZ,
		.status = J30D_FIELD_DEFAULT,
	},

	/* The factory calibration assumes 1500 ms -- see J30D_SLEEP_TIME_MS in
	 * j30d_config.h before changing it. */
	.sleep = {
		.value  = J30D_SLEEP_TIME_MS(1500),
		.status = J30D_FIELD_DEFAULT,
	},

	.sys_cfg1 = {
		.value  = J30D_SYS_CFG1_FILT_DISABLE
		        | J30D_SYS_CFG1_OUT_I2C
		        | J30D_SYS_CFG1_OSRT_512,
		.status = J30D_FIELD_DEFAULT,
	},

	.sys_cfg2 = {
		.value  = J30D_SYS_CFG2_ADC_DITH_Msk
		        | J30D_SYS_CFG2_SWAP_Msk,
		.status = J30D_FIELD_DEFAULT,
	},

	.ch1_cfg = {
		.value  = J30D_SYS_CHx_CFG_OSRP_8192
		        | J30D_SYS_CHx_CFG_GAIN_64,
		.status = J30D_FIELD_DEFAULT,
	},

	.ch2_cfg = {
		.value  = J30D_SYS_CHx_CFG_OSRP_8192
		        | J30D_SYS_CHx_CFG_GAIN_64,
		.status = J30D_FIELD_DEFAULT,
	},

	/* Off. Turning it on also requires platform.release_to_int and
	 * platform.now_ms, and makes the I2C bus exclusive to this device -- see
	 * j30d_config.h. */
	.interrupt_enable = false,

	.bias_cfg = {
		.value  = J30D_BIAS_CFG_5_16_AVDD,
		.status = J30D_FIELD_DEFAULT,
	},
};
