/*
 * j30d_config.c
 *
 * Definition of j30d_default_config. Values carried over unchanged from the
 * legacy jirs30d.c `sensor_config` initializer. Customer-editable -- these
 * are the values applied whenever j30d_init() is passed a NULL config, or a
 * field is marked J30D_FIELD_DEFAULT. (A field marked J30D_FIELD_UNCHANGED
 * is left alone entirely and never reads this table.)
 */

#include "j30d_config.h"
#include "j30d_regs.h"   /* for the bit-value macros used below */

const j30d_config_t j30d_default_config = {

	/* Left alone by default: 0x7F answers on every unit whatever this holds,
	 * so there is nothing the driver needs here. Set it only if your product
	 * wants a specific address on the bus -- change BOTH fields:
	 *
	 *     .value  = 0x10,                  <- your 7-bit address
	 *     .status = J30D_FIELD_UPDATED,    <- without this, .value is ignored
	 *
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

	.sleep = {
		.value  = 0x0E,   /* 1500 ms */
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

	.bias_cfg = {
		.value  = J30D_BIAS_CFG_5_16_AVDD,
		.status = J30D_FIELD_DEFAULT,
	},
};
