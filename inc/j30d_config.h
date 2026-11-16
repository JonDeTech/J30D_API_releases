/*
 * j30d_config.h
 *
 * The sensor configuration surface. This is the file a customer is
 * expected to actually open and edit -- it has no I2C/platform detail in
 * it, just register-level settings and their default values.
 *
 * Every field in j30d_config_t carries its own status, so
 * j30d_update_config() only writes what actually changed, instead of
 * re-writing the whole register set on every call.
 */

#ifndef J30D_CONFIG_H_
#define J30D_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
	J30D_FIELD_UNCHANGED = 0,  /* leave whatever is currently on the device */
	J30D_FIELD_DEFAULT   = 1,  /* driver's compiled-in default (j30d_default_config) */
	J30D_FIELD_UPDATED   = 2,  /* .value below was set explicitly by the caller -- write it */
} j30d_field_status_t;

/* Carries no register address: which register a field maps to is the
 * driver's business, and j30d.c holds the one mapping. */
typedef struct
{
	uint8_t               value;
	j30d_field_status_t   status;
} j30d_config_field_t;

typedef struct
{
	/* RAM - CHIP_ADDRESS (0x92): the device's own 7-bit I2C address.
	 * Changes the address immediately, with no EEPROM programming or reset,
	 * so it is applied at every j30d_init() like any other field.
	 *
	 * Your port must target the 0x7F wildcard, which every unit answers
	 * whatever its address -- a port aimed at a fixed address would lose the
	 * device the moment this is written.
	 *
	 * Bits 6:0 only. 0x00 is the I2C general call address, so a unit left
	 * there acknowledges every general call. UNCHANGED by default. */
	j30d_config_field_t chip_address;

	j30d_config_field_t command;   /* RAM    - COMMAND: mode select, mode enable, sleep enable (J30D_COMMAND_SLEEP_EN --
	                                 * see the sleep-handling note in j30d.h; you set this bit here, but you never
	                                 * write this field's .value by hand to wake/sleep the device at runtime, the
	                                 * driver does that automatically) */
	j30d_config_field_t sleep;     /* RAM    - SLEEP_TIME */
	j30d_config_field_t sys_cfg1;  /* EEPROM - SYS_CFG1: OSR ambient, output mode, filter */
	j30d_config_field_t sys_cfg2;  /* EEPROM - SYS_CFG2: swap, dither, sleep, ... */
	j30d_config_field_t ch1_cfg;   /* EEPROM - SYS_CH1_CFG: OSR, gain */
	j30d_config_field_t ch2_cfg;   /* EEPROM - SYS_CH2_CFG: OSR, gain */
	j30d_config_field_t bias_cfg;  /* EEPROM - BIAS_CFG */

	/* Not a register shadow -- a driver mode, hence a plain bool. Read by
	 * j30d_init() only; j30d_update_config() ignores it.
	 *
	 * Uses the sleep-mode INT line to signal a new sample. The device has no
	 * data-ready interrupt, so the driver sets the sleep threshold to its
	 * minimum, which makes every sample raise INT. Your EXTI handler calls
	 * j30d_notify_interrupt() and signals a task; that task calls
	 * j30d_get_data(), never the handler.
	 *
	 * Requires platform.release_to_int, platform.now_ms and SLEEP_EN in
	 * `command`; j30d_init() returns J30D_ERR_PARAM without them.
	 *
	 * MAKES THE I2C BUS EXCLUSIVE TO THE J30D -- while the device sleeps,
	 * SDA is not an I2C pin. Do not use on a shared bus. */
	bool interrupt_enable;
} j30d_config_t;

/* Compiled-in defaults (values -- edit these for your product), applied by
 * j30d_init() when passed a NULL config, and used to resolve any field
 * marked J30D_FIELD_DEFAULT in a j30d_init()/j30d_update_config() call.
 * Note this is J30D_FIELD_DEFAULT, not J30D_FIELD_UNCHANGED: an UNCHANGED
 * field is not written at all and does not consult this table. Defined in
 * j30d_config.c. */
extern const j30d_config_t j30d_default_config;

/* ---- setting a field ---------------------------------------------------
 * A field takes effect only with its status set, and a .value written
 * without J30D_FIELD_UPDATED is silently ignored. These set both at once:
 *
 *     cfg.sleep = J30D_SET(J30D_SLEEP_TIME_MS(1000));        assignment (C)
 *     .sleep    = J30D_SET_INIT(J30D_SLEEP_TIME_MS(1000)),   initializer
 *
 * J30D_SET() is a compound literal, so it is C only; the initializer form
 * also works in a static const table such as j30d_default_config, and in
 * C++. */
#define J30D_SET_INIT(v)   { (uint8_t)(v), J30D_FIELD_UPDATED }
#define J30D_SET(v)        ((j30d_config_field_t)J30D_SET_INIT(v))

/* ---- field values ------------------------------------------------------
 * What goes in each field's .value, combined with |. */

/* command -- COMMAND: mode select, mode enable, sleep enable, clock. */
#define J30D_COMMAND_MODE_SEL_Pos    0U
#define J30D_COMMAND_MODE_SEL_Msk    ((uint8_t)(3U << J30D_COMMAND_MODE_SEL_Pos))
#define J30D_COMMAND_MODE_EN_Pos     3U
#define J30D_COMMAND_MODE_EN         ((uint8_t)(1U << J30D_COMMAND_MODE_EN_Pos))
#define J30D_COMMAND_SLEEP_EN_Pos    5U
#define J30D_COMMAND_SLEEP_EN        ((uint8_t)(1U << J30D_COMMAND_SLEEP_EN_Pos))
#define J30D_COMMAND_CLK_Pos         4U
#define J30D_COMMAND_CLK_600KHZ      ((uint8_t)(0U << J30D_COMMAND_CLK_Pos))
#define J30D_COMMAND_CLK_1200KHZ     ((uint8_t)(1U << J30D_COMMAND_CLK_Pos))
#define J30D_COMMAND_MODE_SEL_SINGLE     ((uint8_t)(0U << J30D_COMMAND_MODE_SEL_Pos))
#define J30D_COMMAND_MODE_SEL_DUAL       ((uint8_t)(2U << J30D_COMMAND_MODE_SEL_Pos))
#define J30D_COMMAND_MODE_SEL_DUAL_FAST  ((uint8_t)(3U << J30D_COMMAND_MODE_SEL_Pos))

/* sleep -- SLEEP_TIME, the gap between conversions while the device
 * sleeps: 100 ms steps from 100 ms to 25.6 s. J30D_SLEEP_TIME_MS(1500) is
 * the register value for 1500 ms. `ms` must be a compile-time constant;
 * anything that is not a multiple of 100 in that range fails to compile,
 * with "size of array is negative".
 *
 * The factory calibration assumes the default 1500 ms. Another interval
 * changes the ASIC's self-heating and introduces an offset. */
#define J30D_SLEEP_TIME_MS_VALID(ms) \
	(((ms) % 100u == 0u) && ((ms) >= 100u) && ((ms) <= 25600u))
#define J30D_SLEEP_TIME_MS(ms) \
	((uint8_t)((ms) / 100u - 1u + 0u * sizeof(char[J30D_SLEEP_TIME_MS_VALID(ms) ? 1 : -1])))

/* sys_cfg1 -- SYS_CFG1: ambient OSR, output mode, filter. */
#define J30D_SYS_CFG1_OSRT_AMB_Pos   0U
#define J30D_SYS_CFG1_OSR_AMB_Msk    ((uint8_t)(7U << J30D_SYS_CFG1_OSRT_AMB_Pos))
#define J30D_SYS_CFG1_OUT_MODE_Pos   3U
#define J30D_SYS_CFG1_OUT_MODE_Msk   ((uint8_t)(3U << J30D_SYS_CFG1_OUT_MODE_Pos))
#define J30D_SYS_CFG1_FILT_COEF_Pos  5U
#define J30D_SYS_CFG1_FILT_COEF_Msk  ((uint8_t)(7U << J30D_SYS_CFG1_FILT_COEF_Pos))
#define J30D_SYS_CFG1_OSRT_512       ((uint8_t)(0U << J30D_SYS_CFG1_OSRT_AMB_Pos))
#define J30D_SYS_CFG1_OSRT_1024      ((uint8_t)(1U << J30D_SYS_CFG1_OSRT_AMB_Pos))
#define J30D_SYS_CFG1_OSRT_2048      ((uint8_t)(2U << J30D_SYS_CFG1_OSRT_AMB_Pos))
#define J30D_SYS_CFG1_OSRT_4096      ((uint8_t)(3U << J30D_SYS_CFG1_OSRT_AMB_Pos))
#define J30D_SYS_CFG1_OSRT_128       ((uint8_t)(4U << J30D_SYS_CFG1_OSRT_AMB_Pos))
#define J30D_SYS_CFG1_OSRT_256       ((uint8_t)(5U << J30D_SYS_CFG1_OSRT_AMB_Pos))
#define J30D_SYS_CFG1_OSRT_8192      ((uint8_t)(6U << J30D_SYS_CFG1_OSRT_AMB_Pos))
#define J30D_SYS_CFG1_OSRT_16384     ((uint8_t)(7U << J30D_SYS_CFG1_OSRT_AMB_Pos))
#define J30D_SYS_CFG1_OUT_I2C        ((uint8_t)(0U << J30D_SYS_CFG1_OUT_MODE_Pos))
#define J30D_SYS_CFG1_OUT_PWM        ((uint8_t)(1U << J30D_SYS_CFG1_OUT_MODE_Pos))
#define J30D_SYS_CFG1_OUT_RELAY      ((uint8_t)(2U << J30D_SYS_CFG1_OUT_MODE_Pos))
#define J30D_SYS_CFG1_FILT_DISABLE   ((uint8_t)(0U << J30D_SYS_CFG1_FILT_COEF_Pos))
#define J30D_SYS_CFG1_FILT_1         ((uint8_t)(1U << J30D_SYS_CFG1_FILT_COEF_Pos))
#define J30D_SYS_CFG1_FILT_2         ((uint8_t)(2U << J30D_SYS_CFG1_FILT_COEF_Pos))
#define J30D_SYS_CFG1_FILT_3         ((uint8_t)(3U << J30D_SYS_CFG1_FILT_COEF_Pos))
#define J30D_SYS_CFG1_FILT_4         ((uint8_t)(4U << J30D_SYS_CFG1_FILT_COEF_Pos))
#define J30D_SYS_CFG1_FILT_5         ((uint8_t)(5U << J30D_SYS_CFG1_FILT_COEF_Pos))
#define J30D_SYS_CFG1_FILT_6         ((uint8_t)(6U << J30D_SYS_CFG1_FILT_COEF_Pos))
#define J30D_SYS_CFG1_FILT_7         ((uint8_t)(7U << J30D_SYS_CFG1_FILT_COEF_Pos))

/* sys_cfg2 -- SYS_CFG2. */
#define J30D_SYS_CFG2_SWAP_Msk          ((uint8_t)(1U << 0))   /* input_swap: 1 = CH inputs bonded in reverse polarity */
#define J30D_SYS_CFG2_VT_SCALE_Msk      ((uint8_t)(1U << 1))   /* 0 = +-16mV full range, 1 = +-128mV */
#define J30D_SYS_CFG2_SLEEP_POL_Msk     ((uint8_t)(1U << 2))   /* SLEEP_OUT_Polarity: which side of the threshold triggers INT in Sleep mode -- not a sleep enable */
#define J30D_SYS_CFG2_RELAY_POL_Msk     ((uint8_t)(1U << 3))   /* Relay_OUT_Polarity: which side of the threshold drives Relay output high -- not a relay enable */
#define J30D_SYS_CFG2_PGA_PWR_Pos       4U           /* PGA_PWR<1:0>: power/noise trade-off, not a power-down bit */
#define J30D_SYS_CFG2_PGA_PWR_Msk       ((uint8_t)(3U << J30D_SYS_CFG2_PGA_PWR_Pos))
#define J30D_SYS_CFG2_SERIAL_FILT_Msk   ((uint8_t)(1U << 6))   /* SERIAL_filter_en: IIC input deglitch filter */
#define J30D_SYS_CFG2_ADC_DITH_Msk      ((uint8_t)(1U << 7))

/* ch1_cfg, ch2_cfg -- SYS_CH1_CFG / SYS_CH2_CFG: channel OSR and PGA gain. */
#define J30D_SYS_CHx_CFG_OSRP_Pos    0U
#define J30D_SYS_CHx_CFG_OSRP_Msk    ((uint8_t)(7U << J30D_SYS_CHx_CFG_OSRP_Pos))
#define J30D_SYS_CHx_CFG_GAIN_Pos    3U
#define J30D_SYS_CHx_CFG_GAIN_Msk    ((uint8_t)(7U << J30D_SYS_CHx_CFG_GAIN_Pos))
#define J30D_SYS_CHx_CFG_GAIN_8      ((uint8_t)(0U << J30D_SYS_CHx_CFG_GAIN_Pos))
#define J30D_SYS_CHx_CFG_GAIN_12     ((uint8_t)(1U << J30D_SYS_CHx_CFG_GAIN_Pos))
#define J30D_SYS_CHx_CFG_GAIN_16     ((uint8_t)(2U << J30D_SYS_CHx_CFG_GAIN_Pos))
#define J30D_SYS_CHx_CFG_GAIN_32     ((uint8_t)(3U << J30D_SYS_CHx_CFG_GAIN_Pos))
#define J30D_SYS_CHx_CFG_GAIN_48     ((uint8_t)(4U << J30D_SYS_CHx_CFG_GAIN_Pos))
#define J30D_SYS_CHx_CFG_GAIN_64     ((uint8_t)(5U << J30D_SYS_CHx_CFG_GAIN_Pos))
#define J30D_SYS_CHx_CFG_GAIN_96     ((uint8_t)(6U << J30D_SYS_CHx_CFG_GAIN_Pos))
#define J30D_SYS_CHx_CFG_GAIN_128    ((uint8_t)(7U << J30D_SYS_CHx_CFG_GAIN_Pos))
#define J30D_SYS_CHx_CFG_OSRP_512    ((uint8_t)(0U << J30D_SYS_CHx_CFG_OSRP_Pos))
#define J30D_SYS_CHx_CFG_OSRP_1024   ((uint8_t)(1U << J30D_SYS_CHx_CFG_OSRP_Pos))
#define J30D_SYS_CHx_CFG_OSRP_2048   ((uint8_t)(2U << J30D_SYS_CHx_CFG_OSRP_Pos))
#define J30D_SYS_CHx_CFG_OSRP_4096   ((uint8_t)(3U << J30D_SYS_CHx_CFG_OSRP_Pos))
#define J30D_SYS_CHx_CFG_OSRP_128    ((uint8_t)(4U << J30D_SYS_CHx_CFG_OSRP_Pos))
#define J30D_SYS_CHx_CFG_OSRP_256    ((uint8_t)(5U << J30D_SYS_CHx_CFG_OSRP_Pos))
#define J30D_SYS_CHx_CFG_OSRP_8192   ((uint8_t)(6U << J30D_SYS_CHx_CFG_OSRP_Pos))
#define J30D_SYS_CHx_CFG_OSRP_16384  ((uint8_t)(7U << J30D_SYS_CHx_CFG_OSRP_Pos))

/* bias_cfg -- BIAS_CFG. */
#define J30D_BIAS_CFG_DAC_Msk        ((uint8_t)(15U << 0))
#define J30D_BIAS_CFG_5_16_AVDD      ((uint8_t)(0x0D))

#ifdef __cplusplus
}
#endif

#endif /* J30D_CONFIG_H_ */
