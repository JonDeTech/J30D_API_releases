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

/* Deliberately carries no register address: which register a field maps to
 * is the driver's business, not the customer's, and a caller-supplied
 * address was silently ignored anyway. j30d.c holds the one authoritative
 * mapping. */
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

	/* Not a register shadow -- a driver mode, hence a plain bool.
	 *
	 * Uses the sleep-mode INT line to signal a new sample. The device has no
	 * data-ready interrupt, so the driver sets the sleep threshold to its
	 * minimum, which makes every sample raise INT. Your EXTI handler calls
	 * j30d_notify_interrupt() and signals a task; that task calls
	 * j30d_get_data(), never the handler.
	 *
	 * Requires platform.release_to_int; j30d_init() returns J30D_ERR_PARAM
	 * without it.
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

#ifdef __cplusplus
}
#endif

#endif /* J30D_CONFIG_H_ */
