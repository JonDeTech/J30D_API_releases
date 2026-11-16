/*
 * j30d_types.h
 *
 * Shared enums/structs used across the public API. No platform or config
 * details live here on purpose -- this file only describes *data shapes*,
 * so it's safe to include from anywhere without pulling in I2C/GPIO
 * concerns.
 */

#ifndef J30D_TYPES_H_
#define J30D_TYPES_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Transport/handshake level result. This is NOT about whether the sensor
 * had fresh data -- see j30d_field_state_t for that. A J30D_OK return from
 * j30d_get_data() means "the I2C bus operations that were attempted
 * completed"; it says nothing about which fields turned out fresh. */
typedef enum
{
	J30D_OK = 0,
	J30D_ERR_PARAM,               /* NULL handle/pointer, or invalid argument */
	J30D_ERR_NOT_INIT,            /* called before j30d_init() succeeded */
	J30D_ERR_I2C,                 /* transport failure: NACK, controller/arbitration error */
	J30D_ERR_TIMEOUT,             /* transport or handshake did not complete in time, or
	                                * platform.lock reported it could not acquire the lock */
	J30D_ERR_EEPROM_NOT_LOADED,   /* soft-reset handshake: EEPROM-loaded bit never set */
	J30D_ERR_NO_CALIB,            /* the unit carries no usable factory calibration
	                                * (j30d_calib_t.version CH2 or UNKNOWN, or .valid false) */
	J30D_ERR_RESTARTED,           /* j30d_get_data(): the device had stopped converting and
	                                * the driver restarted it -- see j30d_get_data() in j30d.h */
} j30d_err_t;

/* Define J30D_NO_FLOAT for the whole build (driver and application alike)
 * to drop every float from this API: j30d_field_t loses .value and
 * j30d_calib_t carries its trim data as raw fixed-point words. Intended for
 * FPU-less parts -- on a Cortex-M0+ the .value conversions are soft-float
 * calls, and the library they pull in costs several KB of flash. Nothing is
 * lost that cannot be recomputed: .raw_code is still there, and the scale
 * relationship is published below. Note this changes struct layouts, so it
 * must be set consistently across every translation unit. */

/* Per-field freshness. Every j30d_field_t always holds a real, previously
 * valid value (or 0 before the first successful read) -- state only tells
 * you how much to trust it *this call*, it never gates whether value is
 * populated. */
typedef enum
{
	J30D_STALE = 0,   /* no new sample this call; value is last known-good */
	J30D_FRESH = 1,   /* sensor had new data ready and it was read successfully this call */
	J30D_ERROR = 2,   /* sensor reported new data, but reading it failed; value is last known-good */
} j30d_field_state_t;

typedef struct
{
#if !defined(J30D_NO_FLOAT)
	float               value;      /* engineering units -- see the comment on the matching field in j30d_data_t */
#endif
	int32_t             raw_code;   /* sign-extended, unscaled register value .value was derived from --
	                                  * i.e. value == raw_code / <the field's scale constant>, exactly, since
	                                  * every scale constant this driver uses is a power of two. Present for
	                                  * integrations that need the bare ADC code (e.g. to match an external
	                                  * protocol/filter pipeline that predates this driver) instead of
	                                  * reconstructing it by inverting `value`. */
	j30d_field_state_t  state;
} j30d_field_t;

/* j30d_get_data() refreshes every field below in one call -- see its doc
 * comment in j30d.h.
 *
 * There is no `obj` field: the ASIC's on-chip "object temperature" converts
 * the IR reading under an emissivity/distance assumption that does not
 * apply to heat-flux measurement. */
typedef struct
{
	j30d_field_t ch1;       /* .value = millivolts: the ASIC's corrected thermopile voltage
	                          * (CAL registers), scaled by SYS_CFG2's VT_SCALE -- see
	                          * J30D_VTP_SCALE below. The factory heat-flux calibration
	                          * (j30d_calib.h) applies to raw_ch1, not to this. */
	j30d_field_t ch2;       /* as ch1, for channel 2 */
	j30d_field_t amb;       /* .value = degrees C, the ASIC's internal temperature sensor */
	j30d_field_t raw_ch1;   /* .value = millivolts at the PGA input */
	j30d_field_t raw_ch2;   /* .value = millivolts at the PGA input */
	j30d_field_t raw_amb;   /* unscaled ADC code only -- .value == .raw_code.
	                          * The ambient sensor bypasses the analog front end,
	                          * so it has no gain and no millivolt equivalent. */
} j30d_data_t;

/* Scale constants used to derive .value from .raw_code (amb uses
 * J30D_AMB_SCALE; ch1/ch2 use J30D_VTP_SCALE while SYS_CFG2's VT_SCALE is
 * clear, +-16 mV full scale, and J30D_VTP_SCALE_WIDE while it is set,
 * +-128 mV; raw_ch1/raw_ch2 are NOT a plain division of these -- see
 * raw_to_mv() in j30d_util.h, they also fold in PGA gain and the ADC
 * reference voltage. raw_amb is not scaled at all:
 * the ambient sensor has no analog front end, so it has no gain and no mV
 * interpretation -- its .value is simply its .raw_code). Public so code
 * outside the driver (adapters, tooling) that needs the relationship
 * between .value and .raw_code doesn't have to duplicate magic numbers.
 *
 * Every one of them is a power of two, which is what makes the round-trip
 * value == raw_code / scale exact. The _SHIFT forms below say the same
 * thing without involving the FPU, and are defined in every build. */
#define J30D_AMB_SCALE_SHIFT        14
#define J30D_VTP_SCALE_SHIFT        19   /* VT_SCALE clear: 2^23 codes over 16 mV */
#define J30D_VTP_SCALE_WIDE_SHIFT   16   /* VT_SCALE set:   2^23 codes over 128 mV */
#define J30D_RAW_ADC_SCALE_SHIFT    23

#if !defined(J30D_NO_FLOAT)
#define J30D_AMB_SCALE        16384.0f    /* 2^14 */
#define J30D_VTP_SCALE        524288.0f   /* 2^19 */
#define J30D_VTP_SCALE_WIDE   65536.0f    /* 2^16 */
#define J30D_RAW_ADC_SCALE    8388608.0f  /* 2^23 */

/* The ADC's reference, in volts. Pairs with J30D_RAW_ADC_SCALE above to turn
 * a raw code into millivolts: mV = 1000 * code * VREF / (gain * SCALE). */
#define J30D_ADC_VREF        1.1f
#endif

#ifdef __cplusplus
}
#endif

#endif /* J30D_TYPES_H_ */
