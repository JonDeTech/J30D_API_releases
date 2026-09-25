/*
 * j30d_calib.h
 *
 * Factory-programmed production/calibration data. This is a 48-byte block
 * written to the device's EEPROM at calibration time (offset/gain trim
 * for the heat-flux conversion, plus traceability: timestamp, UID, lot,
 * wafer, silicon revision). j30d_init() reads and parses it once,
 * best-effort, into h->calib -- see j30d.h.
 *
 * This is read-only from the driver's side: nothing in j30d.c ever writes
 * this block (the legacy driver's write path, used only by factory/CJB
 * calibration tooling, is intentionally not part of this driver).
 *
 * Ported from the legacy jirs30d.c/.h JIRS30D_Calibration_t /
 * detect_unit_version() / ReadProductionData(), bit-for-bit: same byte
 * offsets, same CRC16/CCITT (poly 0x8408, reflected), same Q8.8/S7Q8 fixed
 * point decoding. See j30d.c for the parsing code.
 */

#ifndef J30D_CALIB_H_
#define J30D_CALIB_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* How much of the 48-byte calibration block j30d_init() was able to trust,
 * in the order the legacy driver added support for it. Ported unchanged
 * from jirs30d.h's UnitVersion_t (same numeric values) so a UART/log
 * command dumping this doesn't shift meaning across the transition. */
typedef enum
{
	J30D_UNIT_VERSION_UNKNOWN          = 0,   /* couldn't tell -- treat calib as absent */
	J30D_UNIT_VERSION_CH2              = 1,   /* earliest units: block is all 0xFF, no calibration data at all */
	J30D_UNIT_VERSION_CH1_NOTIMESTAMP  = 2,   /* has offset/gain, timestamp present, no revision byte */
	J30D_UNIT_VERSION_REV1             = 3,   /* revision == 1, no CRC to check */
	J30D_UNIT_VERSION_REV2PLUS         = 4,   /* revision >= 2, CRC16 verified against the stored checksum */
} j30d_unit_version_t;

typedef struct
{
	bool                 valid;    /* true once j30d_init() has successfully read+parsed this block.
	                                 * False (all other fields zeroed) if the read failed outright --
	                                 * distinct from `version == J30D_UNIT_VERSION_UNKNOWN`, which means
	                                 * the read succeeded but the block's *contents* couldn't be trusted
	                                 * (blank EEPROM, or a CRC mismatch on a rev>=2 unit). */
	j30d_unit_version_t  version;

	uint32_t  prod_timestamp;   /* factory calibration Unix timestamp -- meaningful for version >= CH1_NOTIMESTAMP */
	uint32_t  uid;
	uint8_t   revision;
	uint16_t  lot_no;
	uint16_t  wafer_no;

	/* Offset/gain trim for the heat-flux conversion, one pair per channel.
	 * Units and the formula they feed depend on `version` -- see
	 * J30D_Adapter_Convert() in j30d_adapter.c for the version-specific
	 * math this driver replicates from legacy JIRS30D_Convert(). Only
	 * channel 1's pair is currently consumed by that adapter (channel 1
	 * is the fixed signal source, see j30d_adapter.h); channel 2's is
	 * carried here for completeness/future use. */
#if defined(J30D_NO_FLOAT)
	/* J30D_NO_FLOAT build (see j30d_types.h): the raw 16-bit trim words
	 * straight out of the EEPROM block, undecoded, so the driver contains
	 * no floating point at all. Both formats have 8 fraction bits -- the
	 * real value is word / 256 -- but the integer part's signedness
	 * depends on `version`:
	 *   J30D_UNIT_VERSION_REV2PLUS -> S7Q8: read as int16_t (signed)
	 *   CH1_NOTIMESTAMP / REV1     -> U8Q8: read as uint16_t (unsigned)
	 *   CH2 / UNKNOWN              -> no trustworthy data; left 0
	 * e.g. signed: ((int16_t)c->sensor_offset1_raw) / 256.0 */
	uint16_t  sensor_offset1_raw;
	uint16_t  sensor_gain1_raw;
	uint16_t  sensor_offset2_raw;
	uint16_t  sensor_gain2_raw;
#else
	float     sensor_offset1;
	float     sensor_gain1;
	float     sensor_offset2;
	float     sensor_gain2;
#endif

	uint16_t  crc;   /* as read from the device (big-endian in the block) -- for diagnostics only */
} j30d_calib_t;

#ifdef __cplusplus
}
#endif

#endif /* J30D_CALIB_H_ */
