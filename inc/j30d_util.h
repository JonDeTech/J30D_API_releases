/*
 * j30d_util.h
 *
 * Pure helpers: byte order, fixed-point conversion, the calibration block's
 * CRC, and raw-code-to-millivolt scaling. Every function here is a function
 * of its arguments alone -- none of them touch a handle, the platform hooks
 * or any driver state.
 *
 * They are static inline in a header rather than a translation unit of their
 * own precisely because they have no state: there is nothing to link against
 * and nothing to keep consistent across files.
 *
 * Not part of the public API: j30d.h does not include it, and nothing
 * declared here is covered by the version guarantee. Same standing as
 * j30d_regs.h -- an implementation-facing header that ships because the
 * driver ships as source.
 */
#ifndef J30D_UTIL_H_
#define J30D_UTIL_H_

#include <stdint.h>

#include "j30d_regs.h"
#include "j30d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 24-bit two's complement, MSB first, widened to int32_t. */
static inline int32_t sign_extend24(const uint8_t *b /* 3 bytes, MSB first */)
{
	uint32_t v = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | (uint32_t)b[2];
	if (v & 0x800000u)
	{
		v |= 0xFF000000u;
	}
	return (int32_t)v;
}

static inline uint32_t j30d_be32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static inline uint16_t j30d_be16(const uint8_t *p)
{
	return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/* The raw 16-bit trim word, before any Q-format interpretation. */
static inline uint16_t j30d_trim_word(uint8_t msb, uint8_t lsb)
{
	return (uint16_t)(((uint16_t)msb << 8) | (uint16_t)lsb);
}

#define J30D_CRC16_POLY   0x8408u   /* reflected CCITT polynomial, fixed by the block format */

/* The calibration block's checksum. Byte-swapped on the way out, which is
 * how the factory stores it. */
static inline uint16_t j30d_crc16(const uint8_t *data, uint16_t length)
{
	uint32_t crc = 0xFFFFu;

	if (length == 0)
	{
		return (uint16_t)(~crc);
	}

	while (length--)
	{
		uint32_t byte = *data++;
		for (uint8_t i = 0; i < 8; i++, byte >>= 1)
		{
			if ((crc & 0x0001u) ^ (byte & 0x0001u))
			{
				crc = (crc >> 1) ^ J30D_CRC16_POLY;
			}
			else
			{
				crc >>= 1;
			}
		}
	}

	crc = (~crc) & 0xFFFFu;
	uint32_t swapped = ((crc << 8) | ((crc >> 8) & 0xFFu)) & 0xFFFFu;
	return (uint16_t)swapped;
}

#if !defined(J30D_NO_FLOAT)

/* U8Q8: 8-bit unsigned integer part + 8-bit fraction (range 0..255.996). */
static inline float j30d_u8q8_to_float(uint8_t msb, uint8_t lsb)
{
	return (float)j30d_trim_word(msb, lsb) / 256.0f;
}

/* S7Q8: sign bit + 7-bit integer part + 8-bit fraction (range -128..127.996). */
static inline float j30d_s7q8_to_float(uint8_t msb, uint8_t lsb)
{
	return (float)(int16_t)j30d_trim_word(msb, lsb) / 256.0f;
}

/* SYS_CHx_CFG's gain field to the gain it selects. */
static inline float decode_gain(uint8_t cfg_reg_value)
{
	static const float gain_table[8] = { 8.0f, 12.0f, 16.0f, 32.0f, 48.0f, 64.0f, 96.0f, 128.0f };
	uint8_t idx = (cfg_reg_value & J30D_SYS_CHx_CFG_GAIN_Msk) >> J30D_SYS_CHx_CFG_GAIN_Pos;
	return gain_table[idx & 0x7u];
}

static inline float raw_to_mv(int32_t raw, float gain)
{
	return 1000.0f * ((float)raw * J30D_ADC_VREF) / (gain * J30D_RAW_ADC_SCALE);
}

#endif /* !J30D_NO_FLOAT */

#ifdef __cplusplus
}
#endif

#endif /* J30D_UTIL_H_ */
