/*
 * j30d_regs.h
 *
 * Device register addresses, and the bits only the driver uses. Internal
 * to the driver -- customers configure the sensor through j30d_config.h /
 * j30d.h, not by poking these addresses directly, so this header is not
 * included by j30d.h itself. The values that go in each config field
 * (J30D_COMMAND_*, J30D_SYS_CFG1_*, J30D_SYS_CHx_CFG_*, ...) are public and
 * live in j30d_config.h, which this header includes.
 */

#ifndef J30D_REGS_H_
#define J30D_REGS_H_

#include <stdint.h>

#include "j30d_config.h"   /* the field values */

/* --- RAM --- */
#define J30D_REG_SOFT_RESET          0x00
#define J30D_SOFT_RESET_Pos1         5U
#define J30D_SOFT_RESET_Pos2         2U
#define J30D_SOFT_RESET_Msk          ((uint8_t)((1U << J30D_SOFT_RESET_Pos1) | (1U << J30D_SOFT_RESET_Pos2)))

#define J30D_REG_DATA_READY          0x02
#define J30D_TA_DRDY_Pos             0U
#define J30D_TA_DRDY_Msk             ((uint8_t)(1U << J30D_TA_DRDY_Pos))
#define J30D_TO1_DRDY_Pos            1U
#define J30D_TO1_DRDY_Msk            ((uint8_t)(1U << J30D_TO1_DRDY_Pos))
#define J30D_TO2_DRDY_Pos            2U
#define J30D_TO2_DRDY_Msk            ((uint8_t)(1U << J30D_TO2_DRDY_Pos))
/* Bit 3 (OBJ_DRDY) is not defined: this driver never reads the OBJ
 * register -- see the note on J30D_REG_OBJ_MSB below. */

#define J30D_REG_DATA_READY_RAW      0x03
#define J30D_TA_RAW_DRDY_Pos         4U
#define J30D_TA_RAW_DRDY_Msk         ((uint8_t)(1U << J30D_TA_RAW_DRDY_Pos))
#define J30D_TO1_RAW_DRDY_Pos        5U
#define J30D_TO1_RAW_DRDY_Msk        ((uint8_t)(1U << J30D_TO1_RAW_DRDY_Pos))
#define J30D_TO2_RAW_DRDY_Pos        6U
#define J30D_TO2_RAW_DRDY_Msk        ((uint8_t)(1U << J30D_TO2_RAW_DRDY_Pos))

#define J30D_REG_ERROR_CODE          0x04
/* Only bit currently meaningful: EEPROM-loaded-after-reset flag, checked
 * during j30d_init()'s soft-reset handshake. */
#define J30D_ERROR_CODE_EEPROM_LOADED_Msk   ((uint8_t)(0x02))

/* J30D_REG_OBJ_MSB (0x10) is not defined. The ASIC's on-chip "object
 * temperature" converts the IR reading under an emissivity/distance
 * assumption that does not apply to heat-flux measurement, so the
 * register, its DRDY bit and a data field for it are all left out. */
#define J30D_REG_AMB_MSB             0x16
#define J30D_REG_CAL_MSB             0x19   /* ch1 cal, 6 bytes covers ch1+ch2 */
#define J30D_REG_RAW_MSB             0x22   /* ch1/ch2/amb raw, 9 bytes */

#define J30D_REG_COMMAND             0x30
#define J30D_REG_SLEEP_TIME          0x31   /* 0x00 = 100ms, then +100ms per step */
#define J30D_REG_BLOW_START          0x40   /* blows EEPROM fuse -- production use only */

/* --- EEPROM --- */
/* ID0/ID1: not a chip/part identifier despite the name -- free scratch
 * space either JDT or the customer can write their own data into. Reads
 * back 0x00/0x00 on every unit currently in use; that's normal, not a
 * fault or an unprogrammed-EEPROM warning sign. */
#define J30D_REG_ID0                 0x90
#define J30D_REG_ID1                 0x91
#define J30D_REG_CHIP_ADDRESS        0x92

#define J30D_REG_SYS_CFG1            0x93
#define J30D_REG_SYS_CFG2            0x94
#define J30D_REG_SYS_CH1_CFG         0x95
#define J30D_REG_SYS_CH2_CFG         0x96
#define J30D_REG_BIAS_CFG            0x97

/* Sleep-mode INT threshold, 16 bits MSB first, two's complement.
 * LSB = 1mV * 2^-8, so 0x8000 is -128mV. Compared after every
 * wake-and-convert in sleep; SYS_CFG2's SLEEP_POL selects which side
 * triggers. EEPROM-backed; the driver writes it at init only when
 * interrupt mode is enabled. */
#define J30D_REG_T_THRESHOLD_SLEEP_MSB 0xAC
#define J30D_REG_T_THRESHOLD_SLEEP_LSB 0xAD

/* 0x8000 = -128mV, the most negative value the register expresses. At or
 * below the bottom of the range under either VT_SCALE setting, so every
 * sample clears it -- which is what makes INT usable as a data-ready
 * signal. Written only when interrupt mode is enabled. */
#define J30D_INT_THRESHOLD_MSB         0x80u
#define J30D_INT_THRESHOLD_LSB         0x00u

#define J30D_REG_CH1_OFF_MSB         0xAE
#define J30D_REG_CH1_OFF_LSB         0xAF
#define J30D_REG_CH1_GAIN_MSB        0xB0
#define J30D_REG_CH1_GAIN_LSB        0xB1
#define J30D_REG_CH2_OFF_MSB         0xB5
#define J30D_REG_CH2_OFF_LSB         0xB6
#define J30D_REG_CH2_GAIN_MSB        0xB7
#define J30D_REG_CH2_GAIN_LSB        0xB8

/* Production/calibration data -- a 48-byte block written at factory
 * calibration time, read out once at init and parsed into h->calib (see
 * j30d_calib.h). Offsets are relative to J30D_REG_CUSTOM_MEMORY_START */
#define J30D_REG_CUSTOM_MEMORY_START      0xC0
#define J30D_CUSTOM_MEMORY_SIZE           48    /* 0xC0 to 0xEF inclusive */
#define J30D_CUSTOM_MEMORY_CRC_LEN        46    /* 0xC0 to 0xED inclusive (excl. CRC bytes) */

#define J30D_CAL_OFF_PROD_TIMESTAMP       0x00  /* 4 bytes, big-endian */
#define J30D_CAL_OFF_UNIQUE_ID            0x04  /* 4 bytes, big-endian */
#define J30D_CAL_OFF_CHIP_REVISION        0x08  /* 1 byte */
#define J30D_CAL_OFF_LOT_NUMBER           0x09  /* 2 bytes, big-endian */
#define J30D_CAL_OFF_WAFER_NUMBER         0x0B  /* 2 bytes, big-endian */
#define J30D_CAL_OFF_SENSOR_OFFSET_1      0x0D  /* 2 bytes, Q8.8 (see j30d_calib.h) */
#define J30D_CAL_OFF_SENSOR_GAIN_1        0x11  /* 2 bytes, Q8.8 */
#define J30D_CAL_OFF_SENSOR_OFFSET_2      0x15  /* 2 bytes, Q8.8 */
#define J30D_CAL_OFF_SENSOR_GAIN_2        0x19  /* 2 bytes, Q8.8 */
#define J30D_CAL_OFF_ALPHA_FACTOR         0x1D  /* 2 bytes -- not currently parsed, see j30d_calib.h */
#define J30D_CAL_OFF_CHECKSUM             0x2E  /* 2 bytes, big-endian, CRC16/CCITT over the first 46 bytes */

#endif /* J30D_REGS_H_ */
