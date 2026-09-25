/*
 * j30d_regs.h
 *
 * Device register map and bit definitions. Internal to the driver --
 * customers configure the sensor through j30d_config.h / j30d.h, not by
 * poking these addresses directly, so this header is not included by
 * j30d.h itself (only by j30d.c and j30d_config.c).
 *
 * Prefixed J30D_ (rather than the bare names used in the legacy
 * jirs30d.h) so this can coexist in the same build as the existing driver
 * during the transition without macro collisions.
 *
 * Values carried over unchanged from the existing jirs30d.h register map.
 */

#ifndef J30D_REGS_H_
#define J30D_REGS_H_

#include <stdint.h>

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
/* Bit 3 (OBJ_DRDY) intentionally not defined here -- this driver never
 * reads the OBJ register, see J30D_REG_OBJ_MSB's removal note below. */

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

/* J30D_REG_OBJ_MSB (0x10) intentionally not defined here. The ASIC also
 * computes an on-chip "object temperature" from the IR reading under an
 * emissivity/distance assumption, but this driver targets heat-flux
 * measurement, where that conversion doesn't apply -- so the register,
 * its DRDY bit, and the corresponding data field are all left out rather
 * than carried along unused. */
#define J30D_REG_AMB_MSB             0x16
#define J30D_REG_CAL_MSB             0x19   /* ch1 cal, 6 bytes covers ch1+ch2 */
#define J30D_REG_RAW_MSB             0x22   /* ch1/ch2/amb raw, 9 bytes */

#define J30D_REG_COMMAND             0x30
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
#define J30D_SYS_CFG1_OSRT_8192      ((uint8_t)(6U << J30D_SYS_CFG1_OSRT_AMB_Pos))   /* was misspelled _8196 */
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

/* Names below verified against the NSA3300 datasheet's own register table
 * (0x94), not carried over blind from the legacy driver -- several of the
 * legacy names were wrong in a way that matters. In particular, bits 2 and
 * 4 are NOT power/sleep enables (despite what their old names implied):
 * bit 2 is the polarity of the INT output during Sleep mode, and bits 5:4
 * are a PGA power/noise trade-off setting, not a device power-down bit.
 * Nothing here controls whether the device sleeps -- that's COMMAND's
 * SLEEP_EN (0x30 bit 5) only. */
#define J30D_REG_SYS_CFG2               0x94
#define J30D_SYS_CFG2_SWAP_Msk          ((uint8_t)(1U << 0))   /* input_swap: 1 = CH inputs bonded in reverse polarity */
#define J30D_SYS_CFG2_VT_SCALE_Msk      ((uint8_t)(1U << 1))   /* 0 = +-16mV full range, 1 = +-128mV */
#define J30D_SYS_CFG2_SLEEP_POL_Msk     ((uint8_t)(1U << 2))   /* SLEEP_OUT_Polarity: which side of the threshold triggers INT in Sleep mode -- not a sleep enable */
#define J30D_SYS_CFG2_RELAY_POL_Msk     ((uint8_t)(1U << 3))   /* Relay_OUT_Polarity: which side of the threshold drives Relay output high -- not a relay enable */
#define J30D_SYS_CFG2_PGA_PWR_Pos       4U           /* PGA_PWR<1:0>: power/noise trade-off, not a power-down bit */
#define J30D_SYS_CFG2_PGA_PWR_Msk       ((uint8_t)(3U << J30D_SYS_CFG2_PGA_PWR_Pos))
#define J30D_SYS_CFG2_SERIAL_FILT_Msk   ((uint8_t)(1U << 6))   /* SERIAL_filter_en: IIC input deglitch filter */
#define J30D_SYS_CFG2_ADC_DITH_Msk      ((uint8_t)(1U << 7))

#define J30D_REG_SYS_CH1_CFG         0x95
#define J30D_REG_SYS_CH2_CFG         0x96
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

#define J30D_REG_BIAS_CFG            0x97
#define J30D_BIAS_CFG_DAC_Msk        ((uint8_t)(15U << 0))
#define J30D_BIAS_CFG_5_16_AVDD      ((uint8_t)(0x0D))

/* Per-channel offset/gain trim registers -- selected at init based on
 * calibration/unit version (see JIRS30D_CHANNEL handling in the legacy
 * driver). */
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
 * j30d_calib.h). Offsets are relative to J30D_REG_CUSTOM_MEMORY_START,
 * matching the legacy driver's CUSTOM_MEMORY_* layout exactly. */
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
