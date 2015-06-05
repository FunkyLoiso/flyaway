#ifndef HMC5883_REGISTERS_H_INCLUDED
#define HMC5883_REGISTERS_H_INCLUDED

#include <stdint.h>

/* TWI/I2C slave address (write @ 0x3c on bus, read @ 0x3d on bus) */
#define HMC5883_TWI_ADDR       (0x1e)

/* Signed axis data sample resolution (bits) */
#define HMC5883_DATA_RESOLUTION (12)

/* Value signalizing data overflow */
#define DATA_OUTPUT_OVERFLOW    ((int16_t)0xf000)

/** \brief Data scaling - varies by range/gain setting */
/** @{ */

#define SCALE_0_9GA             (1280)  /* 0.9 Ga (1280 counts / Gauss) */
#define SCALE_1_2GA             (1024)  /* 1.2 Ga (1024 counts / Gauss) */
#define SCALE_1_9GA             (768)   /* 1.9 Ga (768 counts / Gauss) */
#define SCALE_2_5GA             (614)   /* 2.5 Ga (614 counts / Gauss) */
#define SCALE_4_0GA             (415)   /* 4.0 Ga (415 counts / Gauss) */
#define SCALE_4_6GA             (361)   /* 4.6 Ga (361 counts / Gauss) */
#define SCALE_5_5GA             (307)   /* 5.5 Ga (307 counts / Gauss) */
#define SCALE_7_9GA             (219)   /* 7.9 Ga (219 counts / Gauss) */

/** @} */

/** \brief Device ID Definitions */
/** @{ */

#define ID_A_DEFAULT            (0x48)  /* normal value of ID register A */
#define ID_B_DEFAULT            (0x34)  /* normal value of ID register B */
#define ID_C_DEFAULT            (0x33)  /* normal value of ID register C */
#define HMC5883_DEV_ID         (0x483433)  /* combined ID value */

/** @} */

/** \brief HMC5883 Register Addresses */
/** @{ */

#define HMC5883_CONFIG_REG_A   (0x00)  /* configuration register A */
#define HMC5883_CONFIG_REG_B   (0x01)  /* configuration register B */
#define HMC5883_MODE_REG       (0x02)  /* mode register */
#define HMC5883_MAG_X_HI       (0x03)  /* X mag reading - MSB */
#define HMC5883_MAG_X_LO       (0x04)  /* X mag reading - LSB */
#define HMC5883_MAG_Z_HI       (0x05)  /* Z mag reading - MSB */
#define HMC5883_MAG_Z_LO       (0x06)  /* Z mag reading - LSB */
#define HMC5883_MAG_Y_HI       (0x07)  /* Y mag reading - MSB */
#define HMC5883_MAG_Y_LO       (0x08)  /* Y mag reading - LSB */
#define HMC5883_STATUS_REG     (0x09)  /* device status */
#define HMC5883_ID_REG_A       (0x0a)  /* ID register A */
#define HMC5883_ID_REG_B       (0x0b)  /* ID register B */
#define HMC5883_ID_REG_C       (0x0c)  /* ID register C */

/** @} */

/** \brief HMC5883 Register Bit Definitions */
/** @{ */

/* HMC5883_CONFIG_REG_A (0x00) */

#define MEAS_MODE               (0x03)  /* measurement mode mask (2 bits) */
#define MEAS_MODE_NORM          (0x00)  /* normal measurement mode */
#define MEAS_MODE_POS           (0x01)  /* positive bias */
#define MEAS_MODE_NEG           (0x02)  /* negative bias */

#define DATA_RATE               (0x1c)  /* data rate mask (3 bits) */
#define DATA_RATE_0_75HZ        (0x00)  /* 0.75 Hz */
#define DATA_RATE_1_5HZ         (0x04)  /* 1.5 Hz */
#define DATA_RATE_3HZ           (0x08)  /* 3 Hz */
#define DATA_RATE_7_5HZ         (0x0c)  /* 7.5 Hz */
#define DATA_RATE_15HZ          (0x10)  /* 15 Hz */
#define DATA_RATE_30HZ          (0x14)  /* 30 Hz */
#define DATA_RATE_75HZ          (0x18)  /* 75 Hz */

/* HMC5883_CONFIG_REG_B (0x01) */

#define GAIN_0_9GA              (0x00)  /* +/- 0.9 Ga (1280 counts/ Gauss) */
#define GAIN_1_2GA              (0x20)  /* +/- 1.2 Ga (1024 counts/ Gauss) */
#define GAIN_1_9GA              (0x40)  /* +/- 1.9 Ga (768 counts / Gauss) */
#define GAIN_2_5GA              (0x60)  /* +/- 2.5 Ga (614 counts / Gauss) */
#define GAIN_4_0GA              (0x80)  /* +/- 4.0 Ga (415 counts / Gauss) */
#define GAIN_4_6GA              (0xa0)  /* +/- 4.6 Ga (361 counts / Gauss) */
#define GAIN_5_5GA              (0xc0)  /* +/- 5.5 Ga (307 counts / Gauss) */
#define GAIN_7_9GA              (0xe0)  /* +/- 7.9 Ga (219 counts / Gauss) */

/* HMC5883_MODE_REG (0x02) */

#define MODE_CONTIN             (0x00)  /* continuous conversion mode */
#define MODE_SINGLE             (0x01)  /* single measurement mode */
#define MODE_IDLE               (0x02)  /* idle mode */
#define MODE_SLEEP              (0x03)  /* sleep mode */

/* HMC5883_STATUS_REG (0x09) */

#define STATUS_RDY              (0x01)  /* data ready */
#define STATUS_LOCK             (0x02)  /* data output locked */
#define STATUS_REN              (0x04)  /* internal voltage regulator enabled */

/* Self-test Definitions */

#define HMC5883_TEST_GAIN      GAIN_2_5GA  /* gain value during self-test */
#define HMC5883_TEST_X_MIN     550         /* min X */
#define HMC5883_TEST_X_NORM    766         /* normal X */
#define HMC5883_TEST_X_MAX     850         /* max X */
#define HMC5883_TEST_Y_MIN     550         /* min Y */
#define HMC5883_TEST_Y_NORM    766         /* normal Y */
#define HMC5883_TEST_Y_MAX     850         /* max Y */
#define HMC5883_TEST_Z_MIN     550         /* min Z */
#define HMC5883_TEST_Z_NORM    713         /* normal Z */
#define HMC5883_TEST_Z_MAX     850         /* max Z */

#endif