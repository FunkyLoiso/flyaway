#ifndef ITG_3200_REGISTERS_INCLUDED
#define ITG_3200_REGISTERS_INCLUDED

#define ITG3200_TWI_ADDR        (0x68)

#define TEMP_COUNTS_PER_DEG_C   (280)        /* counts per degree C */
#define TEMP_OFFSET             (-13200)     /* temperature sensor offset */
#define TEMP_REF_DEG            (35)         /* reference temp (degrees C) */
#define SCALE_LSB_PER_DPS       (14.375)     /* LSB's per dgree/s */

/** \brief HMC5883L Register Addresses */
/** @{ */

#define ITG3200_WHOAMI          (0x00)       /* chip ID - I2C address */
#define ITG3200_SMPLRT_DIV      (0x15)       /* sample rate divider */
#define ITG3200_DLPF_FS         (0x16)       /* full scale & dig low pass */
#define ITG3200_INT_CFG         (0x17)       /* interrupt config */
#define ITG3200_INT_STATUS      (0x1a)       /* interrupt status */
#define ITG3200_TEMP_OUT_H      (0x1b)       /* temperature out - MSB */
#define ITG3200_TEMP_OUT_L      (0x1c)       /* temperature out - LSB */
#define ITG3200_GYRO_XOUT_H     (0x1d)       /* gyro X out - MSB */
#define ITG3200_GYRO_XOUT_L     (0x1e)       /* gyro X out - LSB */
#define ITG3200_GYRO_YOUT_H     (0x1f)       /* gyro Y out - MSB */
#define ITG3200_GYRO_YOUT_L     (0x20)       /* gyro Y out - LSB */
#define ITG3200_GYRO_ZOUT_H     (0x21)       /* gyro Z out - MSB */
#define ITG3200_GYRO_ZOUT_L     (0x22)       /* gyro Z out - LSB */
#define ITG3200_PWR_MGM         (0x3e)       /* power management */

/** @} */

/** \brief HMC5883L Register Bit Definitions */
/** @{ */

/* ITG3200_WHOAMI (0x00), defaults to the ITG3200 I2C bus address */

#define ITG3200_ID_VAL          (ITG3200_TWI_ADDR)

/* ITG3200_DLPF_FS (0x16) */

#define DLPF_CFG_256HZ          (0x00)       /* 256Hz  low pass b/w */
#define DLPF_CFG_188HZ          (0x01)       /* 188Hz  low pass b/w */
#define DLPF_CFG_98HZ           (0x02)       /* 98Hz   low pass b/w */
#define DLPF_CFG_42HZ           (0x03)       /* 42Hz   low pass b/w */
#define DLPF_CFG_20HZ           (0x04)       /* 20Hz   low pass b/w */
#define DLPF_CFG_10HZ           (0x05)       /* 10Hz   low pass b/w */
#define DLPF_CFG_5HZ            (0x06)       /* 5Hz    low pass b/w */
#define DLPF_CFG_2100HZ         (0x07)       /* 2.1KHz low pass b/w */
#define FS_SEL_2000             (0x18)       /* +/- 2000 deg/sec full scale */

/* ITG3200_INT_CFG (0x17) */

#define INT_CFG_RAW_RDY_EN      (0x01)       /* data available interrupt */
#define INT_CFG_ITG_RDY_EN      (0x04)       /* device ready int */
#define INT_CFG_ANYRD_2CLEAR    (0x10)       /* clr latch on any reg read */
#define INT_CFG_LATCH_INT_EN    (0x20)       /* latch until int cleared */
#define INT_CFG_OPEN            (0x40)       /* open drain for int output */
#define INT_CFG_ACTL            (0x80)       /* int is active low */

/* ITG3200_INT_STATUS (0x1a) */

#define INT_STATUS_RAW_DATA_RDY (0x01)       /* raw data is ready */
#define INT_STATUS_ITG_RDY      (0x04)       /* device (PLL) is ready */

/* ITG3200_PWR_MGM (0x3e) */

#define PWR_MGM_CLK_SEL_INT     (0x00)       /* internal oscillator */
#define PWR_MGM_CLK_SEL_X       (0x01)       /* PLL w/ X gyro reference */
#define PWR_MGM_CLK_SEL_Y       (0x02)       /* PLL w/ Y gyro reference */
#define PWR_MGM_CLK_SEL_Z       (0x03)       /* PLL w/ Z gyro reference */
#define PWR_MGM_CLK_SEL_EXT_32K (0x04)       /* PLL w/ external 32.768 KHz */
#define PWR_MGM_CLK_SEL_EXT_19M (0x05)       /* PLL w/ external 19.2 MHz */
#define PWR_MGM_STBY_ZG         (0x08)       /* put Z gyro in standby mode */
#define PWR_MGM_STBY_YG         (0x10)       /* put Y gyro in standby mode */
#define PWR_MGM_STBY_XG         (0x20)       /* put X gyro in standby mode */
#define PWR_MGM_SLEEP           (0x40)       /* enable low power sleep mode */
#define PWR_MGM_H_RESET         (0x80)       /* reset device */

#endif
