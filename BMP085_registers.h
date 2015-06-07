#ifndef BMP085_REGISTERS_H_INCLUDED
#define BMP085_REGISTERS_H_INCLUDED

/* TWI/I2C address (write @ 0xee on bus, read @ 0xef on bus) */
#define BMP_TWI_ADDR               (0x77)

/** \brief BMP Register Addresses */
/** @{ */
#define BMP_CHIP_ID                (0xd0)   /* chip ID - always 0x55 */
#define BMP_CHIP_VERSION           (0xd1)   /* chip revision */
#define BMP_SOFT_RESET             (0xe0)   /* reset device */
#define BMP_CONTROL                (0xf4)   /* device control register */
#define BMP_DATA_MSB               (0xf6)   /* temp. or press. data MSB */
#define BMP_DATA_LSB               (0xf7)   /* temp. or press. data LSB */
#define BMP_DATA_XLSB              (0xf8)   /* press. data XLSB (19 bit data) */
/** @} */

/* EEPROM Calibration Coefficient Addresses (MSB | LSB) */

#define BMP_EEPROM_ADDR            (0xaa)   /* BMP085/BMP180 EEPROM base address */
#define BMP_EEPROM_SIZE_BYTES      (22)     /* BMP085/BMP180 EEPROM size (bytes) */

/** \brief BMP Register Bit Definitions */
/** @{ */

/* BMP_CHIP_ID (0xd0) */

#define BMP085_ID_VAL              (0x55)   /* BMP085 chip id value */
#define BMP085_VER_VAL             (0x01)   /* BMP085 chip version value */

#define BMP180_ID_VAL              (0x55)   /* BMP180 chip id value */
#define BMP180_VER_VAL             (0x02)   /* BMP180 chip version value */

/* BMP_SOFT_RESET (0xe0) */

#define BMP_RESET_CMD              (0xb6)   /* soft reset command */

/* BMP_CONTROL (0xf4) */

#define BMP_TEMP_READ              (0x2e)   /* read temperature */
#define BMP_PRESS_READ             (0x34)   /* read pressure (@ osrs = 0) */

/* Operating Ranges */

#define BMP_MIN_hPa                (300)    /* +9 000 (m) above sea level */
#define BMP_MAX_hPa                (1100)   /* -500 (m) above sea level */

#define BMP_MIN_COUNTS             (0)
#define BMP_MAX_COUNTS             (0xfffful)

#endif
