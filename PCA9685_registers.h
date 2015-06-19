#ifndef PCA9685_registers_h__
#define PCA9685_registers_h__

#define PCA9685_MODE1           0x00
#define PCA9685_MODE2           0x01
#define PCA9685_SUBADDR1        0x02
#define PCA9685_SUBADDR2        0x03
#define PCA9685_SUBADDR3        0x04
#define PCA9685_ALLCALLADDR     0x05
#define PCA9685_LEDX_ON_L       0x06
#define PCA9685_LEDX_ON_H       0x07
#define PCA9685_LEDX_OFF_L      0x08
#define PCA9685_LEDX_OFF_H      0x09

#define PCA9685_ALL_LED_ON_L    0xFA
#define PCA9685_ALL_LED_ON_H    0xFB
#define PCA9685_ALL_LED_OFF_L   0xFC
#define PCA9685_ALL_LED_OFF_H   0xFD
#define PCA9685_PRESCALE        0xFE

#define PCA9685_NUMREGS         0xFF
#define PCA9685_MAXCHAN         0x10

#define LED_FULL                (1 << 4)
#define MODE1_SLEEP             (1 << 4)
#define MODE1_AUTO_INCREMENT    (1 << 5)
#define MODE2_INVRT             (1 << 4)
#define MODE2_OUTDRV            (1 << 2)

#define LED_N_ON_H(N)   (PCA9685_LEDX_ON_H + (4 * (N)))
#define LED_N_ON_L(N)   (PCA9685_LEDX_ON_L + (4 * (N)))
#define LED_N_OFF_H(N)  (PCA9685_LEDX_OFF_H + (4 * (N)))
#define LED_N_OFF_L(N)  (PCA9685_LEDX_OFF_L + (4 * (N)))

#endif // PCA9685_registers_h__
