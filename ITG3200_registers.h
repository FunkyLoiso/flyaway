/*
ITG3200.h
April 28, 2010
by: Jim Lindblom
*/

#ifndef ITG_3200_REGISTERS_INCLUDED
#define ITG_3200_REGISTERS_INCLUDED

// ITG3200 Register Defines
#define WHO_AM_I          0x00
#define	SMPL_RATE_DIVIDER 0x15
#define DIG_LOWPASS_FLTR	0x16
#define INTERRUPT_CONF    0x17
#define INTERRUPT_STATUS	0x1A
#define	TMP_H             0x1B
#define	TMP_L           	0x1C
#define	GX_H              0x1D
#define	GX_L              0x1E
#define	GY_H              0x1F
#define	GY_L              0x20
#define GZ_H              0x21
#define GZ_L              0x22
#define POWER_MGMNT     	0x3E

#endif
