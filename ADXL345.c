/*
Basic readout of ADXL345v2 accelerometer via I2C

Oryginal code taken from the very bottom of this page:
http://www.raspberrypi.org/forums/viewtopic.php?t=55834

Updated by Jan Balewski, August 2014
*/

#include <wiringPiI2C.h>
#include "ADXL345.h"
#include "ADXL345_register.h"
#include "logging.h"

int adxl345_fd = -1;
unsigned char adxl345_adr = 0x00;

int ADXL345_init(unsigned char adr) {

  /*  int wiringPiI2CSetup (int devId) ;
  The return value is the standard Linux filehandle,
  or -1 if any error – in which case, you can consult errno as usual.
  */

  int fd = wiringPiI2CSetup(adr);
  if (fd < 0)  {
    return errno;
  }

  int rc = wiringPiI2CWriteReg8(fd, POWER_CTL, PCTL_MEASURE); //put into measure mode
  if (rc) return rc;

  rc = wiringPiI2CWriteReg8(fd, DATA_FORMAT, FULL_RES | RANGE_PM_4g);
  if (rc) return rc;

  adxl345_fd = fd;
  adxl345_adr = adr;
  LOG_DEBUG("ADXL345 on adr 0x%x init OK\n", adr);
  return 0;
}
int ADXL345_read(short* acc_x, short* acc_y, short* acc_z) {

}

//==========================================
//==========================================
int select_device() {

  if (ioctl(fd, I2C_SLAVE, devId) < 0) {
    fprintf(stderr, "device 0x%x ADXL345v2 not present\n", devId);
    return false;
  }

  return true;
}


//==========================================
//==========================================
bool ADXL345v2::writeToDevice(char * buf, int len){
  if (write(fd, buf, len) != len)   {
    fprintf(stderr, "Can't write to device ADXL345v2 buf=%s len=%d\n", fd, buf, len);
    return false;
  }
  return true;
}


//===============
bool  ADXL345v2::readXYZ(short &x, short &y, short &z) {
  assert(fd > 0); // crash if port was not opened earlier
  if (!selectDevice())   return false;
  //   printf("selectDevice(fd,ADXL345v2...)  passed\n");
  char buf[7];
  buf[0] = 0x32;     // This is the register we wish to read from
  if (!writeToDevice(buf, 2))     return false;

  if (read(fd, buf, 6) != 6) {  // Read back data into buf[]
    printf("Unable to read from slave for ADXL345v2\n");
    return false;
  }
  else {
    x = (buf[1] << 8) | buf[0];
    y = (buf[3] << 8) | buf[2];
    z = (buf[5] << 8) | buf[4];
  }
  return true;
}


//==========================================
//==========================================
//==========================================
int ADXL345v2::init()  {

  /*  int wiringPiI2CSetup (int devId) ;
      The return value is the standard Linux filehandle,
      or -1 if any error – in which case, you can consult errno as usual.
      */

  fd = wiringPiI2CSetup(devId);
  if (fd < 0)  {
    printf("failed init  ADXL345v2, ret=%d\n", fd); exit(1);
  }


  assert(fd > 0); // crash if port was not opened earlier
  char buf[6];       // Buffer for data being read/ written on the i2c bus

  if (!selectDevice()) return -1;

  buf[0] = 0x2d;                   // Commands for performing a ranging
  buf[1] = 0x18;

  if (!writeToDevice(buf, 2))  return -2;


  buf[0] = 0x31;              // Commands for performing a ranging
  buf[1] = 0x0A; //09 4g , A 8g

  if (!writeToDevice(buf, 2))  return -3;
  printf("ADXL345v2::init() OK\n");
  return 0;
}

