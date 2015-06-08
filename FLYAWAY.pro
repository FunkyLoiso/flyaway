# ----------------------------------------------------
# This file is generated by the Qt Visual Studio Add-in.
# ------------------------------------------------------

TEMPLATE = app
CONFIG += console
CONFIG -= qt
LIBS += -lwiringPi
HEADERS += ./ADXL345.h \
    ./ADXL345_registers.h \
    ./HMC5883.h \
    ./HMC5883_registers.h \
    ./ITG3200.h \
    ./ITG3200_registers.h \
    ./logging.h \
    ./input.h \
    ./sensors.h \
    ./vector.h \
    ./BMP085.h \
    ./BMP085_registers.h \
    twos_complement.h
SOURCES += ./ADXL345.c \
    ./HMC5883.c \
    ./ITG3200.c \
    ./main.c \
    ./BMP085.c \
    sensors.c

CONFIG(debug, debug|release) {
  DEFINES += _DEBUG
}
