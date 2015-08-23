# ----------------------------------------------------
# This file is generated by the Qt Visual Studio Add-in.
# ------------------------------------------------------

TEMPLATE = app
CONFIG += console
CONFIG -= qt
LIBS += -lwiringPi -lconfig
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
    twos_complement.h \
    cpu_cycles.h \
    integration.h \
    linear_velocity_regulator.h \
    MadgwickAHRS.h \
    sensor_fusion.h \
    angle_regulator.h \
    math_funcs.h \
    altitude_regulator.h \
    differentiaton.h \
    motors_controller.h \
    PCA9685_registers.h \
    throttle_mixing.h \
    sixaxis.h
SOURCES += ./ADXL345.c \
    ./HMC5883.c \
    ./ITG3200.c \
    ./main.c \
    ./BMP085.c \
    sensors.c \
    integration.c \
    linear_velocity_regulator.c \
    MadgwickAHRS.c \
    sensor_fusion.c \
    angle_regulator.c \
    altitude_regulator.c \
    differentiaton.c \
    motors_controller.c \
    throttle_mixing.c \
    cpu_cycles.c \
    vector.c \
    twos_complement.c \
    sixaxis.c

CONFIG(debug, debug|release) {
  DEFINES += _DEBUG
}

QMAKE_CFLAGS += -Wno-missing-field-initializers
