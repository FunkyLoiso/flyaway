#ifndef SIXAXIS_H
#define SIXAXIS_H

#include <stdint.h>
#include <stdbool.h>

#define SIXAXIS_AXIS_COUNT 4

enum {
  SIXAXIS_SELECT = 0,
  SIXAXIS_L3,
  SIXAXIS_R3,
  SIXAXIS_START,
  SIXAXIS_UP,
  SIXAXIS_RIGHT,
  SIXAXIS_DOWN,
  SIXAXIS_LEFT,
  SIXAXIS_L2,
  SIXAXIS_R2,
  SIXAXIS_L1,
  SIXAXIS_R1,
  SIXAXIS_TRIANGLE,
  SIXAXIS_CIRCLE,
  SIXAXIS_SQUARE,
  SIXAXIS_CROSS,
  SIXAXIS_PS,
  SIXAXIS_BUTTON_COUNT
};

typedef union {
  uint8_t button[SIXAXIS_BUTTON_COUNT];

  uint8_t select;
  uint8_t l3;
  uint8_t r3;
  uint8_t start;
  uint8_t up;
  uint8_t right;
  uint8_t down;
  uint8_t left;
  uint8_t l2;
  uint8_t r2;
  uint8_t l1;
  uint8_t r1;
  uint8_t tri;
  uint8_t cir;
  uint8_t squ;
  uint8_t cro;
  uint8_t ps;
} sixaxis_button_state;

typedef union{
  int16_t axis[SIXAXIS_AXIS_COUNT];

  int16_t left_hor; // <0 is left, >0 is right
  int16_t left_ver; // <0 is up, >0 is down
  int16_t right_hor;
  int16_t right_ver;
} sixaxis_axis_state;

typedef void (*sixaxis_button_callback) (uint8_t button, bool pressed);

int sixaxis_init(const char* device, sixaxis_button_callback button_callback);
int sixaxis_update(sixaxis_button_state* buttons, sixaxis_axis_state* axes);

#endif // SIXAXIS_H
