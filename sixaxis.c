#include "sixaxis.h"

#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "logging.h"

#define JS_EVENT_BUTTON 0x01 // button pressed/released
#define JS_EVENT_AXIS   0x02 // joystick moved
#define JS_EVENT_INIT   0x80 // initial state of device

struct {
  uint32_t time;
  int16_t value;
  uint8_t type;
  uint8_t number;
}event;

static int fd = 0;
static sixaxis_button_callback button_cb;

int sixaxis_init(const char* device, sixaxis_button_callback button_callback){
  fd = open(device, O_RDONLY | O_NONBLOCK);
  if(!fd) {
    LOG_ERROR_ERRNO("Failed to open sixaxis at %s", device);
    return 10;
  }

  button_cb = button_callback;

  return 0;
}

int sixaxis_update(sixaxis_button_state* buttons, sixaxis_axis_state* axes){
  if(!fd) return 10;

  while(read(fd, &event, sizeof(event)) > 0) { /* process all events in queue if any */
    event.type &= ~JS_EVENT_INIT; /* treat init events like any other */
    if(JS_EVENT_BUTTON == event.type) {
      if(event.number < SIXAXIS_BUTTON_COUNT) {
        buttons->button[event.number] = event.value;
        if(button_cb) {
          button_cb(event.number, event.value);
        }
      }
    }
    else if(JS_EVENT_AXIS == event.type) {
      if(event.number < SIXAXIS_AXIS_COUNT) {
        axes->axis[event.number] = event.value;
      }
    }
  }

  if(errno != EAGAIN) { /* EAGAIN is returned when queue is empty */
    return errno;
  }
  return 0;
}
