#ifndef __JAG_BUBLIFUK_H__
#define __JAG_BUBLIFUK_H__

// Why can't I pass typedef or enum in Arduino?
// http://stackoverflow.com/a/18159078
enum bot_state_t {
  STATE_ARM_DOWN,
  STATE_ARM_UP,
  STATE_FAN_BLOW,
  STATE_FAN_BLOW_IN_PROGRESS,

  STATE_DETACH_SERVO,
  STATE_INITIAL
};


#endif
