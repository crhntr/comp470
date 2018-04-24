#include <motordriver_4wd.h>

#define STATE_WANDER 0
#define STATE_RETURN 1
#define STATE_CLIMB  2
#define STATE_END    3

int state;

// ticks per rotation
#define TPR 72

// Interrupt Service Routines for encoders

volatile long left_encoder_count = 0, right_encoder_count = 0;
int left_dirn = 1, right_dirn = 1;

void leftEncoder () { left_encoder_count += left_dirn; }
void rightEncoder () { right_encoder_count += right_dirn; }

void init () {
  state = STATE_WANDER;

  attachInterrupt(0, leftEncoder, CHANGE);
  attachInterrupt(1, rightEncoder, CHANGE);
}

void loop () {
  switch (state) {
    case STATE_WANDER:
    
    break;
    case STATE_RETURN:
    
    break;
    case STATE_CLIMB:
    
    break;
    case STATE_END:
    
    break;
  }
}
