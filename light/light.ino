#include "motordriver_4wd.h"
#include "seeed_pwm.h"

enum {LEFT, RIGHT} state;

void setup() {
  digitalWrite(A0, INPUT_PULLUP);
  MOTOR.init();
  MOTOR.setSpeedDir1(16, DIRF);
  MOTOR.setSpeedDir2(8, DIRR);
  state = RIGHT;
  delay(100);

  Serial.begin(9600);
}

void loop() {
  int val = analogRead(A0);

  Serial.println(val);

  if (val > 100) {
    if (state == LEFT) {
      MOTOR.setSpeedDir1(16, DIRF);
      MOTOR.setSpeedDir2(8, DIRR);
      state = RIGHT;
      delay(1000);
    } else if (state == RIGHT) {
      MOTOR.setSpeedDir1(8, DIRF);
      MOTOR.setSpeedDir2(16, DIRR);
      state = LEFT;
      delay(1000);
    }
  }
}
