//////////Libraries////////////////////////
//github.com/reeedstudio/libraries/tree/master/motordriver_4wd
//github.com/Seeed-Studio/SeeedMotorShieldV2
//github.com/pjpmarques/ChainableLED
// 0-255 setColorRGB(0, red, green, blue)

#include "motordriver_4wd.h"
#include "seeed_pwm.h"

#include <ChainableLED.h>
#define DELAY 500
#define DELAY 500

#define STATE_Forward 0
#define STATE_StopBot 1
#define STATE_BackUp 2
#define STATE_TurnLeft 3

//Might need digital pins.
ChainableLED led(2, 3, 1); //(pin, pin, number of LEDs)

//when pressed bump = 1, when release bump = 0
int bumpButton = 11;
int bump = 0;
int rgbLed = 0;
int state = 0;

void setup ()
{
    MOTOR.init();
    pinMode(bumpButton, INPUT);
    Serial.begin(9600);
}

void loop ()
{
  //Did we bump?
  bump = digitalRead(bumpButton);

  Serial.println(bump);
  Serial.println(state);

  switch (state)
  {
    case STATE_Forward: forward();
    case STATE_StopBot: stopBot();
    case STATE_BackUp: backUp();
    case STATE_TurnLeft: turnLeft();
  }
}

void forward ()
{
  while (!digitalRead(bumpButton))
  {
    led.setColorRGB( 0, 0, 255, 0); //LED to green

    MOTOR.setSpeedDir1(20, DIRF);
    MOTOR.setSpeedDir2(20, DIRR);
  }
  state = 1;
}

void stopBot ()
{
  led.setColorRGB( 0, 255, 0, 0); //LED to Red
  MOTOR.setStop1();
  MOTOR.setStop2();
  state = 2;
  delay(DELAY * 2);
}

void backUp ()
{
  led.setColorRGB( 0, 0, 0, 255); //LED to blue
  MOTOR.setSpeedDir1(10, DIRR);
  MOTOR.setSpeedDir2(10, DIRF);
  state = 3;
  delay(DELAY);
}

void turnLeft ()
{
  MOTOR.setSpeedDir1(30, DIRF);
  MOTOR.setSpeedDir2(30, DIRF);
  state = 0;
  delay(555);
}

