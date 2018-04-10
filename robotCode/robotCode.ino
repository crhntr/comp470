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
#define STATE_TurnRight 4

//Might need digital pins.
ChainableLED led(3, 2, 5); //(pin, pin, number of LEDs)

//when pressed bump = 1, when release bump = 0
int bumpButton = 11;
int bump = 0;
int rgbLed = 0;
int state = 0;

void setup ()
{
    led.init();
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
    case STATE_Forward: led.setColorRGB( 0, 0, 255, 0); forward(); break;
    case STATE_StopBot: led.setColorRGB( 0, 255, 0, 0); stopBot(); break;
    case STATE_BackUp: led.setColorRGB( 0, 0, 0, 255); backUp(); break;
    case STATE_TurnLeft: led.setColorRGB( 0, 127, 0, 127); turnLeft(); break;
    case STATE_TurnRight: led.setColorRGB( 0, 255, 165, 0); turnRight(); break;
  }
}

void forward ()
{
  while (!digitalRead(bumpButton))
  {
    MOTOR.setSpeedDir1(20, DIRF);
    MOTOR.setSpeedDir2(20, DIRR);
  }
  state = STATE_StopBot;
}

void stopBot ()
{
  //LED to Red
  MOTOR.setStop1();
  MOTOR.setStop2();
  state = STATE_BackUp;
  delay(DELAY * 2);
}

void backUp ()
{
  MOTOR.setSpeedDir1(10, DIRR);
  MOTOR.setSpeedDir2(10, DIRF);
  state = STATE_TurnLeft;
  delay(DELAY);
}

void turnLeft ()
{
  MOTOR.setSpeedDir1(30, DIRF);
  MOTOR.setSpeedDir2(30, DIRF);
  state = STATE_Forward;
  delay(555);
}

void turnRight ()
{
  MOTOR.setSpeedDir1(30, DIRF);
  MOTOR.setSpeedDir2(30, DIRF);
  state = STATE_Forward;
  delay(555);
}
