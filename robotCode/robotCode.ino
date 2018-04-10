//////////Libraries////////////////////////
//github.com/reeedstudio/libraries/tree/master/motordriver_4wd
//github.com/Seeed-Studio/SeeedMotorShieldV2
//github.com/pjpmarques/ChainableLED
// 0-255 setColorRGB(0, red, green, blue)

#include "motordriver_4wd.h"
#include "seeed_pwm.h"

#include <ChainableLED.h>
#define DELAY 500

//Might need digital pins.
ChainableLED led(A0, A1, 1); //(pin, pin, number of LEDs)

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

  switch (state)
  {
    case 0: forward();
    case 1: stopBot();
    case 2: backUp();
    case 3: turnLeft();
  }
}

void forward ()
{
  if (bump == 0)
  {
    led.setColorRGB( 0, 0, 255, 0); //LED to green

    MOTOR.setSpeedDir1(10, DIRF);
    MOTOR.setSpeedDir2(10, DIRR);
  }
  else
  {
    state = 1;
  }
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
  MOTOR.setSpeedDir1(10, DIRF);
  MOTOR.setSpeedDir2(10, DIRF);
  state = 0;
  delay(DELAY);
}
