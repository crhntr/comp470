//////////Libraries////////////////////////
//github.com/reeedstudio/libraries/tree/master/motordriver_4wd
//github.com/Seeed-Studio/SeeedMotorShieldV2
//github.com/pjpmarques/ChainableLED
// 0-255 setColorRGB(0, red, green, blue)

#include "motordriver_4wd.h"
#include "seeed_pwm.h"
#include <ChainableLED.h>

#define DELAY               500
#define NINETY              555                   //turns ~90 Degrees at robot speed 30, 30
#define SERVO_DELAY         1000                  //Time between servo rotations

#define SERVO_PIN           12                     // Any pin will do


                              //Rover State Machine
#define STATE_Forward       0
#define STATE_StopBot       1
#define STATE_BackUp        2
#define STATE_TurnLeft      3
#define STATE_TurnRight     4
#define STATE_TurnAbout     5


                              //Servo State Machine
#define SERVO_Zero          0
#define SERVO_Ninety        1
#define SERVO_OneEighty     2
#define SERVO_NinetyReturn  3

ChainableLED led(3, 2, 5);                        //(pin, pin, number of LEDs)

const int pingPin = SCL;                          //for ultra sonic sensor
const int bumpButton = 11;

int lenMicroSecondsOfPeriod = 20 * 1000;          // 20 milliseconds (ms)
int lenMicroSecondsOfPulse = 1.0 * 1000;          // 1.0 ms is 0 degrees
int bump = 0;                                     //when pressed bump = 1, when release bump = 0
int rover_state = 0;
int servo_state = 0;

int leftDetect = 0;
int rightDetect = 0;

void setup ()
{
    led.init();
    MOTOR.init();
    pinMode(bumpButton, INPUT);
    int dist_cm = Ping(pingPin);
    Serial.begin(9600);
}

void loop ()
{
  //Did we bump?
  bump = digitalRead(bumpButton);

  //Serial.println(bump);
  Serial.print("state: ");
  Serial.println(rover_state);

  switch (rover_state)
  {
    case STATE_Forward: 
        led.setColorRGB( 0, 0, 255, 0);           //Green
        forward(); 
      break;
    case STATE_StopBot: 
        led.setColorRGB( 0, 255, 0, 0);           //Red
        stopBot(); 
      break;
    case STATE_BackUp: 
        led.setColorRGB( 0, 0, 0, 255);           //Blue
        backUp(); 
      break;
    case STATE_TurnLeft: 
        led.setColorRGB( 0, 127, 0, 127);         //Purple
        turnLeft(); 
      break;
    case STATE_TurnRight: 
        led.setColorRGB( 0, 255, 165, 0);         //Orange
        turnRight(); 
      break;
    case STATE_TurnAbout:
        led.setColorRGB(0, 247, 11, 147);         //Pink
        turnAbout();
      break;
  }
}

void forward ()
{
  if(!bump)
  {
    MOTOR.setSpeedDir1(20, DIRF);
    MOTOR.setSpeedDir2(20, DIRR);
  }
  else
  {
    rover_state = STATE_StopBot;
    MOTOR.setSpeedDir1(10, DIRR);
    MOTOR.setSpeedDir2(10, DIRF);
    delay(DELAY/2);
  }
}

void stopBot ()
{
  MOTOR.setStop1();
  MOTOR.setStop2();
  sonicLookAbout();
  //delay(DELAY * 2);                              //Don't think we need a delay here anymore?
}

void backUp ()
{
  MOTOR.setSpeedDir1(10, DIRR);
  MOTOR.setSpeedDir2(10, DIRF);
  if(leftDetect < 20 && rightDetect < 20)
  {
    rover_state = STATE_TurnAbout;
  }
  else if(leftDetect < 20)                         //need to test actual distances
  {
    rover_state = STATE_TurnRight;
  }
  else
  {
    rover_state = STATE_TurnLeft;
  }

  leftDetect = 0;                                  //resetting values for next loop
  rightDetect = 0;
  
  delay(DELAY);
}

void turnLeft ()
{
  MOTOR.setSpeedDir1(30, DIRF);
  MOTOR.setSpeedDir2(30, DIRF);
  rover_state = STATE_Forward;
  delay(NINETY);
}

void turnRight ()
{
  MOTOR.setSpeedDir1(30, DIRR);
  MOTOR.setSpeedDir2(30, DIRR);
  rover_state = STATE_Forward;
  delay(NINETY);
}

void turnAbout()
{
  MOTOR.setSpeedDir1(30, DIRF);
  MOTOR.setSpeedDir2(30, DIRF);
  rover_state = STATE_Forward;
  delay(NINETY * 2);                                  //180 Degree turn
}


//This logic needs redone but is a foundation
void sonicLookAbout()
{
  Serial.print("servo state: ");
  Serial.println(servo_state);
    switch(servo_state){
    case SERVO_Zero:
        turnServo(lenMicroSecondsOfPulse * 0.75);
        servo_state = SERVO_Ninety;
        delay(SERVO_DELAY);
      break;
    case SERVO_Ninety:
        turnServo(lenMicroSecondsOfPulse * 1.5);
        servo_state = SERVO_OneEighty;
      delay(SERVO_DELAY);
      break;
    case SERVO_OneEighty:
        turnServo(lenMicroSecondsOfPulse * 4.0);
        servo_state = SERVO_NinetyReturn;
      delay(SERVO_DELAY); 
      break;   
    case SERVO_NinetyReturn:
        turnServo(lenMicroSecondsOfPulse * 1.5);
        servo_state = SERVO_Zero;                     //resets for next time
        rover_state = STATE_BackUp;                   //Exits the servo state 
      delay(SERVO_DELAY);
      break;
  }
}

void turnServo(double pulse)
{
  for(int i = 0; i < 40; i++)
   {  
     digitalWrite(SERVO_PIN, HIGH);
     // Delay for the length of the pulse
     delayMicroseconds(pulse);
    
     // Turn the voltage low for the remainder of the pulse
     digitalWrite(SERVO_PIN, LOW);
    
     // Delay this loop for the remainder of the period so we don't
     // send the next signal too soon or too late
     int dist_cm = Ping(pingPin);  

     if(servo_state == SERVO_Zero)                    //might have directions swapped
     {
        leftDetect = dist_cm;
     }
     else if(servo_state == SERVO_OneEighty)
     {
        rightDetect = dist_cm;
     }
     Serial.println(dist_cm);
     delayMicroseconds(lenMicroSecondsOfPeriod - pulse);
 } 
}

int Ping( int pingPin )
{
  long duration, cm; 

  // The PING is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:

  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING: a HIGH
  // pulse duration of which is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.

  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  cm = duration / 29 / 2; // 1cm/29us  

  return cm;
}
