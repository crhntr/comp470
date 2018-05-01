
#include "motordriver_4wd.h"
#include "seeed_pwm.h"
#include <ChainableLED.h>

#define SERVO_PIN           13

const int pingPin = SCL;                          //for ultra sonic sensor
const int bumpButton = 11;

//Pi Constants
const double PIE    = 3.14159265;
const double PIE_O2 = PIE/2.0;
const double PIE2   = PIE*2.0;


//-------- motor control 

void TurnLeft90();
void TurnRight90();
void Straight( int speed, int dirn );
void UpdateLoc();

//-------- dead reckoning 

// ticks per rotation
#define TPR 72
 
// robot measurements (mm)
#define RW    42.5  // radius wheel
#define D     158.0

// robot config variables
double x = 100.0, 
       y = 100.0, 
       dx = 0.0, 
       dy = 0.0;
double theta =  PI/2.0;

// encoder variables
volatile long left_encoder_count = 0, 
              right_encoder_count = 0;   
int left_dirn = 1, 
    right_dirn = 1;

//-------- model of environment 

double LEFT = 0.0;
double RIGHT = 1500.0;
double BOTTOM = 0.0;
double TOP = 1500.0;

float positionX = 0.0, 
      positionY = 0.0;

float map[7][7] = {
  {1, 1, 1, 1, 1, 1, 1},
  {1, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 1},
  {1, 1, 1, 1, 1, 1, 1},
}

#define SERVO_DELAY         1000

ChainableLED led(3, 2, 5);                        //(pin, pin, number of LEDs)

int lenMicroSecondsOfPeriod = 20 * 1000;          // 20 milliseconds (ms)
int lenMicroSecondsOfPulse = 1.0 * 1000;          // 1.0 ms is 0 degrees
int bump = 0;                                     //when pressed bump = 1, when release bump = 0
int rover_state = 0;
int servo_state = 0;

int leftDetect = 0;
int rightDetect = 0;

void LeftEncoder();
void TurnLeft90();
void TurnRight90();
void Straight( int speed, int dirn );


void setup() {
  led.init();
  MOTOR.init();
  pinMode(bumpButton, INPUT);
  int dist_cm = Ping(pingPin);
  attachInterrupt(0, leftEncoder, CHANGE);
  attachInterrupt(1, rightEncoder, CHANGE);
  Serial.begin(9600);
}

void loop() {
  UpdateLoc();

}

void TurnLeft90()
{
    right_encoder_count = left_encoder_count = 0;
    
    left_dirn = -1; 
    right_dirn = 1;
    
    MOTOR.setSpeedDir1(40, DIRR); 
    MOTOR.setSpeedDir2(40, DIRR); 
    
    while (right_encoder_count < 64)
    {
      delayMicroseconds(1);
    }

    MOTOR.setSpeedDir1(0, DIRF); 
    MOTOR.setSpeedDir2(0, DIRR);
}

void TurnRight90()
{
    right_encoder_count = left_encoder_count = 0;
    
    left_dirn = 1; 
    right_dirn = -1;
    
    MOTOR.setSpeedDir1(40, DIRF); 
    MOTOR.setSpeedDir2(40, DIRF); 
    while (left_encoder_count < 64)
    {
      delayMicroseconds(1);
    }

    MOTOR.setSpeedDir1(0, DIRF); 
    MOTOR.setSpeedDir2(0, DIRR);
}

void Straight( int speed, int dirn )
{
    //---- setup encoder variables
    left_dirn = dirn; 
    right_dirn = dirn;
    
    if (speed == 0)       //-- stop
    {
      MOTOR.setSpeedDir1(0, DIRF); 
      MOTOR.setSpeedDir2(0, DIRR);   
      return;
    }
    else if (dirn == 1)   //-- fwd
    {
      MOTOR.setSpeedDir1(speed+0.25*speed, DIRF); 
      MOTOR.setSpeedDir2(speed, DIRR); 
    }
    else                  //-- bwd
    {
      MOTOR.setSpeedDir1(speed+0.25*speed, DIRR); 
      MOTOR.setSpeedDir2(speed, DIRF); 
    }
}

void UpdateLoc()
{
  dx = PIE * RW * cos(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  x = x + dx;
  
  dy = PIE * RW * sin(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  y = y + dy;
  
  right_encoder_count = left_encoder_count = 0;
}

//======================================================================================
// Interrupt Service Routines for encoders
//======================================================================================
void LeftEncoder()
{
  left_encoder_count = left_encoder_count + left_dirn;
}

void RightEncoder()
{
  right_encoder_count = right_encoder_count + right_dirn;
}



