#include <motordriver_4wd.h>
#include <seeed_pwm.h>
#include <ChainableLED.h>

//-------- PIE constants

const double PIE    = 3.14159265;
const double PIE_O2 = PIE / 2.0;
const double PIE2 = PIE * 2.0;

//-------- LED crap

#define NUM_LEDS  1
ChainableLED leds(13, 11, NUM_LEDS);

//-------- motor control

void TurnLeft90();
void TurnRight90();
void Straight( int speed, int dirn );

//-------- dead reckoning

// ticks per rotation
#define TPR 72

// robot measurements (mm)
#define RW    42.5  // radius wheel
#define D     158.0

// robot config variables
double x = 100.0, y = 100.0, dx = 0.0, dy = 0.0;
double theta =  PI / 2.0;

// encoder variables
volatile long left_encoder_count = 0, right_encoder_count = 0;
int left_dirn = 1, right_dirn = 1;


//-------- robot state

enum {FWD, REV} state;

//-------- model of environment

double LEFT = 0.0;
double RIGHT = 1500.0;
double BOTTOM = 0.0;
double TOP = 1500.0;


//======================================================================================
// setup
//======================================================================================
void setup()
{
  leds.init();
  MOTOR.init();

  attachInterrupt(0, LeftEncoder, CHANGE);
  attachInterrupt(1, RightEncoder, CHANGE);

  // go straight
  Straight( 10, 1 );
  leds.setColorRGB(0, 0, 100, 0);  // green
  state = FWD;

  //Serial.begin(9600);
}

//======================================================================================
// Loop
//======================================================================================
void loop()
{
  delay(100);

  //---- update robot config (x,y,theta)
  dx = PIE * RW * cos(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  x = x + dx;

  dy = PIE * RW * sin(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  y = y + dy;

  right_encoder_count = left_encoder_count = 0;

  //---- a simple two-state behavior to stay in the box

  if ((state == FWD) && (x >= RIGHT || x <= LEFT || y >= TOP || y <= BOTTOM))
  {
    //---- stop
    Straight( 0, 0 );
    delay(100);

    //---- back up
    leds.setColorRGB(0, 100, 0, 0);  // red
    Straight( 10, -1 );
    delay(500);

    //---- update state
    state = REV;
  }
  else if ((state == REV) && (x < RIGHT && x > LEFT && y < TOP && y > BOTTOM))
  {
    //---- stop
    Straight( 0, 0 );

    //---- turn right 90
    leds.setColorRGB(0, 0, 0, 100);
    TurnRight90();
    //---- update robot config (theta)
    theta  = fmod(theta - PIE_O2, PIE2);
    delay(100);

    //---- go straight
    leds.setColorRGB(0, 0, 100, 0);  // green
    Straight( 10, 1 );

    //---- update state
    state = FWD;
  }

}


//======================================================================================
// TurnLeft90
//======================================================================================
void
TurnLeft90()
{
  right_encoder_count = left_encoder_count = 0;

  left_dirn = -1; right_dirn = 1;
  MOTOR.setSpeedDir1(40, DIRR); MOTOR.setSpeedDir2(40, DIRR);
  while (right_encoder_count < 64)
  {
    delayMicroseconds(1);
  }

  MOTOR.setSpeedDir1(0, DIRF); MOTOR.setSpeedDir2(0, DIRR);
}

//======================================================================================
// TurnRight90
// dirn is 1 for right, -1 for left
//======================================================================================
void
TurnRight90()
{
  right_encoder_count = left_encoder_count = 0;

  left_dirn = 1; right_dirn = -1;
  MOTOR.setSpeedDir1(40, DIRF); MOTOR.setSpeedDir2(40, DIRF);
  while (left_encoder_count < 64)
  {
    delayMicroseconds(1);
  }

  MOTOR.setSpeedDir1(0, DIRF); MOTOR.setSpeedDir2(0, DIRR);
}

//======================================================================================
// Straight
// dirn is 1 for fwd, -1 for bwd
//======================================================================================
void
Straight( int speed, int dirn )
{
  //---- setup encoder variables
  left_dirn = dirn; right_dirn = dirn;

  if (speed == 0)       //-- stop
  {
    MOTOR.setSpeedDir1(0, DIRF); MOTOR.setSpeedDir2(0, DIRR);   return;
  }
  else if (dirn == 1)   //-- fwd
  {
    MOTOR.setSpeedDir1(speed + 0.25 * speed, DIRF); MOTOR.setSpeedDir2(speed, DIRR);
  }
  else                  //-- bwd
  {
    MOTOR.setSpeedDir1(speed + 0.25 * speed, DIRR); MOTOR.setSpeedDir2(speed, DIRF);
  }
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
