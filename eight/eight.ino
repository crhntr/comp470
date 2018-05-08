#include "motordriver_4wd.h"
#include "seeed_pwm.h"
#include <ChainableLED.h>

ChainableLED led(3, 2, 5);

const double PIE    = 3.14159265;
const double PIE_O2 = PIE/2.0;
const double PIE2 = PIE*2.0;

double theta =  PI/2.0;

#define RW    42.5  // radius wheel
#define D     158.0
#define TPR 72

#define BLOCK_SIZE 300

#define DIRECTION_NORTH 0
#define DIRECTION_EAST  1
#define DIRECTION_SOUTH 2
#define DIRECTION_WEST  3

const int ULTRASONIC_PIN = SCL;
const int BUMP_PIN = 11;
const int SERVO_PIN = 13;

int lenMicroSecondsOfPeriod = 20 * 1000; // 20 milliseconds (ms)
int lenMicroSecondsOfPulse = 1.0 * 1000; // 1.0 ms is 0 degrees

int directionVector[][2] = {
  {1, 0}, {0, 1},
  {-1, 0}, {0, -1}
};

double upperCert = 0.8;
double lowerCert = 0.2;

int directionBot;
int directionUlt;

double positionX = 300 + 150;
double positionY = 300 + 150;

double world[][7] = {
  {1, 1.0, 1.0, 1.0, 1.0, 1.0, 1},
  {1, 0.0, 0.5, 0.5, 0.5, 0.5, 1},
  {1, 0.5, 0.5, 0.5, 0.5, 0.5, 1},
  {1, 0.5, 0.5, 0.5, 0.5, 0.5, 1},
  {1, 0.5, 0.5, 0.5, 0.5, 0.5, 1},
  {1, 0.5, 0.5, 0.5, 0.5, 0.5, 1},
  {1, 1.0, 1.0, 1.0, 1.0, 1.0, 1}
};

int wanderGoalX;
int wanderGoalY;
int wanderStreight;
volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;
int left_dirn;
int right_dirn;

int ping (int pingPin);
void turnLeft();
void turnRight();

void leftEncoder () { left_encoder_count = left_encoder_count + left_dirn; }
void rightEncoder () { right_encoder_count = right_encoder_count + right_dirn; }

int looperCount = 0;

void setup() {
  wanderGoalX = 5;
  wanderGoalY = 5;

  directionBot = DIRECTION_NORTH;
  directionUlt = DIRECTION_NORTH;

  led.init();
  MOTOR.init();
  
  pinMode(BUMP_PIN, INPUT);
  attachInterrupt(0, leftEncoder, CHANGE);
  attachInterrupt(1, rightEncoder, CHANGE);

  MOTOR.setSpeedDir1(10, DIRF);
  MOTOR.setSpeedDir2(10, DIRR);

  left_dirn = 1;
  right_dirn = 1;

  Serial.begin(9600);
  looperCount = 1000;
}

#define CHECKPOINT_INITIAL 0
#define CHECKPOINT_STATE_WANDER 1
#define STATE_DONE 2
int checkpointState = CHECKPOINT_INITIAL;

double dx, dy;

void loop () {
  //---- update robot config (x,y,theta)
  dx = PIE * RW * cos(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  positionX += dx;

  dy = PIE * RW * sin(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  positionY += dy;

  // calculate x, y in map
  int indexX = (int) positionX / BLOCK_SIZE;
  int indexY = (int) positionY / BLOCK_SIZE;

  // calculate next x, y if moving forward in map
  int nextIndexX = indexX + directionVector[directionBot][0];
  int nextIndexY = indexY + directionVector[directionBot][1];

  int bump = digitalRead(BUMP_PIN);

  double dist = ping(ULTRASONIC_PIN);
  int lookingAtX = indexX + directionVector[directionUlt][0];
  int lookingAtY = indexY + directionVector[directionUlt][1];

  if (looperCount > 10) {
    Serial.print(positionX);
    Serial.print(" ");
    Serial.print(positionY);
    Serial.print(" ");
    Serial.print(indexX);
    Serial.print(" ");
    Serial.print(indexY);
    Serial.print(" ");
    Serial.print(lookingAtX);
    Serial.print(" ");
    Serial.print(lookingAtY);
    Serial.print(" ");
    Serial.print(world[nextIndexY][nextIndexX]);
    Serial.print('\n');
   
    looperCount = 0;
  }

  looperCount++;


  switch (checkpointState) {
    case CHECKPOINT_STATE_WANDER:

        if (indexX == wanderGoalX && indexY == wanderGoalY) {
          checkpointState = STATE_DONE;

          stop(); // ACTION <----------
        } else {
          if (bump) {

            world[nextIndexY][nextIndexX] = 1;

            turnLeft(); // TURN LEFT ACTION <----------

          } else if (world[nextIndexY][nextIndexX] > upperCert) { // is probably blocked

            turnLeft(); // TURN LEFT ACTION <----------

          } else if (world[nextIndexY][nextIndexX] < lowerCert) { // is probably clear

//            MOTOR.setSpeedDir1(10, DIRR);
//            MOTOR.setSpeedDir2(10, DIRF);
          }

          world[indexY][indexX] = 0;
        }
        break;
    case STATE_DONE:
      MOTOR.setSpeedDir1(0, DIRF);
      MOTOR.setSpeedDir2(0, DIRF);
      delayMicroseconds(10000);
    break;
  }
}


int ping (int pingPin) {
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

void turnServo(double pulse) {
  for(int i = 0; i < 40; i++) {
     digitalWrite(SERVO_PIN, HIGH);
     // Delay for the length of the pulse
     delayMicroseconds(pulse);

     // Turn the voltage low for the remainder of the pulse
     digitalWrite(SERVO_PIN, LOW);

     delayMicroseconds(lenMicroSecondsOfPeriod - pulse);
   }
}

void stop () {
  MOTOR.setSpeedDir1(0, DIRR);
  MOTOR.setSpeedDir2(0, DIRR);
}


void turnLeft() {
    directionBot = (directionBot + 3) % 4;
    right_encoder_count = left_encoder_count = 0;

    left_dirn = -1;
    right_dirn = 1;

    MOTOR.setSpeedDir1(40, DIRR);
    MOTOR.setSpeedDir2(40, DIRR);

    while (right_encoder_count < 64) {
      delayMicroseconds(1);
    }

    MOTOR.setSpeedDir1(0, DIRF);
    MOTOR.setSpeedDir2(0, DIRR);
}

void turnRight() {
    directionBot = (directionBot + 1) % 4;

    right_encoder_count = left_encoder_count = 0;

    left_dirn = 1;
    right_dirn = -1;

    MOTOR.setSpeedDir1(40, DIRF);
    MOTOR.setSpeedDir2(40, DIRF);
    while (left_encoder_count < 64) {
      delayMicroseconds(1);
    }

    MOTOR.setSpeedDir1(0, DIRF);
    MOTOR.setSpeedDir2(0, DIRR);
}

