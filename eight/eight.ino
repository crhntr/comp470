#include "motordriver_4wd.h"
#include "seeed_pwm.h"
#include <ChainableLED.h>

ChainableLED led(3, 2, 5);

#define BLOCK_SIZE 300

#define DIRECTION_NORTH 0
#define DIRECTION_EAST  1
#define DIRECTION_SOUTH 2
#define DIRECTION_WEST  3

const int ULTRASONIC_PIN = SCL;
const int BUMP_PIN = 11;
const int SERVO_PIN = 13;

int lenMicroSecondsOfPeriod = 20 * 1000;          // 20 milliseconds (ms)

int directionVector[][2] = {
  {1, 0}, {0, 1},
  {-1, 0}, {0, -1}
};

float upperCert = 0.8;
float lowerCert = 0.2;

int directionBot;
int directionUlt;

float positionX = 300 + 150;
float positionY = 300 + 150;

float world[][7] = {
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
int left_encoder_count;
int right_encoder_count;
int left_dirn;
int right_dirn;

int ping (int pingPin);
void turnLeft();
void turnRight();

void leftEncoder () { left_encoder_count = left_encoder_count + left_dirn; }
void rightEncoder () { right_encoder_count = right_encoder_count + right_dirn; }

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
  Serial.begin(9600);
}

#define CHECKPOINT_STATE_WANDER 0
#define STATE_DONE 1
int checkpointState = 0;

void loop () {
  int indexX = (int) positionX / BLOCK_SIZE;
  int indexY = (int) positionY / BLOCK_SIZE;

  int nextIndexX = indexX + directionVector[directionBot][0];
  int nextIndexY = indexY + directionVector[directionBot][1];

  int bump = digitalRead(BUMP_PIN);

  float dist = ping(ULTRASONIC_PIN);
  int lookingAtX = indexX + directionVector[directionUlt][0];
  int lookingAtY = indexY + directionVector[directionUlt][1];
  float lookingAtBlock = world[lookingAtY][lookingAtX];
  if (lookingAtBlock < 0.99 && lookingAtBlock > 0.01) {

  }
  world[lookingAtY][lookingAtX] = 1 - 1/dist;

  switch (checkpointState) {
    case CHECKPOINT_STATE_WANDER:
        if (indexX == wanderGoalX && indexY == wanderGoalY) {
          checkpointState = STATE_DONE;
        } else {
          if (bump) {
            world[nextIndexY][nextIndexX] = 1;
          } else if (world[nextIndexY][nextIndexY] > upperCert) { // is probably blocked
            
          } else if (world[nextIndexY][nextIndexY] < lowerCert) { // is probably clear

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
