#include "motordriver_4wd.h"
#include "seeed_pwm.h"
#include <ChainableLED.h>

ChainableLED led(3, 2, 5);

double theta =  PI/2.0;

#define RW    42.5  // radius wheel
#define D     158.0
#define TPR 72

#define BLOCK_SIZE 160

#define DIRECTION_NORTH 0
#define DIRECTION_EAST  1
#define DIRECTION_SOUTH 2
#define DIRECTION_WEST  3

enum {LEFT, RIGHT} photo_state;

const int ULTRASONIC_PIN = SCL;
const int BUMP_PIN = 11;
const int SERVO_PIN = 13;
const int PHOTO_PIN = 0;

int lenMicroSecondsOfPeriod = 20 * 1000; // 20 milliseconds (ms)
int lenMicroSecondsOfPulse = 1.0 * 1000; // 1.0 ms is 0 degrees

int directionVector[][2] = {
  { 0, 1}, {-1,0}, { 0,-1}, { 1, 0}
};

int state = 0;

double probablyBlockedThreashold = 0.8;
double probablyClearThreashold = 0.2;

int directionBot;
int directionUlt;

double positionX = BLOCK_SIZE * 1.5;
double positionY = BLOCK_SIZE * 1.5;

int world[][7] = {
  {1, 1, 1, 1, 1, 1, 1},
  {1, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 1},
  {1, 1, 1, 1, 1, 1, 1}
};

int wanderGoalX;
int wanderGoalY;
int wanderStreight;
int left_dirn = 1;
int right_dirn = 1;
volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;
int upRamp = 0;

int ping (int pingPin);
// void turnLeft();
// void turnRight();
void RampTime();

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

  Serial.begin(9600);
  looperCount = 1000;

  MOTOR.setSpeedDir1(10, DIRF);
  MOTOR.setSpeedDir2(10, DIRR);
}

#define CHECKPOINT_INITIAL 0
#define CHECKPOINT_STATE_WANDER 1
#define REACHED_GOAL 2
int checkpointState = CHECKPOINT_STATE_WANDER;

int preferRightTurn = 0;

double dx, dy;

void loop () {
  delay(50);

  switch (directionBot) {
    case DIRECTION_NORTH: theta = PI/2 ; break;
    case DIRECTION_EAST:  theta = PI; break;
    case DIRECTION_SOUTH: theta = PI/2+PI ; break;
    case DIRECTION_WEST:  theta = 0 ; break;
  }

  //---- update robot config (x,y,theta)
  dx = PI * RW * cos(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  positionX += dx;

  dy = PI * RW * sin(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  positionY += dy;

  left_encoder_count = 0;
  right_encoder_count = 0;

  // calculate x, y in map
  int indexX = (int) (positionX / BLOCK_SIZE);
  int indexY = (int) (positionY / BLOCK_SIZE);

  // calculate next x, y if moving forward in map
  int nextIndexX = indexX + directionVector[directionBot][0];
  int nextIndexY = indexY + directionVector[directionBot][1];

  int bump = digitalRead(BUMP_PIN);

  double dist = ping(ULTRASONIC_PIN);
  int lookingAtX = indexX + directionVector[directionUlt][0];
  int lookingAtY = indexY + directionVector[directionUlt][1];

  if (looperCount > 10) {

    Serial.print("facing: ");
    switch (directionBot) {
      case DIRECTION_NORTH: Serial.println("DIRECTION_NORTH"); break;
      case DIRECTION_EAST:  Serial.println("DIRECTION_EAST");  break;
      case DIRECTION_SOUTH: Serial.println("DIRECTION_SOUTH"); break;
      case DIRECTION_WEST:  Serial.println("DIRECTION_WEST");  break;
    }

    displayMap(indexX, indexY, nextIndexX, nextIndexY);
    Serial.print(positionX);
    Serial.print(" ");
    Serial.print(positionY);
    Serial.print(" ");
    Serial.print(indexX);
    Serial.print(" ");
    Serial.print(indexY);
    Serial.print(" ");
    Serial.print(nextIndexX);
    Serial.print(" ");
    Serial.print(nextIndexY);
    Serial.print(" ");
    Serial.print(world[nextIndexY][nextIndexX]);
    Serial.print('\n');

    looperCount = 0;
  }

  looperCount++;


  switch (checkpointState) {
    case CHECKPOINT_STATE_WANDER:
        if (indexX == wanderGoalX && indexY == wanderGoalY) {
          Serial.println("REACHED_GOAL");
          stop();
          delay(3000);

          MOTOR.setSpeedDir1(30, DIRR);
          MOTOR.setSpeedDir2(30, DIRR);
          delay(555*2);

          wanderGoalX = 1;
          wanderGoalY = 1;
          preferRightTurn = 1;

          checkpointState = CHECKPOINT_STATE_WANDER;
        } else {
          if (bump || world[nextIndexX][nextIndexY] == 1) {
            stop();

            Serial.println("I HIT A THING... SORRY!");
            delay(2000);

            switch (directionBot) {
              case DIRECTION_NORTH:
                if (!world[indexX][indexY+1]) {
                  directionBot = DIRECTION_EAST;
                  MOTOR.setSpeedDir1(30, DIRR);
                  MOTOR.setSpeedDir2(30, DIRR);
                } else {
                  directionBot = DIRECTION_WEST;
                  MOTOR.setSpeedDir1(30, DIRF);
                  MOTOR.setSpeedDir2(30, DIRF);}
              break;
              case DIRECTION_EAST:
                if (!world[indexX-1][indexY]) {
                  directionBot = DIRECTION_NORTH;
                  MOTOR.setSpeedDir1(30, DIRF);
                  MOTOR.setSpeedDir2(30, DIRF);
                } else {
                  directionBot = DIRECTION_SOUTH;
                  MOTOR.setSpeedDir1(30, DIRR);
                  MOTOR.setSpeedDir2(30, DIRR);
                }
              break;
              case DIRECTION_SOUTH:
                if (!world[indexX][indexY-1]) {
                  directionBot = DIRECTION_EAST;
                  MOTOR.setSpeedDir1(30, DIRF);
                  MOTOR.setSpeedDir2(30, DIRF);
                } else {
                  directionBot = DIRECTION_WEST;
                  MOTOR.setSpeedDir1(30, DIRR);
                  MOTOR.setSpeedDir2(30, DIRR);
                }
              break;
              case DIRECTION_WEST:
                if (!world[indexX+1][indexY]) {
                  directionBot = DIRECTION_NORTH;
                  MOTOR.setSpeedDir1(30, DIRF);
                  MOTOR.setSpeedDir2(30, DIRF);
                } else {
                  directionBot = DIRECTION_SOUTH;
                  MOTOR.setSpeedDir1(30, DIRR);
                  MOTOR.setSpeedDir2(30, DIRR);
                }
              break;
            }
            right_encoder_count = left_encoder_count = 0;
            delay(555);
            displayMap(indexX, indexY, nextIndexX, nextIndexX);
            MOTOR.setSpeedDir1(10, DIRF);
            MOTOR.setSpeedDir2(10, DIRR);
        }
        break;}
  }
}

void displayMap(int posX, int posY, int lposX, int lposY) {
  for (int i = 0; i < 7; i++) {
    for (int j = 0; j < 7; j++) {
      if (i == lposX && j == lposY) {
        Serial.print('*');
      } else if (i == posX && j == posY) {
        Serial.print('@');
      } else if (world[i][j]) {
        Serial.print('#');
      } else {
        Serial.print(' ');
      }
    }
    Serial.print('\n');
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
  Serial.println("DON'T STOP BELIEVING");
  MOTOR.setStop1();
  MOTOR.setStop2();
}

void fwd () {
  left_dirn = 1;
  right_dirn = 1;
  MOTOR.setSpeedDir1(10, DIRF);
  MOTOR.setSpeedDir2(10, DIRR);
}

void bck () {
  stop();

  left_dirn = -1;
  right_dirn = -1;
  MOTOR.setSpeedDir1(10, DIRR);
  MOTOR.setSpeedDir2(10, DIRF);
}

// void turnLeft() {
//
//     directionBot = leftOf(directionBot);
//     right_encoder_count = left_encoder_count = 0;
//
//     left_dirn = -1; right_dirn = 1;
//
//     MOTOR.setSpeedDir1(40, DIRR);
//     MOTOR.setSpeedDir2(40, DIRR);
//
//     while (right_encoder_count < 64 || right_encoder_count > 64) {
//       delayMicroseconds(1);
//     }
//
// }
//
// void turnRight() {
//
//     directionBot = rightOf(directionBot);
//     right_encoder_count = left_encoder_count = 0;
//
//     left_dirn = 1; right_dirn = -1;
//
//     MOTOR.setSpeedDir1(40, DIRF);
//     MOTOR.setSpeedDir2(40, DIRF);
//     while (left_encoder_count < 64 || left_encoder_count > 64) {
//       delayMicroseconds(1);
//     }
//
// }

void RampTime()
{
  int val = analogRead(A0);

  Serial.println(val);
  while(val > 900 && !upRamp)
  {
    val = analogRead(A0);
    Serial.println(val);
    if(val < 80)
        upRamp = 1;
    else
      MOTOR.setSpeedDir1(15, DIRR);
      MOTOR.setSpeedDir2(15, DIRR);

  }

  if (val > 60)
  {
    if (state == LEFT)
    {
      MOTOR.setSpeedDir1(18, DIRF);
      MOTOR.setSpeedDir2(12, DIRR);
      state = RIGHT;
      delay(500);
    }
    else
    if (state == RIGHT)
    {
      MOTOR.setSpeedDir1(12, DIRF);
      MOTOR.setSpeedDir2(18, DIRR);
      state = LEFT;
      delay(500);
    }
  }
}
