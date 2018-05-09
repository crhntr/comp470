#define SERVO_PIN         2  // Any pin on the Arduino or Mega will work.

int lenMicroSecondsOfPeriod = 20 * 1000; // 20 milliseconds (ms)
int lenMicroSecondsOfPulse = 1.0 * 1000; // 1.0 ms is 0 degrees

int Ping( int );
const int pingPin = SCL;

int state = 0;

void setup()
{
  pinMode(SERVO_PIN, OUTPUT);
  int dist_cm = Ping(pingPin);
  Serial.begin(9600);

}

void loop()
{

 // Servos work by sending a 20 ms pulse.
 // 1.0 ms at the start of the pulse will turn the servo to the 0 degree position
 // 1.5 ms at the start of the pulse will turn the servo to the 90 degree position
 // 2.0 ms at the start of the pulse will turn the servo to the 180 degree position
 // Turn voltage high to start the period and pulse

  switch(state){
    case 0: //0 degrees
      turn(lenMicroSecondsOfPulse);
      state = 1;
      delay(1000);
      break;
    case 1:
      turn(lenMicroSecondsOfPulse * 1.5);
      state = 2;
      delay(2000);
      break;
    case 2:
      turn(lenMicroSecondsOfPulse * 2.25);
      state = 3;
      delay(3000);
    case 3:
      turn(lenMicroSecondsOfPulse * 1.5);
      state = 0;
      delay(4000);
      break;
  }

}

void turn(double pulse)
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
