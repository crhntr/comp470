
volatile long left_encoder_count = 0, right_encoder_count = 0;

void setup () {
  attachInterrupt(0, LeftEncoder, CHANGE);
  attachInterrupt(1, RightEncoder, CHANGE);
  Serial.begin(9600);
}

void loop () {
  Serial.println(left_encoder_count);
  Serial.println(right_encoder_count);
  delay(1000);
}

//======================================================================================
// Interrupt Service Routines for encoders
//=====================================================================================
void LeftEncoder () {
  left_encoder_count = left_encoder_count + 1;
}

void RightEncoder () {
  right_encoder_count = right_encoder_count + 1;
}
