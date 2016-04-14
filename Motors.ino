
void setupMotors() {
  // Define Motor Pins as Outputs
  analogWriteResolution(11);
  for (int i = 0; i < 4; i++) {
    analogWriteFrequency(motorPins[i], 23437);
    pinMode(motorPins[i], OUTPUT);
  }
  
  // Define Encoder Pins as Inputs
  for (int i = 0; i < 4; i++) {
    pinMode(encoderPins[i], INPUT);
  }
  
  // Encoder Interrupt Setups
  attachInterrupt(ER1, rightEncoder1, RISING);
  attachInterrupt(ER2, rightEncoder2, RISING);
  attachInterrupt(EL1, leftEncoder1, RISING);
  attachInterrupt(EL2, leftEncoder2, RISING);
}

void setLeftPWM(int value) {
  if (value > maxPWM) {
    value = maxSpeed;
  }
  if (value < -maxPWM) {
    value = -maxSpeed;
  }
  if (value >= 0) {
    digitalWrite(ML1, LOW);
    analogWrite(ML2, value);
  }

  else if (value == 0) {
    digitalWrite(ML1, LOW);
    digitalWrite(ML2, LOW);
  }
  else {
    digitalWrite(ML2, LOW);
    analogWrite(ML1, abs(value));
  }
}

void setRightPWM(int value) {
  if (value > maxPWM) {
    value = maxSpeed;
  }
  if (value < -maxPWM) {
    value = -maxSpeed;
  }
  if (value >= 0) {
    analogWrite(MR1, value);
    digitalWrite(MR2, LOW);
  }
  else if (value == 0) {
    digitalWrite(MR1, LOW);
    digitalWrite(MR2, LOW);
  }
  else {
    analogWrite(MR2, abs(value));
    digitalWrite(MR1, LOW);
  }
}

void stopMotors() {
  digitalWrite(MR1, HIGH);
  digitalWrite(MR2, HIGH);
  digitalWrite(ML1, HIGH);
  digitalWrite(ML2, HIGH);
}
// Right Encoder ISR
void rightEncoder1() {
  if (digitalReadFast(ER2) == LOW) {
    rightTicks++;
  }
  else {
    rightTicks--;
  }
}


void rightEncoder2() {
  if (digitalReadFast(ER1) == HIGH) {
    rightTicks++;
  }
  else {
    rightTicks--;
  }
}


// Left Encoder ISR
void leftEncoder1() {
  if (digitalReadFast(EL2) == HIGH) {
    leftTicks++;
  }
  else {
    leftTicks--;
  }
}

void leftEncoder2() {
  if (digitalReadFast(EL1) == LOW) {
    leftTicks++;
  }
  else {
    leftTicks--;
  }
}
