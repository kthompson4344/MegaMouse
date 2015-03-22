
void setupMotors() {
  // Define Motor Pins as Outputs
  for (int i = 0; i < 4; i++) {
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
  if (value >= 0) {
    analogWrite(ML1, value);
    digitalWrite(ML2, LOW);
  }

  else if (value == 0) {
    digitalWrite(ML1, LOW);
    digitalWrite(ML2, LOW);
  }
  else {
    analogWrite(ML2, abs(value));
    digitalWrite(ML1, LOW);
  }
}

void setRightPWM(int value) {
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
