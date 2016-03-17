void setupSensors() {
  // Define IR Pair TXs as Outputs
  for (int i = 0; i < 6; i++) {
    pinMode(TX[i], OUTPUT);
    digitalWriteFast(TX[i], LOW);
  }
  pinMode(diagHighPower, OUTPUT);
  digitalWriteFast(diagHighPower, LOW);

  analogReadAveraging(5);
}

// Runs on a timer every 80 microseconds.  Works like a state machine, goes through groups of sensors individually
// and then updates their values.
void readSensors() {
  static int leftFrontAmbient = 0;
  static int rightFrontAmbient = 0;
  static int leftSensorAmbient = 0;
  static int rightSensorAmbient = 0;
  static int leftMiddleAmbient = 0;
  static int rightMiddleAmbient = 0;
  static int state = 0;
  if (!haveSensorReading) {
    switch (state) {
      case 0: // Read ambient values.
        leftFrontAmbient = analogRead(RX[lf]);
        rightFrontAmbient = analogRead(RX[rf]);
        leftSensorAmbient = analogRead(RX[left]);
        rightSensorAmbient = analogRead(RX[right]);
        leftMiddleAmbient = analogRead(RX[diagl]);
        rightMiddleAmbient = analogRead(RX[diagr]);
        break;

      case 1: // Turn on left front sensor.
        digitalWriteFast(TX[lf],HIGH);
        break;

      case 2: // Read left front sensor, then turn it off.
        leftFront = analogRead(RX[lf]) - leftFrontAmbient;
        digitalWriteFast(TX[lf],LOW);
        break;

      case 3: // Turn on right front sensor.
        digitalWriteFast(TX[rf],HIGH);
        break;

      case 4: // Read right front sensor, then turn it off.
        rightFront = analogRead(RX[rf]) - rightFrontAmbient;
        digitalWriteFast(TX[rf],LOW);
        break;

      case 5: // Turn on side sensors.
        digitalWriteFast(TX[left],HIGH);
        digitalWriteFast(TX[right],HIGH);
        break;

      case 6: // Read side sensors, then turn them off.
        leftSensor = analogRead(RX[left]) - leftSensorAmbient;
        rightSensor = analogRead(RX[right]) - rightSensorAmbient;
        digitalWriteFast(TX[left],LOW);
        digitalWriteFast(TX[right],LOW);
        break;

      case 7: // Turn on diagonal sensors.
        digitalWriteFast(TX[diagl],HIGH);
        digitalWriteFast(diagHighPower, HIGH);
        break;

      case 8: // Read diagonal sensors, then turn them off.
        leftMiddleValue = analogRead(RX[diagl]) - leftMiddleAmbient;
        rightMiddleValue = analogRead(RX[diagr]) - rightMiddleAmbient;
        digitalWriteFast(TX[diagl],LOW);
        digitalWriteFast(diagHighPower, LOW);
        break;
    }
    ++state;

    if (state > 8) {
      state = 0;
      haveSensorReading = true;
    }
  }
}


void refreshSensor() {
  haveSensorReading = false;
}

bool wallFront() {
  return ( (leftFront + rightFront)/2 > 12);
}

bool wallLeft() {
  return (leftSensor > hasLeftWall);
}

bool wallRight() {
  return (rightSensor > hasRightWall);
}

void printSensors() {
  Serial.print(leftFront);Serial.print(" ");
  Serial.print(leftMiddleValue);Serial.print(" ");
  Serial.print(leftSensor);Serial.print(" ");
  Serial.print(rightSensor);Serial.print(" ");
  Serial.print(rightMiddleValue);Serial.print(" ");
  Serial.println(rightFront);
}

