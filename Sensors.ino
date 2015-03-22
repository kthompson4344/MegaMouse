void setupSensors() {
  // Define IR Pair TXs as Outputs
  for (int i = 0; i < 6; i++) {
    pinMode(TX[i], OUTPUT);
  }
  pinMode(diagHighPower, OUTPUT);
}

int readPair(int pair) {
  // TODO Shorten Times
  int reading;
  digitalWrite(TX[pair], HIGH);
  delayMicroseconds(100);
  reading = analogRead(RX[pair]);
  digitalWrite(TX[pair], LOW);
  delayMicroseconds(100);
  return reading;
}

//void readSensors() {
//  // TODO Calibrate Better
//  leftSensor = 1390.9 * (1 / getLn(readPair(left))) - 211.64;
//  rightSensor = 7169.4 * (1 / getLn(readPair(right))) - 862.45;
//  leftMiddleValue = 792.18 * (1 / getLn(readPair(diagl))) - 110.27 + 15; //15?
//  rightMiddleValue = 391.17 * (1 / getLn(readPair(diagr))) - 63.22 + 15; //15?
//}

void readSensors() {
  digitalWriteFast(TX[lf],HIGH);
  delayMicroseconds(60);
  leftFront = analogRead(RX[lf]);
  digitalWriteFast(TX[lf],LOW);
  delayMicroseconds(80);
  
  digitalWriteFast(TX[rf],HIGH);
  delayMicroseconds(60);
  rightFront = analogRead(RX[rf]);
  digitalWriteFast(TX[rf],LOW);
  delayMicroseconds(80);
  
  digitalWriteFast(TX[left],HIGH);
  digitalWriteFast(TX[right],HIGH);
  delayMicroseconds(60);
  leftSensor = analogRead(RX[left]);
  rightSensor = analogRead(RX[right]);
  digitalWriteFast(TX[left],LOW);
  digitalWriteFast(TX[right],LOW);
  delayMicroseconds(80);
  
  digitalWriteFast(TX[diagl],HIGH);
  digitalWriteFast(23, HIGH);
  delayMicroseconds(60);
  leftMiddleValue = analogRead(RX[diagl]);
  rightMiddleValue = analogRead(RX[diagr]);
  digitalWriteFast(TX[diagl],LOW);
  digitalWriteFast(23, LOW);
}
