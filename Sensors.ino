void setupSensors() {
  // Define IR Pair TXs as Outputs
  for (int i = 0; i < 6; i++) {
    pinMode(TX[i], OUTPUT);
    digitalWriteFast(TX[i], LOW);
  }
  pinMode(diagHighPower, OUTPUT);
  digitalWriteFast(diagHighPower, LOW);
}

//void readSensors() {
//  // TODO Calibrate Better
//  leftSensor = 1390.9 * (1 / getLn(readPair(left))) - 211.64;
//  rightSensor = 7169.4 * (1 / getLn(readPair(right))) - 862.45;
//  leftMiddleValue = 792.18 * (1 / getLn(readPair(diagl))) - 110.27 + 15; //15?
//  rightMiddleValue = 391.17 * (1 / getLn(readPair(diagr))) - 63.22 + 15; //15?
//}

//void readSensors() {
//  digitalWriteFast(TX[lf],HIGH);
//  delayMicroseconds(60);
//  leftFront = analogRead(RX[lf]);
//  digitalWriteFast(TX[lf],LOW);
//  delayMicroseconds(80);
//  
//  digitalWriteFast(TX[rf],HIGH);
//  delayMicroseconds(60);
//  rightFront = analogRead(RX[rf]);
//  digitalWriteFast(TX[rf],LOW);
//  delayMicroseconds(80);
//  
//  digitalWriteFast(TX[left],HIGH);
//  digitalWriteFast(TX[right],HIGH);
//  delayMicroseconds(60);
//  leftSensor = analogRead(RX[left]);
//  rightSensor = analogRead(RX[right]);
//  digitalWriteFast(TX[left],LOW);
//  digitalWriteFast(TX[right],LOW);
//  delayMicroseconds(80);
//  
//  digitalWriteFast(TX[diagl],HIGH);
//  digitalWriteFast(23, HIGH);
//  delayMicroseconds(60);
//  leftMiddleValue = analogRead(RX[diagl]);
//  rightMiddleValue = analogRead(RX[diagr]);
//  digitalWriteFast(TX[diagl],LOW);
//  digitalWriteFast(23, LOW);
//}

void readSensors() {
  int leftFrontAmbient;
  int rightFrontAmbient;
  int leftSensorAmbient;
  int rightSensorAmbient;
  int leftMiddleAmbient;
  int rightMiddleAmbient;
  static int state = 0;
  if (haveSensorReading == 0) {
    switch (state) {
    case 0 :
      leftFrontAmbient = analogRead(RX[lf]);
      rightFrontAmbient = analogRead(RX[rf]);
      leftSensorAmbient = analogRead(RX[left]);
      rightSensorAmbient = analogRead(RX[right]);
      leftMiddleAmbient = analogRead(RX[diagl]);
      rightMiddleAmbient = analogRead(RX[diagr]);
      break;
      
    case 1 :
      digitalWriteFast(TX[lf],HIGH);
      break;
      
    case 2 :
      leftFront = analogRead(RX[lf]) - leftFrontAmbient;
      digitalWriteFast(TX[lf],LOW);
      break;
      
    case 3 :
      digitalWriteFast(TX[rf],HIGH);
      break;
      
    case 4 :
      rightFront = analogRead(RX[rf]) - rightFrontAmbient;
      digitalWriteFast(TX[rf],LOW);
      break;
      
    case 5 :
      digitalWriteFast(TX[left],HIGH);
      digitalWriteFast(TX[right],HIGH);
      break;
      
    case 6 :
      leftSensor = analogRead(RX[left]) - leftSensorAmbient;
      rightSensor = analogRead(RX[right]) - rightSensorAmbient;
      digitalWriteFast(TX[left],LOW);
      digitalWriteFast(TX[right],LOW);
      break;
      
    case 7 :
      digitalWriteFast(TX[diagl],HIGH);
      digitalWriteFast(diagHighPower, HIGH);
      break;
      
    case 8 :
      leftMiddleValue = analogRead(RX[diagl]) - leftMiddleAmbient;
      rightMiddleValue = analogRead(RX[diagr]) - rightMiddleAmbient;
      digitalWriteFast(TX[diagl],LOW);
      digitalWriteFast(diagHighPower, LOW);
      break;
  }
  state++;
  
  if (state > 8) {
    state = 0;
    haveSensorReading = 1;
  }
  }
}

void displaySensors() {
//   mySerial.print(leftFront);
//   mySerial.print(" ");
//   mySerial.print(leftMiddleValue);
//   mySerial.print(" ");
//   mySerial.print(leftSensor);
//   mySerial.print(" ");
//   mySerial.print(rightSensor);
//   mySerial.print(" ");
   mySerial.println(rightMiddleValue);
//   mySerial.print(" ");
//   mySerial.println(rightFront);
 }
 
 boolean wallFront() {
  return (leftFront > 250 && rightFront > 250);
}

boolean wallLeft() {
  //  readSensors();
  return (leftSensor > hasLeftWall);
}

boolean wallRight() {
  //  readSensors();
  return (rightSensor > hasRightWall);
}
