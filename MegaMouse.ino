// Teensy 3 I2C Library
#include <i2c_t3.h>
#include "Gyro.h"
#include "Motors.h"
#include "Sensors.h"
#include "Profiles.h"
#include <SoftwareSerial.h>

#define hasLeftWall 800
#define hasRightWall 800
int leftBaseSpeed = 0;
int rightBaseSpeed = 0;
// PID Constants
#define straightKp 16
#define turnKp 16
#define Kd 10

const int rxPin = 0;
const int txPin = 16;


int ticksR[1000];
int ticksL[1000];
int leftM[1000];
int rightM[1000];
int leftS[1000];
int rightS[1000];
int readings = 0;

volatile bool firstCell = true;
volatile bool rightValid = true;
volatile bool leftValid = true;
bool go = false;
volatile bool needMove = true;
volatile bool haveSensorReading = true;

IntervalTimer correctionTimer;
IntervalTimer sensorTimer;
SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);
volatile float angle = 0.0;

volatile enum {
  NO = 0,
  FORWARD = 1,
  TURN_LEFT = 2,
  TURN_RIGHT = 3,
  TURN_AROUND = 4
} moveType;

const int buttonPin = 24;
const int LED = 11;

void setup() {

  float degreesTraveled;
  float initialZ;
//  Serial.begin(9600);
  mySerial.begin(115200);

  // 12 bit ADC resolution
  analogReadResolution(12);

  setupMotors();
  setupSensors();

  correctionTimer.begin(correction, 1000);
  correctionTimer.priority(255);
  sensorTimer.begin(readSensors, 80);
  sensorTimer.priority(250);

  pinMode(LED, OUTPUT);

  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(intPin, INPUT);

  // Wait for Button Press to Start
  // readSensors();
  //while (digitalRead(buttonPin) == 1) {
  //}
  //while (rightFront < 3300 && rightMiddleValue < 3300 && rightSensor < 3300) {
  //    readSensors();
  //    Serial.println(leftSensor);
  //  }
  delay(3000);
  setupGyro();
  //  delay(2000);
  //  getGres();
  //  initialZ = (float)readGyroData()*gRes - gyroBias[2];
  //  count = millis();
  delay(1000);
  //  while(1) {
  //      setLeftPWM(30);
  //      setRightPWM(30);
  //    }
  //turnRight();
  //  go = 1;
  //  turnRight();
  //turnLeft();

  //moveType = NO;
  //  while (1) {
  //    digitalWriteFast(LED2,HIGH);
//  moveForward();
//  while (needMove == 0);
//  turnLeft();
//  turnRight();
//  while (needMove == 0);
//  turnRight();
//  while (needMove == 0);
  //    digitalWriteFast(LED1,LOW);
  //    digitalWriteFast(LED2,LOW);
  //    turnAround();
  //    while (needMove == 0);
//  setLeftPWM(0);
//  setRightPWM(0);
//  pivotTurnRight();
  //while(1);
  //pivotTurnRight();
//go = 1;
//    moveForward();
//    while (needMove == 0);
//    moveForward();
//    while (needMove == 0);
//    moveForward();
//    while (needMove == 0);
//  ////    digitalWrite(LED2,LOW);
//      turnRight();
//  ////    turnLeft();
//      while (needMove == 0);
//  //    turnRight();
//      turnLeft();
//      while (needMove == 0);
//      turnRight();
//      while (needMove == 0);
//      moveForward();
//      while (needMove == 0);
  //    moveForward();
  //    while (needMove == 0);
  //    moveForward();
  //    while (needMove == 0);
  //    digitalWrite(LED2,LOW);
  //    turnRight();
  //    while (needMove == 0);
  //    turnRight();
  //    while (needMove == 0);
  //    moveForward();
  //    while (needMove == 0);
  //    moveForward();
  //    while (needMove == 0);
  //    moveForward();
  //    while (needMove == 0);
  //    moveForward();
  //    while (needMove == 0);
  //    digitalWrite(LED2,LOW);
  //    turnRight();
  //    while (needMove == 0);
  //    while(1) {
//    go = 0;
//    sensorTimer.end();
//    correctionTimer.end();
//      setLeftPWM(0);
//      setRightPWM(0);
//      for(int i = 0; i < readings; i++) {
//        mySerial.print(ticksL[i]);
//        mySerial.print(",");
//        mySerial.print(ticksR[i]);
//        mySerial.print(",");
//        mySerial.print(rightM[i]);
//        mySerial.print(",");
//        mySerial.print(leftM[i]);
//        mySerial.print(",");
//        mySerial.print(rightS[i]);
//        mySerial.print(",");
//        mySerial.println(leftS[i]);
//      }
  //    }
  //    moveForward();
  //    while (needMove == 0);
  //    moveForward();
  //    while (needMove == 0);
  //    turnRight();
  //    while (needMove == 0);
  //    moveForward();
  //    while (needMove == 0);
  //    moveForward();
  //    while (needMove == 0);
  //    moveForward();
  //    while (needMove == 0);
  //    moveForward();
  //    while (needMove == 0);
  //////    digitalWrite(LED2,LOW);
  //    turnRight();
  //    while (needMove == 0);
  //    while(1) {
  //    setLeftPWM(0);
  //    setRightPWM(0);
  //    Serial.print((rightTicks+leftTicks)/2);
  //    Serial.print(" ");
  //    Serial.print(leftSensor);
  //    Serial.print(" ");
  //    Serial.println(rightSensor);
  //    delay(100);
  //    }
  //    moveForward();
  //    while (needMove == 0);
  //    moveForward();
  //    while (needMove == 0);
  //    turnRight();
  //    while (needMove == 0);
  //  }
}

void loop() {


  //Serial.println(rightTicks);
    wallFollow();

  //  //displaySensors();

  //  setLeftPWM(0);
  //  setRightPWM(0);
  //  Serial.print("Wall Left = ");
  //  Serial.print(wallLeft());
  //  Serial.print(" ");
  //  Serial.print("Wall Right = ");
  //  Serial.print(wallRight());
  //  Serial.print(" ");
  //  Serial.print("Wall Front = ");
  //  Serial.println(wallFront());
//    displaySensors();
//        delay(1);
}

// Mack calls certain number of move forwards; we add however many ticks for every move forward.

void correction() {
//  displaySensors();
//  readSensors();
  switch (moveType) {
    case FORWARD:
      forwardCorrection();
      break;
    case NO:
      break;
    case TURN_RIGHT:
      turnCorrection();
      break;
    case TURN_LEFT:
      turnCorrection();
      break;
  }
  if(go == 1) {
  ticksR[readings] = rightTicks;
  ticksL[readings] = leftTicks;
  rightM[readings] = rightMiddleValue;
  leftM[readings] = leftMiddleValue;
  rightS[readings] = rightSensor;
  leftS[readings] = leftSensor;
  if(readings<999) {
  readings++;
  }
  }
  haveSensorReading = 0;
}

void moveForward() {
//  digitalWriteFast(LED2, HIGH);

//          digitalWriteFast(LED1, HIGH);
  if (firstCell == 1) {
    rightTicks = 70;
    leftTicks = 70;
    firstCell = 0;
  }
  else {
  rightTicks = 0;
  leftTicks = 0;
  }
  leftBaseSpeed = 240;//240
  rightBaseSpeed = 240;//240

  if (wallRight()) {
    rightValid = true;
  }
  else {
    rightValid = false;
  }
  if (wallLeft()) {
    leftValid = true;
  }
  else {
    leftValid = false;
  }
  moveType = FORWARD;
  needMove = false;
}

void turnRight() {
//  digitalWriteFast(LED2, HIGH);
//  digitalWriteFast(LED1, LOW);
  leftBaseSpeed = 240;
  rightBaseSpeed = 240;
  moveType = TURN_RIGHT;
  needMove = false;
}

void turnLeft() {
//  digitalWriteFast(LED2, LOW);
//  digitalWriteFast(LED1, HIGH);
  leftBaseSpeed = 240;
  rightBaseSpeed = 240;
  moveType = TURN_LEFT;
  needMove = false;
}

void turnAround() {
  const int frontLeftStop = 2000;
  const int frontRightStop = 2300;
  bool leftStop = false;
  bool rightStop = false;
  int tickCount = 180;
  int errorP;
  int errorD;
  int totalError;
  leftBaseSpeed = 240;
  rightBaseSpeed = 240;
  moveType = NO;

  while (!leftStop || !rightStop) {
    if (leftFront <= frontLeftStop) {
      setLeftPWM(leftBaseSpeed);
    }
    else {
      setLeftPWM(0);
      leftStop = 1;
    }
    if (rightFront <= frontRightStop) {
      setRightPWM(rightBaseSpeed);
    }
    else {
      setRightPWM(0);
      rightStop = 1;
    }
  }
  delay(100);
  leftStop = false;
  rightStop = false;

  pivotTurnRight();

//  while (leftStop == 0 || rightStop == 0) {
//
//    if (leftFront <= frontLeftStop - 500) {
//      setLeftPWM(leftBaseSpeed);
//    }
//    else {
//      setLeftPWM(0);
//      leftStop = 1;
//    }
//    if (rightFront <= frontRightStop - 500) {
//      setRightPWM(rightBaseSpeed);
//    }
//    else {
//      setRightPWM(0);
//      rightStop = 1;
//    }
//  }
//
//  delay(100);
//  pivotTurnRight();

  // read next walls here

  leftTicks = 0;
  rightTicks = 0;
  angle = 0;
  while ((rightTicks + leftTicks) / 2 <= tickCount) {
    uint32_t deltat = millis() - count;
    if (deltat > 1) {
      getGres();
      gz = (float)readGyroData() * gRes - gyroBias[2];
      angle += 2 * (gz - 0) * 0.001;
      count = millis();
      
    }
    errorP = 100 * (angle);
      // Calculate PWM based on Error
      currentLeftPWM = leftBaseSpeed + totalError / 124;
      currentRightPWM = rightBaseSpeed - totalError / 124;
    
      // Update Motor PWM values
      setLeftPWM(currentLeftPWM);
      setRightPWM(currentRightPWM);
 
  }
  angle = 0;
  needMove = true;
  leftTicks = 0;
  rightTicks = 0;
}

void forwardCorrection() {
  const int oneCellTicks = 327;
  const int noWallRight = 250; // check this value (250)
  const int noWallLeft = 450; // check this value (450)

  const int pegWallBack = 800; // check this value
  const int pegNoWalls = 1000;
  const int pegWallFront = 1000;

  const int wallBackTicks = 240;
  const int noWallTicks = 218;
  const int frontWallTicks = 215;

  // no walls right and left reach 1000 at 213
  // no wall front, wall back, drops below 800 at 222
  // wall front, no walls back, goes above 1000 at 196
  // encoder tick value when we check walls a cell ahead
  const int readingTicks = 173; // check this value (163)
  // encoder tick value when we switch to next cell's values
  const int newSideTicks = 200; // check this value (200)

  static bool nextRightValid;
  static bool nextLeftValid;
  static bool nextCellDecided = false;
  //  int righTicksRemaining;
  //  int leftTicksRemaining;
  int errorP;
  int errorD;
  int oldErrorP;
  int totalError;
  //  static float angle;
  static int lastTicksL;
  static int lastTicksR;
  static int targetAngle = 0;
  static float straightAngle = 0.0;
  static bool endCell = false;
  static bool currentWallLeft = true;
  static bool currentWallRight = true;
  static bool ticksDecided = false;
//  getAres();
//  Serial.println("3");
//  digitalWrite(LED2,HIGH);
//  az = (float)readAccelData()*aRes - accelBias[2];
//  //  delay(500);
//  Serial.println(az);
//  if (az > 2) {
//    setLeftPWM(0);
//    setRightPWM(0);
//    leftBaseSpeed = 0;
//    rightBaseSpeed = 0;
//    moveType = NO;
//  }

  // Next Cell Wall Detection
  if ((rightTicks + leftTicks) / 2 >= readingTicks && !nextCellDecided) {
    nextRightValid = rightMiddleValue > noWallRight;
    nextLeftValid = leftMiddleValue > noWallLeft;
    nextCellDecided = true;
    if (leftValid != nextLeftValid) {
      leftValid = false;
    }
    if (rightValid != nextRightValid) {
      rightValid = false;
    }
  }

  if ((rightTicks + leftTicks) / 2 >= newSideTicks) {
    leftValid = nextLeftValid;
    rightValid = nextRightValid;
    // nextCellDecided = 0;
  }


  if (leftValid && rightValid) {
//    digitalWriteFast(LED2, HIGH);
//    digitalWriteFast(LED1, HIGH);
//    digitalWriteFast(LED1, HIGH);
//    digitalWriteFast(LED2, HIGH);
//    Serial.println("Has Both");
    angle = 0;
    targetAngle = 0;
    // Has both wall, so error correct with both (working, just need to adjust PD constants when final mouse is built)
    errorP = leftSensor - rightSensor - 100; // 100 is the offset between left and right sensor when mouse in the
                                             // middle of cell
    errorD = errorP - oldErrorP;
//    getGres();
//    gz = (float)readGyroData() * gRes - gyroBias[2];
//    straightAngle += 2 * (gz) * 0.001;
  }
  else if (leftValid) {
    const int wallDist = 1900; // Make this bigger to move closer to the wall
//    digitalWriteFast(LED1, HIGH);
//    digitalWriteFast(LED2, LOW);
//    digitalWriteFast(LED1, HIGH);
//    digitalWriteFast(LED2, LOW);
//    Serial.println("Has Left");
    // Only left wall, insert one wall correction here
    // errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100 * (angle - targetAngle);
    getGres();
    gz = (float)readGyroData() * gRes - gyroBias[2];
    angle += 2 * (gz) * 0.001;
    errorP = 20 * (angle - targetAngle) + .5 * (leftSensor - wallDist);
    errorD = errorP - oldErrorP;

  }
  else if (rightValid) {
    const int wallDist = 2000; //Make this bigger to move closer to the wall
//    digitalWriteFast(LED1, LOW);
//    digitalWriteFast(LED2, HIGH);
//    digitalWriteFast(LED1, LOW);
//    digitalWriteFast(LED2, HIGH);
//    Serial.println("Has Right");
    // Only right wall, insert one wall correction here (TODO?)
    getGres();
    gz = (float)readGyroData() * gRes - gyroBias[2];
    angle += 2 * (gz) * 0.001;
    errorP = 20 * (angle - targetAngle) - .5 * (rightSensor - wallDist);
    errorD = errorP - oldErrorP;
  }
  else {
//    digitalWriteFast(LED1, LOW);
//    digitalWriteFast(LED2, LOW);
//    Serial.println("Has None");
//    if (changeR < 0 || changeL < 0) {
//      errorP = 0;
//      errorD = 0;
//    }
//    else {
//      errorP = 1000*(changeR - changeL);
//    }
    // No walls, use encoders to correct
    getGres();
    gz = (float)readGyroData() * gRes - gyroBias[2];
    angle += 2 * (gz) * 0.001;
    errorP = 20 * (angle - targetAngle);
    errorD = errorP - oldErrorP;
  }

  // no walls right and left reach 1000 at 213
  // no wall front, wall back, drops below 800 at 222
  // wall front, no walls back, goes above 1000 at 196

#if 0
  // No walls in next cell or current cell
  if (!currentWallRight && !nextRightValid && !ticksDecided) {
    if (rightSensor >= pegNoWalls) {
      leftTicks = noWallTicks;
      rightTicks = noWallTicks;
      ticksDecided = true;
    }
  }

  // No walls in next cell, walls in current cell
  else if (!currentWallRight && !nextRightValid && !ticksDecided) {
    if (rightSensor <= pegWallBack) {
      leftTicks = wallBackTicks;
      rightTicks = wallBackTicks;
      ticksDecided = true;
    }
  }

  // No walls in current cell, wall in next cell
  else if (!currentWallRight && nextRightValid && !ticksDecided) {
    if (rightSensor >= pegWallFront) {
        leftTicks = frontWallTicks;
        rightTicks = frontWallTicks;
        ticksDecided = true;
    }
  }

  // No walls in next cell or current cell
  if (!currentWallLeft !nextLeftValid && !ticksDecided) {
    if (leftSensor >= pegNoWalls) {
      leftTicks = noWallTicks;
      rightTicks = noWallTicks;
      ticksDecided = true;
    }
  }

  // No walls in next cell, walls in current cell
  else if (currentWallLeft && !nextLeftValid && !ticksDecided) {
    if (leftSensor <= pegWallBack) {
     leftTicks = wallBackTicks;
      rightTicks = wallBackTicks;
      ticksDecided = true;
    }
  }

  // No walls in current cell, wall in next cell
  else if (!currentWallLeft && nextLeftValid && !ticksDecided) {
    if (leftSensor >= pegWallFront) {
        leftTicks = frontWallTicks;
        rightTicks = frontWallTicks;
        ticksDecided = true;
    }
  }
#endif
    
  if ((rightTicks + leftTicks) / 2 >= oneCellTicks) {
    endCell = true;
  }
  if (endCell) {
    currentWallLeft = nextLeftValid;
    currentWallRight = nextRightValid;
    oldErrorP = 0;
    ticksDecided = 0;
    needMove = 1;
    // rightTicks = 0;
    // leftTicks = 0;
    nextCellDecided = 0;
    moveType = NO;
    endCell = false;
  }


  totalError = straightKp * errorP + Kd * errorD;
  oldErrorP = errorP;

  // Calculate PWM based on Error
  currentLeftPWM = leftBaseSpeed + totalError / 124;
  currentRightPWM = rightBaseSpeed - totalError / 124;

  // Serial.print(currentLeftPWM);
  // Serial.print(" ");
  // Serial.println(currentRightPWM);

  // Update Motor PWM values
  setLeftPWM(currentLeftPWM);
  setRightPWM(currentRightPWM);
}

void turnCorrection() {
  int errorP;
  int errorD;
  int oldErrorP;
  int totalError;
  static float targetAngle;
  static int i = 0;

  static bool straight = 0;
  const int targetTicks = 140;
  //  const int targetTicks = 120;//curve3


  if (!straight) {
//    digitalWriteFast(LED1, HIGH);
//    digitalWriteFast(LED2, HIGH);
    targetAngle = curve2[i];
    // targetAngle = curve3[i];
    getGres();

    gz = (int)readGyroData() * gRes - gyroBias[2];

    if (moveType == TURN_RIGHT) {
      angle += 1.37 * (gz) * 0.001; // comment out for curve3
      // angle += 1 * (gz) * .001;
      
      errorP = 100 * (angle + targetAngle); // comment out for curve3
    }
    else {
      angle += 1.375 * (gz) * 0.001; // comment out for curve3
      // angle += 1 * (gz) * .001;
      errorP = 100 * (angle - targetAngle); // comment out for curve3
    }

    // errorP = 20 * (.85*gz + targetAngle); //curve3
    errorD = errorP - oldErrorP;

    rightTicks = 0;
    leftTicks = 0;
    totalError = turnKp * errorP + Kd * errorD;
    oldErrorP = errorP;

    // Calculate PWM based on Error
    currentLeftPWM = leftBaseSpeed + totalError / 124;
    currentRightPWM = rightBaseSpeed - totalError / 124;

    //  Serial.print(currentLeftPWM);
    //  Serial.print(" ");
    //  Serial.println(currentRightPWM);

    // Update Motor PWM values
    setLeftPWM(currentLeftPWM);
    setRightPWM(currentRightPWM);

  }
  else {
//    digitalWriteFast(LED1, LOW);
//    digitalWriteFast(LED2, LOW);
//    digitalWriteFast(LED2, HIGH);
//    digitalWriteFast(LED1, LOW);

    // setLeftPWM(leftBaseSpeed);
    // setRightPWM(rightBaseSpeed);
    if (leftValid && rightValid) {
//    digitalWriteFast(LED2, HIGH);
//    digitalWriteFast(LED1, HIGH);
//    digitalWriteFast(LED1, LOW);
//    digitalWriteFast(LED2, LOW);
//    Serial.println("Has Both");

      // targetAngle = 0;
      // Has both walls, so error correct with both
      // (working, just need to adjust PD constants when final mouse is built)
      errorP = leftSensor - rightSensor - 100; // 100 is the offset between left and right sensor when mouse in the
                                               // middle of cell
      errorD = errorP - oldErrorP;
      // getGres();
      // gz = (float)readGyroData() * gRes - gyroBias[2];
      // straightAngle += 2 * (gz) * 0.001;
    }
    else if (leftValid) {
      const int wallDist = 1900;//Make this bigger to move closer to the wall
//    digitalWriteFast(LED1, HIGH);
//    digitalWriteFast(LED2, LOW);
//    digitalWriteFast(LED1, LOW);
//    digitalWriteFast(LED2, LOW);
//    Serial.println("Has Left");
      // Only left wall, insert one wall correction here (TODO?)
      // errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100*(angle-targetAngle);
      getGres();
      gz = (float)readGyroData() * gRes - gyroBias[2];
      angle += 2 * (gz) * 0.001;
      errorP = 20 * (angle - targetAngle) + .5 * (leftSensor - wallDist);
      errorD = errorP - oldErrorP;
    }
    else if (rightValid) {
      const int wallDist = 2000; //Make this bigger to move closer to the wall
//      digitalWriteFast(LED1, LOW);
//      digitalWriteFast(LED2, HIGH);
//      digitalWriteFast(LED1, LOW);
//      digitalWriteFast(LED2, LOW);

      // Serial.println("Has Right");
      // Only right wall, insert one wall correction here (TODO?)
      getGres();
      gz = (float)readGyroData() * gRes - gyroBias[2];
      angle += 2 * (gz) * 0.001;
      errorP = 20 * (angle - targetAngle) - .5 * (rightSensor - wallDist);
      errorD = errorP - oldErrorP;
    }
    else {
      const int wallFrontValue = 350;
//      digitalWriteFast(LED1, LOW);
//      digitalWriteFast(LED2, LOW);
//      Serial.println("Has None");
      // No walls, use encoders to correct
//      if (changeR < 0 || changeL < 0) {
//        errorP = 0;
//        errorD = 0;
//      }
//      else {
//        errorP = 1000*(changeR - changeL);
//      }
      getGres();
      gz = (float)readGyroData() * gRes - gyroBias[2];
      angle += 1 * (gz) * 0.001;
      if (rightFront > wallFrontValue && leftFront > wallFrontValue) {
        // errorP = 10* (angle - targetAngle) + 2 * (rightFront - leftFront + 10);
        errorP = 2 * (rightFront - leftFront + 10);
      }
      else {
        errorP = 20 * (angle - targetAngle);
      }
      errorD = errorP - oldErrorP;
    }
    totalError = straightKp * errorP + Kd * errorD;
    oldErrorP = errorP;

    // Calculate PWM based on Error
    currentLeftPWM = leftBaseSpeed + totalError / 124;
    currentRightPWM = rightBaseSpeed - totalError / 124;

    // Serial.print(currentLeftPWM);
    // Serial.print(" ");
    // Serial.println(currentRightPWM);

    // Update Motor PWM values
    setLeftPWM(currentLeftPWM);
    setRightPWM(currentRightPWM);
  }

  if (!straight) {
    ++i;
  }

  if (i >= curve2Time && !straight) {

    //  if (i >= curve3Time) {
    straight = true;
    
        leftValid = 0;
        rightValid = 0;

    if (moveType == TURN_RIGHT) {
      targetAngle = -110;
    }
    else {
      targetAngle = 120;
    }
    //    leftTicks = 0;
    //    rightTicks = 0;
  }

  if (straight) {
//    Serial.println(angle);

    if (wallFront()) {
      const int frontStop = 500;
      if ((leftFront + rightFront)/2 >= frontStop) {
        i = 0;
        angle = 0;
        oldErrorP = 0;
        rightTicks = 0;
        leftTicks = 0;
        moveType = NO;
        needMove = 1;
        straight = 0;
      }
    }
    else {

    
    if ((rightTicks + leftTicks) / 2 >= targetTicks) {
      
      i = 0;
      angle = 0;
      oldErrorP = 0;
      rightTicks = 0;
      leftTicks = 0;
      moveType = NO;
      needMove = 1;
      straight = false;
    }

    }

  }
}



void pivotTurnRight() {
  float angle = 0.0;
  int errorP;
  int errorD;
  int totalError;
  int oldErrorP;
  int tickCount = 190;
  // Gyro calibrated for each speed or turning is not accurate
  float degreesTraveled = 0;
  const int turnSpeed = 450;
  const float targetDegrees = 155;
  // const int turnSpeed = 45;
  // const int targetDegrees = 85.5
  // const int turnSpeed = 40;
  // const int targetDegrees = 86
  float initialZ;
//  rightTicks = 0;
//  leftTicks = 0;
//  delay(200);
  setLeftPWM(turnSpeed);
  setRightPWM(-turnSpeed);
//  while (rightTicks > -90 || leftTicks < 90) {
//  }
  
  getGres();
  gz = (float)readGyroData() * gRes - gyroBias[2];
  initialZ = gz;// May not be necessary
  count = millis();
  setLeftPWM(turnSpeed);
  setRightPWM(-turnSpeed);
  while (degreesTraveled >= -targetDegrees) {
    uint32_t deltat = millis() - count;
    if (deltat > 1) {
      getGres();
      gz = (float)readGyroData() * gRes - gyroBias[2];
      degreesTraveled += 2 * (gz - 0) * 0.001;
      Serial.println(degreesTraveled);
      count = millis();
    }
  }
//
//  // Needs to deccelerate for the motors to stop correctly
  for (int i = turnSpeed; i >= 0; i--) {
    setLeftPWM(i);
    setRightPWM(-i);
  }
//  delay(200);

}


//
//void turnLeft() {
//
//
//  //  int tickCount = 190;
//  // Gyro calibrated for each speed or turning is not accurate
//  float degreesTraveled = 0;
//  const int turnSpeed = 50;
//  const int targetDegrees = 82;
//  //  const int turnSpeed = 45;
//  //  const int targetDegrees = 85.5
//  //  const int turnSpeed = 40;
//  //  const int targetDegrees = 86
//  float initialZ;
//
//  //  rightTicks = 0;
//  //  leftTicks = 0;
//  delay(200);
//  getGres();
//  gz = (float)readGyroData()*gRes - gyroBias[2];
//  initialZ = gz;// May not be necessary
//  count = millis();
//  setLeftPWM(-turnSpeed);
//  setRightPWM(turnSpeed);
//  while (degreesTraveled <= targetDegrees) {
//    uint32_t deltat = millis() - count;
//    if (deltat > 1) {
//  getGres();
//  gz = (float)readGyroData()*gRes - gyroBias[2];
//  degreesTraveled += 2*(gz - initialZ) * 0.001;
//      count = millis();
//
//    }
//  }
//
//// Needs to deccelerate for the motors to stop correctly
//  for (int i = turnSpeed; i >= 0; i--) {
//    setLeftPWM(-i);
//    setRightPWM(i);
//
//  }
//  delay(200);
//}

void wallFollow() {
  if (needMove == 1) {
    if (!wallRight()) {
      turnRight();
    }
    else if (!wallFront()) {
      moveForward();
    }
    else if (!wallLeft()) {
      turnLeft();
    }
    else {
      turnAround();
    }
    //    needMove = 0;
  }
  //  needMove = 0;
}






