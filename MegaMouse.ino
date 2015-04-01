// Teensy 3 I2C Library
#include <i2c_t3.h>
#include "Gyro.h"
#include "Motors.h"t
#include "Sensors.h"
#include "Profiles.h"

#define hasLeftWall 800
#define hasRightWall 800
int leftBaseSpeed = 0;
int rightBaseSpeed = 0;
// PID Constants
#define Kp 16
#define Kd 10
volatile bool rightValid = 1;
volatile bool leftValid = 1;
bool go = 0;
volatile bool needMove = 1;
volatile bool haveSensorReading = 1;
IntervalTimer correctionTimer;
IntervalTimer sensorTimer;
IntervalTimer speedTimer;
volatile float angle = 0;

// mm/s
volatile float leftSpeed = 0;
volatile float rightSpeed = 0;

volatile enum {
  NO = 0,
  FORWARD = 1,
  TURN_LEFT = 2,
  TURN_RIGHT = 3,
  TURN_AROUND = 4
} moveType;

const int buttonPin = 24;
const int LED1 = 11;
const int LED2 = 16;

void setup() {

  float degreesTraveled;
  float initialZ;
  Serial.begin(115200);

  // 12 bit ADC resolution
  analogReadResolution(12);

  setupMotors();
  setupSensors();

  correctionTimer.begin(correction, 1000);
  correctionTimer.priority(255);
  sensorTimer.begin(readSensors, 80);
  sensorTimer.priority(250);


  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
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
  delay(5000);
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
  //pivotTurnRight();
  //while(1);
  //pivotTurnRight();

  //    moveForward();
  //    while (needMove == 0);
  //    moveForward();
  //    while (needMove == 0);
  //    moveForward();
  //    while (needMove == 0);
  ////    digitalWrite(LED2,LOW);
  //    turnRight();
  ////    turnLeft();
  //    while (needMove == 0);
  //    turnRight();
  ////    turnLeft();
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
  //    setLeftPWM(0);
  //    setRightPWM(0);
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
  //
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
  //  displaySensors();
  //      delay(100);
}

//mack calls certain number of move forwards, we add however many ticks for every move forward

void correction() {
  readSensors();
  switch (moveType) {
    case FORWARD :
      forwardCorrection();
      break;
    case NO :
      break;
    case TURN_RIGHT :
      turnCorrection();
      break;
    case TURN_LEFT :
      turnCorrection();
      break;
  }
  haveSensorReading = 0;
}

void moveForward() {
  digitalWriteFast(LED2, HIGH);
          digitalWriteFast(LED1, HIGH);
  rightTicks = 0;
  leftTicks = 0;
  leftBaseSpeed = 240;//240
  rightBaseSpeed = 240;//240
  if (wallRight()) {
    rightValid = 1;
  }
  else {
    rightValid = 0;
  }
  if (wallLeft()) {
    leftValid = 1;
  }
  else {
    leftValid = 0;
  }
  moveType = FORWARD;
  needMove = 0;
}

void turnRight() {
  digitalWriteFast(LED2, HIGH);
          digitalWriteFast(LED1, LOW);
  leftBaseSpeed = 240;
  rightBaseSpeed = 240;
  moveType = TURN_RIGHT;
  needMove = 0;
}

void turnLeft() {
  digitalWriteFast(LED2, LOW);
          digitalWriteFast(LED1, HIGH);
  leftBaseSpeed = 240;
  rightBaseSpeed = 240;
  moveType = TURN_LEFT;
  needMove = 0;
}

void turnAround() {
  const int frontLeftStop = 2200;
  const int frontRightStop = 2300;
  bool leftStop = 0;
  bool rightStop = 0;
  int tickCount = 180;
  int errorP;
  int errorD;
  int totalError;
  leftBaseSpeed = 240;
  rightBaseSpeed = 240;
  moveType = NO;

  while (leftStop == 0 || rightStop == 0) {
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
  leftStop = 0;
  rightStop = 0;

  pivotTurnRight();

  while (leftStop == 0 || rightStop == 0) {

    if (leftFront <= frontLeftStop - 500) {
      setLeftPWM(leftBaseSpeed);
    }
    else {
      setLeftPWM(0);
      leftStop = 1;
    }
    if (rightFront <= frontRightStop - 500) {
      setRightPWM(rightBaseSpeed);
    }
    else {
      setRightPWM(0);
      rightStop = 1;
    }
  }

  delay(100);
  pivotTurnRight();

  //read next walls here


  leftTicks = 0;
  rightTicks = 0;

  while ((rightTicks + leftTicks) / 2 <= tickCount) {
    //    getGres();
    //    gz = (float)readGyroData() * gRes - gyroBias[2];
    //    angle += 2 * (gz) * 0.001;
    //    errorP = 20 * (angle);
    ////    errorD = errorP - oldErrorP;
    //    totalError = Kp * errorP + Kd * errorD;
    ////    oldErrorP = errorP;
    //
    //  // Calculate PWM based on Error
    //  currentLeftPWM = leftBaseSpeed + totalError / 124;
    //  currentRightPWM = rightBaseSpeed - totalError / 124;
    //
    //  // Update Motor PWM values
    //  setLeftPWM(currentLeftPWM);
    //  setRightPWM(currentRightPWM);
    setLeftPWM(leftBaseSpeed);
    setRightPWM(rightBaseSpeed);
  }
  needMove = 1;
  leftTicks = 0;
  rightTicks = 0;
  //  setLeftPWM(0);
  //  setRightPWM(0);
  //  while(1);
}




void forwardCorrection() {
  const int oneCellTicks = 327;
  const int noWallRight = 250;//check this value//250
  const int noWallLeft = 450;//check this value//450

  const int pegWallBack = 800;//check this vlaue
  const int pegNoWalls = 1000;
  const int pegWallFront = 1000;

  const int wallBackTicks = 240;
  const int noWallTicks = 218;
  const int frontWallTicks = 215;

  //no walls right and left reach 1000 at 213
  //no wall front, wall back, drops below 800 at 222
  //wall front, no walls back, goes above 1000 at 196
  // encoder tick value when we check walls a cell ahead
  const int readingTicks = 173;//check this value//163
  // encoder tick value when we switch to next cell's values
  const int newSideTicks = 200;//check this value//200

  static bool nextRightValid;
  static bool nextLeftValid;
  static bool nextCellDecided = 0;
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
  static float straightAngle = 0;
  static bool endCell = 0;
  static bool currentWallLeft = 1;
  static bool currentWallRight = 1;
  static bool ticksDecided = 0;
  //getAres();
  //  Serial.println("3");
  //  digitalWrite(LED2,HIGH);
  //  az = (float)readAccelData()*aRes - accelBias[2];
  ////  delay(500);
  //  Serial.println(az);
  //  if (az > 2) {
  //    setLeftPWM(0);
  //    setRightPWM(0);
  //    leftBaseSpeed = 0;
  //    rightBaseSpeed = 0;
  //    moveType = NO;
  //  }

  // Next Cell Wall Detection
  if ((rightTicks + leftTicks) / 2 >= readingTicks && nextCellDecided == 0) {
    if (rightMiddleValue > noWallRight) {
      nextRightValid = 1;
    }
    else {
      nextRightValid = 0;
    }
    if (leftMiddleValue > noWallLeft) {
      nextLeftValid = 1;
    }
    else {
      nextLeftValid = 0;
    }
    nextCellDecided = 1;
    if (leftValid != nextLeftValid) {
      leftValid = 0;
    }
    if (rightValid != nextRightValid) {
      rightValid = 0;
    }
  }

  if ((rightTicks + leftTicks) / 2 >= newSideTicks) {
    leftValid = nextLeftValid;
    rightValid = nextRightValid;
    //    nextCellDecided = 0;
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
    errorP = leftSensor - rightSensor - 100;//100 is the offset between left and right sensor when mouse in the middle of cell
    errorD = errorP - oldErrorP;
    //      getGres();
    //      gz = (float)readGyroData() * gRes - gyroBias[2];
    //      straightAngle += 2 * (gz) * 0.001;


  }
  else if (leftValid) {
    const int wallDist = 1900;//Make this bigger to move closer to the wall
    //    digitalWriteFast(LED1, HIGH);
    //    digitalWriteFast(LED2, LOW);
//    digitalWriteFast(LED1, HIGH);
//    digitalWriteFast(LED2, LOW);
    //    Serial.println("Has Left");
    // Only left wall, insert one wall correction here
    //      errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100*(angle-targetAngle);
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
    // Only right wall, insert one wall correction here
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
    // No walls, use encoders to correct
    //    if (changeR < 0 || changeL < 0) {
    //      errorP = 0;
    //      errorD = 0;
    //    }
    //    else {
    //      errorP = 1000*(changeR - changeL);
    getGres();
    gz = (float)readGyroData() * gRes - gyroBias[2];
    angle += 2 * (gz) * 0.001;
    errorP = 20 * (angle - targetAngle);
    errorD = errorP - oldErrorP;
    //    }
  }

  //no walls right and left reach 1000 at 213
  //no wall front, wall back, drops below 800 at 222
  //wall front, no walls back, goes above 1000 at 196

    // No walls in next cell or current cell
 /*     if (currentWallRight == 0 && nextRightValid == 0 && ticksDecided == 0) {
        if (rightSensor >= pegNoWalls) {
          leftTicks = noWallTicks;
          rightTicks = noWallTicks;
          ticksDecided = 1;
        }
      }
  
      // No walls in next cell, walls in current cell
      else if (currentWallRight == 1 && nextRightValid == 0 && ticksDecided == 0) {
        if (rightSensor <= pegWallBack) {
          leftTicks = wallBackTicks;
          rightTicks = wallBackTicks;
          ticksDecided = 1;
        }
      }
  
      // No walls in current cell, wall in next cell
      else if (currentWallRight == 0 && nextRightValid == 1 && ticksDecided == 0) {
        if (rightSensor >= pegWallFront) {
            leftTicks = frontWallTicks;
            rightTicks = frontWallTicks;
            ticksDecided = 1;
        }
      }
  
    // No walls in next cell or current cell
      if (currentWallLeft == 0 && nextLeftValid == 0 && ticksDecided == 0) {
        if (leftSensor >= pegNoWalls) {
          leftTicks = noWallTicks;
          rightTicks = noWallTicks;
         ticksDecided = 1;
        }
      }
  
      // No walls in next cell, walls in current cell
      else if (currentWallLeft == 1 && nextLeftValid == 0 && ticksDecided == 0) {
        if (leftSensor <= pegWallBack) {
         leftTicks = wallBackTicks;
          rightTicks = wallBackTicks;
          ticksDecided = 1;
        }
      }
  
      // No walls in current cell, wall in next cell
      else if (currentWallLeft == 0 && nextLeftValid == 1 && ticksDecided == 0) {
        if (leftSensor >= pegWallFront) {
            leftTicks = frontWallTicks;
            rightTicks = frontWallTicks;
            ticksDecided = 1;
        }
      }
    */
  if ((rightTicks + leftTicks) / 2 >= oneCellTicks) {
    endCell = 1;
  }
  if (endCell == 1) {
    currentWallLeft = nextLeftValid;
    currentWallRight = nextRightValid;
    oldErrorP = 0;
    ticksDecided = 0;
    needMove = 1;
    //    rightTicks = 0;
    //    leftTicks = 0;
    nextCellDecided = 0;
    moveType = NO;
    endCell = 0;
  }


  totalError = Kp * errorP + Kd * errorD;
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



void turnCorrection() {
  int errorP;
  int errorD;
  int oldErrorP;
  int totalError;
  static float targetAngle;
  static int i = 0;
  static bool strait = 0;
  const int targetTicks = 145;
  //  const int targetTicks = 120;//curve3

  if (strait == 0) {
//    digitalWriteFast(LED1, HIGH);
//    digitalWriteFast(LED2, HIGH);
    targetAngle = curve2[i];
    //    targetAngle = curve3[i];
    getGres();

    gz = (int)readGyroData() * gRes - gyroBias[2];

    if (moveType == TURN_RIGHT) {
      angle += 1.37 * (gz) * 0.001;//comment out for curve3
      //      angle += 1 * (gz) * .001;
      errorP = 100 * (angle + targetAngle);//comment out for curve3
    }
    else {
      angle += 1.375 * (gz) * 0.001;//comment out for curve3
      //      angle += 1 * (gz) * .001;
      errorP = 100 * (angle - targetAngle);//comment out for curve3
    }

    //    errorP = 20 * (.85*gz + targetAngle);//curve3
    errorD = errorP - oldErrorP;

    rightTicks = 0;
    leftTicks = 0;
    totalError = Kp * errorP + Kd * errorD;
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
    //digitalWriteFast(LED2, HIGH);
    //digitalWriteFast(LED1, LOW);

    //setLeftPWM(leftBaseSpeed);
    //setRightPWM(rightBaseSpeed);
    if (leftValid && rightValid) {
//          digitalWriteFast(LED2, HIGH);
//          digitalWriteFast(LED1, HIGH);
      //      digitalWriteFast(LED1, LOW);
      //      digitalWriteFast(LED2, LOW);
      //    Serial.println("Has Both");
      angle = 0;
      //targetAngle = 0;
      // Has both wall, so error correct with both (working, just need to adjust PD constants when final mouse is built)
      errorP = leftSensor - rightSensor - 100;//100 is the offset between left and right sensor when mouse in the middle of cell
      errorD = errorP - oldErrorP;
      //      getGres();
      //      gz = (float)readGyroData() * gRes - gyroBias[2];
      //      straightAngle += 2 * (gz) * 0.001;

    }
    else if (leftValid) {
      const int wallDist = 1900;//Make this bigger to move closer to the wall
//          digitalWriteFast(LED1, HIGH);
//          digitalWriteFast(LED2, LOW);
      //      digitalWriteFast(LED1, LOW);
      //      digitalWriteFast(LED2, LOW);
      //    Serial.println("Has Left");
      // Only left wall, insert one wall correction here
      //      errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100*(angle-targetAngle);
      getGres();
      gz = (float)readGyroData() * gRes - gyroBias[2];
      angle += 2 * (gz) * 0.001;
      errorP = 20 * (angle - targetAngle) + .5 * (leftSensor - wallDist);
      errorD = errorP - oldErrorP;

    }
    else if (rightValid) {
      const int wallDist = 2000; //Make this bigger to move closer to the wall
//          digitalWriteFast(LED1, LOW);
//          digitalWriteFast(LED2, HIGH);
      //      digitalWriteFast(LED1, LOW);
      //      digitalWriteFast(LED2, LOW);

      //    Serial.println("Has Right");
      // Only right wall, insert one wall correction here
      getGres();
      gz = (float)readGyroData() * gRes - gyroBias[2];
      angle += 2 * (gz) * 0.001;
      errorP = 20 * (angle - targetAngle) - .5 * (rightSensor - wallDist);
      errorD = errorP - oldErrorP;
    }
    else {
//          digitalWriteFast(LED1, LOW);
//            digitalWriteFast(LED2, LOW);
      //    Serial.println("Has None");
      // No walls, use encoders to correct
      //    if (changeR < 0 || changeL < 0) {
      //      errorP = 0;
      //      errorD = 0;
      //    }
      //    else {
      //      errorP = 1000*(changeR - changeL);
      getGres();
      gz = (float)readGyroData() * gRes - gyroBias[2];
      angle += 1 * (gz) * 0.001;
      
      errorP = 20 * (angle - targetAngle);
      errorD = errorP - oldErrorP;
      //    }
    }
    totalError = Kp * errorP + Kd * errorD;
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

  if (strait == 0) {
    i++;
  }

  if (i >= curve2Time && strait == 0) {
    //  if (i >= curve3Time) {
    strait = 1;

    //    leftValid = 0;
    //    rightValid = 0;



    rightValid = 0; //wallRight();
    leftValid = 0; //wallLeft();
    if (moveType == TURN_RIGHT) {
      targetAngle = -110;
    }
    else {
      targetAngle = 120;
    }
    //    leftTicks = 0;
    //    rightTicks = 0;
  }

  if (strait == 1) {
//    Serial.println(angle);
    if ((rightTicks + leftTicks) / 2 >= targetTicks) {
      
      i = 0;
      angle = 0;
      oldErrorP = 0;
      rightTicks = 0;
      leftTicks = 0;
      moveType = NO;
      needMove = 1;
      strait = 0;
    }
  }
}



void pivotTurnRight() {

  static float angle = 0;
  int errorP;
  int errorD;
  int totalError;
  int oldErrorP;
  int tickCount = 190;
  // Gyro calibrated for each speed or turning is not accurate
  float degreesTraveled = 0;
  const int turnSpeed = 400;
  const float targetDegrees = 82;
  //  const int turnSpeed = 45;
  //  const int targetDegrees = 85.5
  //  const int turnSpeed = 40;
  //  const int targetDegrees = 86
  float initialZ;
  //
  //  rightTicks = 0;
  //  leftTicks = 0;
  delay(200);
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

  // Needs to deccelerate for the motors to stop correctly
  for (int i = turnSpeed; i >= 0; i--) {
    setLeftPWM(0);
    setRightPWM(0);

  }
  delay(200);
  //if (rightTicks == 200) {
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

void getSpeed() {
  const float tickDistance = 0.53;//mm
  static int leftTicksLast = 0;
  static int rightTicksLast = 0;
  int leftTicksLocal = leftTicks;
  int rightTicksLocal = rightTicks;
  int changeLeft = leftTicks - leftTicksLast;
  int changeRight = rightTicks - rightTicksLast;
  if (changeLeft < 0 || changeRight < 0) {
    leftTicksLast = leftTicksLocal;
    rightTicksLast = rightTicksLocal;
  }
  else {
    leftSpeed = (float)changeLeft * tickDistance / .01;//mm/s
    rightSpeed = (float)changeRight * tickDistance / .01;//mm/s
    leftTicksLast = leftTicksLocal;
    rightTicksLast = rightTicksLocal;
  }
}




