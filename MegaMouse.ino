// Teensy 3 I2C Library
#include <i2c_t3.h>
#include "Gyro.h"
#include "Motors.h"
#include "Sensors.h"
#include "Profiles.h"

#define hasLeftWall 500
#define hasRightWall 800
int leftBaseSpeed = 0;
int rightBaseSpeed = 0;
// PID Constants
#define Kp 16
#define Kd 10
bool rightValid = 1;
bool leftValid = 1;
bool go = 0;
volatile bool needMove = 1;
volatile bool haveSensorReading = 0;
IntervalTimer correctionTimer;
IntervalTimer sensorTimer;
float angle = 0;
volatile enum {
  NO = 0,
  FORWARD = 1,
  TURN_LEFT = 2,
  TURN_RIGHT = 3,
  TURN_AROUND = 4
} moveType;

const int buttonPin = 24;

const int LED2 = 16;

void setup() {
  float degreesTraveled;
  float initialZ;
  Serial.begin(115200);

  // 12 bit ADC resolution
  analogReadResolution(12);
  correctionTimer.begin(correction, 1000);
  correctionTimer.priority(255);
  sensorTimer.begin(readSensors, 80);
  sensorTimer.priority(250);
  
  setupMotors();
  setupSensors();

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
  //turnRight();
  //  go = 1;
  //  turnRight();
  //turnLeft();
  //moveType = NO;
  while (1) {
    moveForward();
    while (needMove == 0);
    moveForward();
    while (needMove == 0);
    moveForward();
    while (needMove == 0);
    moveForward();
    while (needMove == 0);
    turnRight();
    while (needMove == 0);
    turnRight();
    while (needMove == 0);
  }
}

void loop() {
//  setLeftPWM(30);
//  setRightPWM(40);
  //  Serial.print("Wall Left = ");
  //  Serial.print(wallLeft());
  //  Serial.print(" ");
  //  Serial.print("Wall Right = ");
  //  Serial.print(wallRight());
  //  Serial.print(" ");
  //  Serial.print("Wall Front = ");
  //  Serial.println(wallFront());
  //  delay(100);
}

//mack calls certain number of move forwards, we add however many ticks for every move forward

void correction() {
  switch (moveType) {
    case FORWARD :
      forwardCorrection();
      break;
    case NO :
      break;
    case TURN_RIGHT :
      rightCorrection();
      break;
    case TURN_LEFT :
      break;
  }
  haveSensorReading = 0;
}

void moveForward() {
  leftBaseSpeed = 30;
  rightBaseSpeed = 40;
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
  leftBaseSpeed = 30;
  rightBaseSpeed = 40;
  moveType = TURN_RIGHT;
  needMove = 0;
}

void turnLeft() {
  leftBaseSpeed = 30;
  rightBaseSpeed = 40;
  moveType = TURN_LEFT;
  needMove = 0;
}

void forwardCorrection() {
  const int oneCellTicks = 330;
  const int readingTicks = 180;
  const int noWallRight = 800;//check this value
  const int noWallLeft = 800;//check this value
  const int newSideTicks = 200;//check this value
  //the start square is bounded by three walls so these two will always start as 1
  bool nextRightValid;
  bool nextLeftValid;
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
    //Maybe use this point to tell the algorithm where the walls are
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
  }

  if ((rightTicks + leftTicks) / 2 >= newSideTicks) {
    leftValid = nextLeftValid;
    rightValid = nextRightValid;
    nextCellDecided = 0;
  }


  if (leftValid && rightValid) {
    digitalWriteFast(LED2, HIGH);
    //    Serial.println("Has Both");
    angle = 0;
    targetAngle = 0;
    // Has both wall, so error correct with both (working, just need to adjust PD constants when final mouse is built)
    errorP = leftSensor - rightSensor + 100;//100 is the offset between left and right sensor when mouse in the middle of cell
    errorD = errorP - oldErrorP;
//      getGres();
//      gz = (float)readGyroData() * gRes - gyroBias[2];
//      straightAngle += 2 * (gz) * 0.001;
    

  }
  else if (leftValid) {
    digitalWriteFast(LED2, LOW);
    //    Serial.println("Has Left");
    // Only left wall, insert one wall correction here
    //      errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100*(angle-targetAngle);
    getGres();
    gz = (float)readGyroData() * gRes - gyroBias[2];
    angle += 2 * (gz) * 0.001;
    errorP = 100 * (angle - targetAngle) + (leftSensor - 2000);
    errorD = errorP - oldErrorP;

  }
  else if (rightValid) {
    digitalWriteFast(LED2, LOW);
    //    Serial.println("Has Right");
    // Only right wall, insert one wall correction here
    getGres();
    gz = (float)readGyroData() * gRes - gyroBias[2];
    angle += 2 * (gz) * 0.001;
    errorP = 100 * (angle - targetAngle) - (rightSensor - 2000);
    errorD = errorP - oldErrorP;
  }
  else {
    digitalWriteFast(LED2, LOW);
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
    errorP = 100 * (angle - targetAngle);
    errorD = errorP - oldErrorP;
    //    }
  }
  
  //pick up detection
  if (abs(angle - targetAngle) > 20) {
    moveType = NO;
    leftBaseSpeed = 0;
    rightBaseSpeed = 0;
    currentLeftPWM = 0;
    currentRightPWM = 0;
    setLeftPWM(0);
    setRightPWM(0);
  }

  if ((rightTicks + leftTicks) / 2 >= oneCellTicks) {
    needMove = 1;
    rightTicks = 0;
    leftTicks = 0;
    nextCellDecided = 0;
    moveType = NO;
  }


  totalError = Kp * errorP + Kd * errorD;
  oldErrorP = errorP;

  // Calculate PWM based on Error
  currentLeftPWM = leftBaseSpeed + totalError / 1000;
  currentRightPWM = rightBaseSpeed - totalError / 1000;

  //  Serial.print(currentLeftPWM);
  //  Serial.print(" ");
  //  Serial.println(currentRightPWM);

  // Update Motor PWM values
  setLeftPWM(currentLeftPWM);
  setRightPWM(currentRightPWM);
}



void leftCorrection() {
  int errorP;
  static int errorD;
  int oldErrorP;
  int totalError;
  float targetAngle;
  static int i = 0;
  targetAngle = curve1[i];
  getGres();
  gz = (float)readGyroData() * gRes - gyroBias[2];
  angle += 1.15 * (gz) * 0.001;
  errorP = 100 * (angle - targetAngle);
  errorD = errorP - oldErrorP;

  totalError = Kp * errorP + Kd * errorD;
  oldErrorP = errorP;

  // Calculate PWM based on Error
  currentLeftPWM = leftBaseSpeed + totalError / 1000;
  currentRightPWM = rightBaseSpeed - totalError / 1000;

  // Update Motor PWM values
  setLeftPWM(currentLeftPWM);
  setRightPWM(currentRightPWM);

  i++;
  if (i >= curve1Time) {
    rightTicks = 0;
    leftTicks = 0;
    moveType = NO;
    needMove = 1;
  }
}

void rightCorrection() {
  int errorP;
  static int errorD;
  int oldErrorP;
  int totalError;
  float targetAngle;
  static int i = 0;
  targetAngle = curve1[i];
  getGres();
  gz = (float)readGyroData() * gRes - gyroBias[2];
  angle += 1.0 * (gz) * 0.001;


  errorP = 45 * (angle + targetAngle);
  errorD = errorP - oldErrorP;

  totalError = Kp * errorP + Kd * errorD;
  oldErrorP = errorP;

  // Calculate PWM based on Error
  currentLeftPWM = leftBaseSpeed + totalError / 1000;
  currentRightPWM = rightBaseSpeed - totalError / 1000;

  // Update Motor PWM values
  setLeftPWM(currentLeftPWM);
  setRightPWM(currentRightPWM);

  //  if (i < curve1Time) {
  i++;
  //  }
  if (i >= curve1Time) {
    i = 0;
    angle = 0;
    rightTicks = 0;
    leftTicks = 0;
    moveType = NO;
    needMove = 1;
  }

}

//void turnRight() {
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
//  setLeftPWM(turnSpeed);
//  setRightPWM(-turnSpeed);
//  while (degreesTraveled >= -targetDegrees) {
//    uint32_t deltat = millis() - count;
//    if (deltat > 1) {
//  getGres();
//  gz = (float)readGyroData()*gRes - gyroBias[2];
//  degreesTraveled += 2*(gz - initialZ) * 0.001;
//  Serial.println(degreesTraveled);
//      count = millis();
//
//    }
//  }
//
//  // Needs to deccelerate for the motors to stop correctly
//  for (int i = turnSpeed; i >= 0; i--) {
//    setLeftPWM(i);
//    setRightPWM(-i);
//
//  }
//  delay(200);
//}
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

boolean wallFront() {
  return (leftFront > 2900 && rightFront > 2900);
}

boolean wallLeft() {
  //  readSensors();
  return (leftSensor > hasLeftWall);
  leftValid = 1;
}

boolean wallRight() {
  //  readSensors();
  return (rightSensor > hasRightWall);
  rightValid = 1;
}

void wallFollow() {
  if (!wallRight) {
    moveType = TURN_RIGHT;
  }
  else if (wallFront()) {
    moveType = TURN_LEFT;
  }
  else {
    moveType = FORWARD;
  }
}





