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
volatile bool needMove=1;
IntervalTimer correctionTimer;
float angle = 0;
enum {
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
  correctionTimer.begin(correction,1000);
  correctionTimer.priority(255);
  setupMotors();
  setupSensors();

  pinMode(LED2, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(intPin, INPUT);

  // Wait for Button Press to Start
//  readSensors();
  while (digitalRead(buttonPin) == 1) {
  }
//  while (rightFront < 3300 && rightMiddleValue < 3300 && rightSensor < 3300) {
//    readSensors();
////    Serial.println(leftSensor);
//  }
  delay(1000);
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
  Serial.println("asdf");
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
  readSensors();
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
      leftCorrection();
  }
}

void moveForward() {
  leftBaseSpeed = 30;
  rightBaseSpeed = 40;
  moveType = FORWARD;
  needMove = 0;
}

void turnRight() {
  leftBaseSpeed = 30;
  rightBaseSpeed = 40;
  moveType = TURN_RIGHT;
  needMove = 0;
  Serial.println("RIGHT");
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
      
  
  // Next Cell Wall Detection
  if ((rightTicks + leftTicks)/2 >= readingTicks && nextCellDecided==0) {
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
  
//  if ((rightTicks + leftTicks)/2 >= oneCellTicks) {
//    rightTicks = 0;
//    leftTicks = 0;
//  }
  
  if (leftValid && rightValid) {
    digitalWrite(LED2,HIGH);
//    Serial.println("Has Both");
    angle = 0;
    // Has both wall, so error correct with both (working, just need to adjust PD constants when final mouse is built)
    errorP = leftSensor - rightSensor + 100;//100 is the offset between left and right sensor when mouse in the middle of cell
    errorD = errorP - oldErrorP;
    if ((rightTicks+leftTicks)/2 >= 200) {
      getGres(); 
      gz = (float)readGyroData()*gRes - gyroBias[2];
      targetAngle += 2*(gz) * 0.001;
    }
    
  }
  else if (leftValid) {
    digitalWrite(LED2,LOW);
//    Serial.println("Has Left");
    // Only left wall, insert one wall correction here
//      errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100*(angle-targetAngle);
      errorP = 100*(angle-targetAngle) + (leftSensor-2000);
      errorD = errorP - oldErrorP;
      
  }
  else if (rightValid) {
    digitalWrite(LED2,LOW);
//    Serial.println("Has Right");
    // Only right wall, insert one wall correction here
    errorP = 100*(angle-targetAngle) - (rightSensor-2000);
    errorD = errorP - oldErrorP;
  }
  else {
    digitalWrite(LED2,LOW);
//    Serial.println("Has None");
    // No walls, use encoders to correct
//    if (changeR < 0 || changeL < 0) {
//      errorP = 0;
//      errorD = 0;
//    }
//    else {
//      errorP = 1000*(changeR - changeL);
      getGres(); 
      gz = (float)readGyroData()*gRes - gyroBias[2];
      angle += 2*(gz) * 0.001;
      errorP = 100*(angle-targetAngle);
      errorD = errorP - oldErrorP;
//    }
  }
  
  if ((rightTicks + leftTicks)/2 >= oneCellTicks) {
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
      gz = (float)readGyroData()*gRes - gyroBias[2];
      angle += 1.15*(gz) * 0.001;
      errorP = 100*(angle-targetAngle);
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
      gz = (float)readGyroData()*gRes - gyroBias[2];
      angle += 1.15*(gz) * 0.001;
      errorP = 100*(angle+targetAngle);
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
    angle = 0;
    i = 0;
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
  return (leftFront>2900 && rightFront>2900);
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





