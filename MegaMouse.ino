// Teensy 3 I2C Library
#include <i2c_t3.h>
#include "Gyro.h"
#include "Motors.h"
#include "Sensors.h"
#include "Profiles.h"
#include <SoftwareSerial.h>

#define hasLeftWall 800
#define hasRightWall 800
int exploreSpeed = 240;
int solveSpeed = 240;

int leftBaseSpeed = exploreSpeed;
int rightBaseSpeed = exploreSpeed;
const int rightWallDist = 2000;
const int leftWallDist = 1900;
// PID Constants
#define straightKp 16
#define turnKp 16
#define Kd 10

/* Variables for interface between drive code and algorithm */
volatile char movesBuffer[256];
char bluetoothBuffer[5];
volatile bool walls_global[3] = {false, false, false}; // Left, Front, Right
volatile bool movesReady = false; // Set to true by algorithm, set to false by drive.
volatile bool movesDoneAndWallsSet = false; // Set to true by drive, set to false by algorithm.
/* End of variables for interface */

const int maxSpeed = 450;

bool currentMoveDone = false;
bool firstMove = true;
bool accelerate = true;
bool solving = 1;
int goalSpeed = 0;

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
volatile bool afterTurnAround = false;
volatile bool rightValid = true;
volatile bool leftValid = true;
bool go = false;
volatile bool haveSensorReading = false;

IntervalTimer correctionTimer;
IntervalTimer sensorTimer;
IntervalTimer refreshSensorTimer;
//SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);
volatile float angle = 0.0;

volatile enum {
  NO = 0,
  FORWARD = 1,
  TURN_LEFT = 2,
  TURN_RIGHT = 3,
  TURN_AROUND = 4
} moveType;

volatile enum {
  PRE_TURN = 0,
  TURN = 1,
  POST_TURN = 2
} turnSection;

const int buttonPin = 24;
const int LED = 11;

void setup() {

  float degreesTraveled;
  float initialZ;
  //Serial.begin(9600);
  //mySerial.begin(115200);
  //    Serial.begin(115200);
  // 12 bit ADC resolution
  analogReadResolution(12);

  setupMotors();
  setupSensors();

  pinMode(LED, OUTPUT);

  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(intPin, INPUT);

  // Wait for Button Press to Start
  // readSensors();
  while (digitalRead(buttonPin) == 1) {
  }
  //while (rightFront < 3300 && rightMiddleValue < 3300 && rightSensor < 3300) {
  //        readSensors();
  //        Serial.println(leftSensor);
  //}

  delay(3000);
  setupGyro();
  delay(1000);
  sensorTimer.priority(250);
  sensorTimer.begin(readSensors, 80);
  while (!haveSensorReading) {
  }
  walls_global[0] = wallLeft();
  walls_global[1] = wallFront();
  walls_global[2] = wallRight();
  haveSensorReading = false;
  movesDoneAndWallsSet = true;
  correctionTimer.priority(255);
  correctionTimer.begin(correction, 1000);
}

#include "MackAlgo.h"
mack::MackAlgo algo;
void loop() {
  /// Serial.println("Left: " + walls_global[0] ? "yes":"no");
  ///Serial.println("Front: " + walls_global[1] ? "yes":"no");
  ////////  Serial.println("Right: " + walls_global[2] ? "yes":"no");
  //  wheelCalib();
    algo.solve();
  //   bluetoothPrint();
//  solve();

}

void wheelCalib() {
  Serial.begin(9600);
  correctionTimer.end();
  sensorTimer.end();
  refreshSensorTimer.end();
  setLeftPWM(240);
  setRightPWM(247);
  while (1) {
    delay(1000);
    //Serial.print(leftTicks);
    // Serial.print(" ");
    //Serial.println(rightTicks);
    leftTicks = 0;
    rightTicks = 0;
  }
}


// Mack calls certain number of move forwards; we add however many ticks for every move forward.

void correction() {
  static int totalForwardCount = 0;
  static int forwardCount = 0;
  static bool in_acceleration = false;
  static byte indexInBuffer = 0;
  static bool movedForward = false;
  if (!movesReady) {
    // Hoping we never get here, but maybe the algorithm is slow.
    haveSensorReading = false;
    return;
  }

  if (currentMoveDone) {

    movedForward = false;
    //        bluetoothPrint();
    if (firstMove) {
      firstMove = false;
    }
    if (forwardCount != 0) {
      forwardCount--;
    } else {
      totalForwardCount = 0;
      in_acceleration = false;
    }
    indexInBuffer += 1;
    currentMoveDone = false;
    if (movesBuffer[indexInBuffer] == 0) {
      // Make sure walls_global is set by the time we get here (STILL NEED TO DO IN TURN AROUND).
      movesReady = false;

      //mySerial.print(walls_global[0]);
      //mySerial.print(walls_global[1]);
      //mySerial.println(walls_global[2]);
      movesDoneAndWallsSet = true;
      indexInBuffer = 0;
      haveSensorReading = false;
      return;
    }
  }

  switch (movesBuffer[indexInBuffer]) {
    case 'f':
      if (!in_acceleration) {
        forwardCount = 1;
        int index = indexInBuffer + 1;
        while (movesBuffer[index++] == 'f') {
          forwardCount++;
        }
        totalForwardCount = forwardCount;
        if (solving) {
          goalSpeed = (int)solveSpeed*(float)forwardCount;
          if (goalSpeed > maxSpeed)
            goalSpeed = maxSpeed;
          if (goalSpeed < solveSpeed)
            goalSpeed = solveSpeed;
        }
        else {
          goalSpeed = (int)exploreSpeed * (float)forwardCount;
          if (goalSpeed < exploreSpeed)
            goalSpeed = exploreSpeed;
          if (goalSpeed > maxSpeed)
            goalSpeed = maxSpeed;
        }
        in_acceleration = true;
      }
      else {
        // if((float)totalForwardCount / (float)(forwardCount) == totalForwardCount) {
        if (forwardCount == 2) {
          goalSpeed = 240;
        }
      }
      moveType = FORWARD;

      if (!movedForward) {
        movedForward = true;
        moveForward();
      }
      forwardCorrection();
      break;
    case 'r':
      moveType = TURN_RIGHT;
      if (firstMove) {
        correctionTimer.end();
        leftTicks = 0;
        rightTicks = 0;
        rightTurnFirstCell();
        //haveSensorReading = false;
        // digitalWriteFast(LED, HIGH);
        // while(!haveSensorReading) {}
        //digitalWriteFast(LED, LOW);
        // walls_global[0] = wallLeft();
        //walls_global[1] = wallFront();
        // walls_global[2] = wallRight();
        correctionTimer.priority(255);
        walls_global[0] = wallLeft();
        walls_global[1] = wallFront();
        walls_global[2] = wallRight();
        haveSensorReading = false;
        correctionTimer.begin(correction, 1000);
        return;
      }
      turnCorrection();
      break;
    case 'l': // Fall-through"
      moveType = TURN_LEFT;
      turnCorrection();
      break;
    case 'a':
      correctionTimer.end();
      leftTicks = 0;
      rightTicks = 0;
      turnAround();
      //haveSensorReading = false;
      // digitalWriteFast(LED, HIGH);
      // while(!haveSensorReading) {}
      //digitalWriteFast(LED, LOW);
      // walls_global[0] = wallLeft();
      //walls_global[1] = wallFront();
      // walls_global[2] = wallRight();
      currentMoveDone = true;
      haveSensorReading = false;
      correctionTimer.priority(255);
      correctionTimer.begin(correction, 1000);
      return;
    default:
      moveType = NO;
      //setLeftPWM(0);
      //setRightPWM(0);

      // Don't need to do anything here if we're turning around.
  }

  haveSensorReading = false;
}

void moveForward() {
  // digitalWriteFast(LED2, HIGH);

  // digitalWriteFast(LED1, HIGH);
  if (firstCell) {
    rightTicks = 70;
    leftTicks = 70;
    firstCell = false;
    leftBaseSpeed = 0;
    rightBaseSpeed = 0;
    accelerate = true;
  }
  else if (afterTurnAround) {
    leftBaseSpeed = 0;
    rightBaseSpeed = 0;
    accelerate = true;
    rightTicks = 100;
    leftTicks = 100;
    afterTurnAround = false;
  }
  else {
    rightTicks = 0;
    leftTicks = 0;
  }
  //  if(!accelerate) {
  //      leftBaseSpeed = 240;//240
  //      rightBaseSpeed = 240;//240
  //  }

  rightValid = wallRight();
  leftValid = wallLeft();
  //bluetoothBuffer[0] = leftValid;
  //bluetoothBuffer[1] = ' ';
  //bluetoothBuffer[2] = rightValid;
  //bluetoothBuffer[3] = 0;
  //bluetoothPrint();
  moveType = FORWARD;
  //    needMove = false;
}

//void turnRight() {
//  leftBaseSpeed = 240;
//  rightBaseSpeed = 240;
//  moveType = TURN_RIGHT;
//  //    needMove = false;
//}
//
//void turnLeft() {
//  leftBaseSpeed = 240;
//  rightBaseSpeed = 240;
//  moveType = TURN_LEFT;
//  //    needMove = false;
//}

void turnAround() {
  const int frontLeftStop = 1400;
  const int frontRightStop = 1400;
  bool leftStop = false;
  bool rightStop = false;
  int tickCount = 180;
  int errorP;
  int errorD;
  int totalError;
  bool front;
  leftBaseSpeed = 240;
  rightBaseSpeed = 249;
  moveType = NO;
  if (wallFront()) {
    afterTurnAround = true;
    front = false;
  }
  else {
    afterTurnAround = true;
    front = false;
  }
  if (front) {
    while (rightFront <= frontRightStop || leftFront <= frontLeftStop) {
      if (wallRight() && wallLeft()) {
        errorP = leftSensor - rightSensor - 100; // 100 is the offset between left and right sensor when mouse in the
        // middle of cell
      }
      else if (wallRight()) {
        const int wallDist = rightWallDist; //Make this bigger to move closer to the wall
        // Only right wall
        getGres();
        gz = (float)readGyroData() * gRes - gyroBias[2];
        angle += 2 * (gz) * 0.001;
        errorP = 20 * (angle) - .5 * (rightSensor - wallDist);
        errorD = errorP;
      }
      else if (wallLeft()) {
        const int wallDist = leftWallDist; // Make this bigger to move closer to the wall
        // Only left wall
        // errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100 * (angle - targetAngle);
        getGres();
        gz = (float)readGyroData() * gRes - gyroBias[2];
        angle += 2 * (gz) * 0.001;
        errorP = 20 * (angle) + .5 * (leftSensor - wallDist);
        errorD = errorP;
      }
      else {
        getGres();
        gz = (float)readGyroData() * gRes - gyroBias[2];
        angle += 2 * (gz) * 0.001;
        errorP = 20 * (angle);
        errorD = errorP;
      }
      //      errorP += 3*(rightFront - leftFront);
      errorD = errorP;
      totalError = straightKp * errorP + Kd * errorD;

      // Calculate PWM based on Error
      currentLeftPWM = leftBaseSpeed + totalError / 124;
      currentRightPWM = rightBaseSpeed - totalError / 124;

      // Update Motor PWM values
      setLeftPWM(currentLeftPWM);
      setRightPWM(currentRightPWM);

      //TODO (this is a hack and shouldn't be here, but it makes it work)
      haveSensorReading = false;
      while (!haveSensorReading) {
        readSensors();
        delayMicroseconds(80);
      }
    }
  }
  //Turn Around with no wall in front
  else {
    const int tickValue = 100;
    while ((rightTicks + leftTicks) / 2 < tickValue) {
      if (wallRight() && wallLeft()) {
        errorP = leftSensor - rightSensor - 100; // 100 is the offset between left and right sensor when mouse in the
        // middle of cell
      }
      else if (wallRight()) {
        // Only right wall
        getGres();
        gz = (float)readGyroData() * gRes - gyroBias[2];
        angle += 2 * (gz) * 0.001;
        errorP = 20 * (angle) - .5 * (rightSensor - rightWallDist);
        errorD = errorP;
      }
      else if (wallLeft()) {
        // Only left wall
        // errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100 * (angle - targetAngle);
        getGres();
        gz = (float)readGyroData() * gRes - gyroBias[2];
        angle += 2 * (gz) * 0.001;
        errorP = 20 * (angle) + .5 * (leftSensor - leftWallDist);
        errorD = errorP;
      }
      else {
        getGres();
        gz = (float)readGyroData() * gRes - gyroBias[2];
        angle += 2 * (gz) * 0.001;
        errorP = 20 * (angle);
        errorD = errorP;
      }
      errorD = errorP;
      totalError = straightKp * errorP + Kd * errorD;

      // Calculate PWM based on Error
      currentLeftPWM = leftBaseSpeed + totalError / 124;
      currentRightPWM = rightBaseSpeed - totalError / 124;

      // Update Motor PWM values
      setLeftPWM(currentLeftPWM);
      setRightPWM(currentRightPWM);

      //TODO (this is a hack and shouldn't be here, but it makes it work)
      haveSensorReading = false;
      while (!haveSensorReading) {
        readSensors();
        delayMicroseconds(80);
      }
    }
  }
  setRightPWM(0);
  setLeftPWM(0);
  delay(200);
  leftStop = false;
  rightStop = false;

  pivotTurnRight();

  angle = 0.0;
  delay(200);
  if (front) {
    setLeftPWM(-150);
    setRightPWM(-150);
    delay(350);
    for (int i = -150; i < 0; ++i) {
      setLeftPWM(i);
      setRightPWM(i);
    }
    delay(200);
    firstCell = true;
  }
  else {
    delay(200);
    afterTurnAround = true;

  }

  //    needMove = true;
}

void forwardCorrection() {
  digitalWrite(LED, LOW);
  const int oneCellTicks = 327;
  const int noWallRight = 250; // check this value (250)
  const int noWallLeft = 450; // check this value (450)

  const int pegWallBack = 800; // check this value
  const int pegNoWalls = 1000;
  const int pegWallFront = 1000;

  const int wallBackTicks = 232;
  const int noWallTicks = 209;
  const int frontWallTicks = 204;

  // no walls right and left reach 1000 at 213
  // no wall front, wall back, drops below 800 at 222
  // wall front, no walls back, goes above 1000 at 196
  // encoder tick value when we check walls a cell ahead
  const int readingTicks = 173; // check this value (163)
  // encoder tick value when we switch to next cell's values
  const int newSideTicks = 230; // check this value (200)

  static bool nextRightValid;
  static bool nextLeftValid;
  static bool nextCellDecided = false;
  //    int righTicksRemaining;
  //    int leftTicksRemaining;
  int errorP;
  int errorD;
  int oldErrorP = 0;
  int totalError;
  //    static float angle;
  static int lastTicksL;
  static int lastTicksR;
  static float straightAngle = 0.0;
  static bool endCell = false;
  static bool currentWallLeft = true;
  static bool currentWallRight = true;
  static bool ticksDecided = false;
  static int count = 0;

  if (accelerate) {

    if (leftBaseSpeed == 0) {
      leftBaseSpeed = 30;
      rightBaseSpeed = 30;
    }
    count++;

    if (count % 1 == 0) {
      if (leftBaseSpeed < goalSpeed) {
        leftBaseSpeed++;
        rightBaseSpeed++;
      } else if (leftBaseSpeed > goalSpeed) {
        leftBaseSpeed = goalSpeed;
        rightBaseSpeed = goalSpeed;
      } else {
        count = 0;
      }
    }

  }
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
    //mySerial.println(rightValid);
  }

  if ((rightTicks + leftTicks) / 2 >= newSideTicks) {
    leftValid = nextLeftValid;
    rightValid = nextRightValid;
    // nextCellDecided = false;
  }

  if (leftValid && rightValid) {
    angle = 0.0;

    // Has both wall, so error correct with both (working, just need to adjust PD constants when final mouse is built)
    errorP = leftSensor - rightSensor - 100; // 100 is the offset between left and right sensor when mouse in the
    // middle of cell
    errorD = errorP - oldErrorP;
    //        getGres();
    //        gz = (float)readGyroData() * gRes - gyroBias[2];
    //        straightAngle += 2 * (gz) * 0.001;
  }
  else if (leftValid) {
    
    // Only left wall
    // errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100 * (angle - targetAngle);
    getGres();
    gz = (float)readGyroData() * gRes - gyroBias[2];
    angle += 2 * (gz) * 0.001;
    errorP = 20 * (angle - 5) + .5 * (leftSensor - leftWallDist);
    errorD = errorP - oldErrorP;
  }
  else if (rightValid) {
    // Only right wall
    getGres();
    gz = (float)readGyroData() * gRes - gyroBias[2];
    angle += 2 * (gz) * 0.001;
    errorP = 20 * (angle) - .5 * (rightSensor - rightWallDist);
    errorD = errorP - oldErrorP;
  }
  else {
    static int targetAngle = 0;
    // No walls, use gyro to correct
    getGres();
    gz = (float)readGyroData() * gRes - gyroBias[2];
    angle += 2 * (gz) * 0.001;
//    if (!wallFront) {
//      errorP = -20 * (leftFront - rightFront) + 20*angle;
//    }
//    else {
//    if (rightSensor > 1800 && !currentWallRight) {
//      targetAngle+=2;
//    }
//    if (leftSensor > 1800 && !currentWallLeft) {
//      targetAngle-=2;
//    }

       errorP = 20 * (angle - targetAngle);
    
//    }
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
  if (!currentWallLeft && !nextLeftValid && !ticksDecided) {
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
    ticksDecided = false;
    walls_global[0] = wallLeft();
    walls_global[1] = wallFront();
    walls_global[2] = wallRight();
    currentMoveDone = true;
    //        needMove = true;
    nextCellDecided = false;
    moveType = NO;
    endCell = false;
  }


  totalError = straightKp * errorP + Kd * errorD;
  oldErrorP = errorP;

  // Calculate PWM based on Error
  currentLeftPWM = leftBaseSpeed + totalError / 124;
  currentRightPWM = rightBaseSpeed - totalError / 124;

  // Update Motor PWM values
  setLeftPWM(currentLeftPWM);
  setRightPWM(currentRightPWM);
}

void turnCorrection() {
  int errorP;
  int errorD = 0;
  int oldErrorP = 0;
  int totalError;
  static float targetAngle;
  static int i = 0;
  static bool turn = false;
  static bool straight = 0;
  const int targetTicks = 140;
  const int frontOffset = 000; // difference between left and right front sensors when lined up with the wall

  if (!straight) {
    if (turn) {
      targetAngle = curve2[i];
      getGres();

      gz = (int)readGyroData() * gRes - gyroBias[2];

      //Smaller value = overturn
      if (moveType == TURN_RIGHT) {
        angle += 1.375 * (gz) * 0.001;
        errorP = 100 * (angle + targetAngle);
      }
      else {
        angle += 1.375 * (gz) * 0.001;
        errorP = 100 * (angle - targetAngle);
      }


      errorD = errorP - oldErrorP;

      rightTicks = 0;
      leftTicks = 0;
      totalError = turnKp * errorP + Kd * errorD;
      oldErrorP = errorP;

      // Calculate PWM based on Error
      currentLeftPWM = leftBaseSpeed + totalError / 124;
      currentRightPWM = rightBaseSpeed - totalError / 124;

      // Update Motor PWM values
      setLeftPWM(currentLeftPWM);
      setRightPWM(currentRightPWM);
      ++i;
    }
    //end if (turn)
    else {
      const int frontStop = 650;
      if (wallFront()) {
        if (wallLeft()) {
          // Only left wall
          // errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100 * (angle - targetAngle);
          getGres();
          gz = (float)readGyroData() * gRes - gyroBias[2];
          angle += 2 * (gz) * 0.001;
          //                    errorP = 20 * (angle - targetAngle) + .5 * (leftSensor - wallDist);
          errorP = .5 * (leftSensor - leftWallDist) + 3 * (rightFront - leftFront - frontOffset);
//            errorP = .5 * (leftSensor - leftWallDist);
          //                    errorP = 3 * (rightFront - leftFront - frontOffset);
          errorD = errorP - oldErrorP;
        }
        else if (wallRight()) {
          // Only right wall
          getGres();
          gz = (float)readGyroData() * gRes - gyroBias[2];
          angle += 2 * (gz) * 0.001;
          //                    errorP = 20 * (angle - targetAngle) - .5 * (rightSensor - wallDist);
          //                    errorP = 3 * (rightFront - leftFront - frontOffset);
          errorP = -.5 * (rightSensor - rightWallDist) + 3 * (rightFront - leftFront - frontOffset);
          errorD = errorP - oldErrorP;
        }
        else {
          getGres();
          gz = (float)readGyroData() * gRes - gyroBias[2];
          angle += 2 * (gz) * 0.001;
          errorP = 3 * (rightFront - leftFront - frontOffset);
          //                    errorP = 20 * (angle - targetAngle);
          errorD = errorP - oldErrorP;
        }
        //                getGres();
        //                gz = (float)readGyroData() * gRes - gyroBias[2];
        //                angle += 1 * (gz) * 0.001;
        errorP = 3 * (rightFront - leftFront - frontOffset);

        errorD = errorP - oldErrorP;

        totalError = straightKp * errorP + Kd * errorD;
        oldErrorP = errorP;

        // Calculate PWM based on Error
        currentLeftPWM = leftBaseSpeed + totalError / 124;
        currentRightPWM = rightBaseSpeed - totalError / 124;

        // Update Motor PWM values
        setLeftPWM(currentLeftPWM);
        setRightPWM(currentRightPWM);
        if ((leftFront + rightFront) / 2 >= frontStop) {
          turn = true;
          i = 30;
          rightBaseSpeed = 240;
          angle = 0;
        }
      }
      else {
        turn = true;
        rightBaseSpeed = 240;
        angle = 0;
      }
    }
  }
  //end if (!straight)
  else {
    const int wallFrontValue = 300;
    getGres();
    gz = (float)readGyroData() * gRes - gyroBias[2];
    angle += 1 * (gz) * 0.001;
    //        if (rightFront > wallFrontValue && leftFront > wallFrontValue) {
    //            errorP = 3 * (rightFront - leftFront - frontOffset);
    //        }
    if (wallFront()) {
      if (wallLeft()) {
        // Only left wall

        //                    errorP = 20 * (angle - targetAngle) + .5 * (leftSensor - wallDist);
        errorP = .5 * (leftSensor - leftWallDist) + 3 * (rightFront - leftFront - frontOffset);
        //                    errorP = 3 * (rightFront - leftFront - frontOffset);
        errorD = errorP - oldErrorP;
      }
      else if (wallRight()) {
        // Only right wall
        //                    errorP = 20 * (angle - targetAngle) - .5 * (rightSensor - wallDist);
        //                    errorP = 3 * (rightFront - leftFront - frontOffset);
        errorP = -.5 * (rightSensor - rightWallDist) + 3 * (rightFront - leftFront - frontOffset);
        errorD = errorP - oldErrorP;
      }
      else {

        errorP = 3 * (rightFront - leftFront - frontOffset);
        //                    errorP = 20 * (angle - targetAngle);
        errorD = errorP - oldErrorP;
      }
    }
    else {
      errorP = 20 * (angle - targetAngle);
    }

    totalError = straightKp * errorP + Kd * errorD;
    oldErrorP = errorP;

    // Calculate PWM based on Error
    currentLeftPWM = leftBaseSpeed + totalError / 124;
    currentRightPWM = rightBaseSpeed - totalError / 124;

    // Update Motor PWM values
    setLeftPWM(currentLeftPWM);
    setRightPWM(currentRightPWM);
  }

  if (i >= curve2Time && !straight) {
    straight = true;

    leftValid = false;
    rightValid = false;

    if (moveType == TURN_RIGHT) {
      targetAngle = -110;
    }
    else {
      targetAngle = 110;
    }
  }

  //Detects when turn is done
  if (straight) {
    if (wallFront()) {
      const int frontStop = 500;
      if ((leftFront + rightFront) / 2 >= frontStop) {
        i = 0;
        angle = 0.0;
        oldErrorP = 0;
        rightTicks = 0;
        leftTicks = 0;
        moveType = NO;
        walls_global[0] = wallLeft();
        walls_global[1] = wallFront();
        walls_global[2] = wallRight();
        currentMoveDone = true;
        //                needMove = true;
        straight = false;
        turn = false;
      }
    }
    else {
      if ((rightTicks + leftTicks) / 2 >= targetTicks) {
        i = 0;
        angle = 0.0;
        oldErrorP = 0;
        rightTicks = 0;
        leftTicks = 0;
        moveType = NO;
        walls_global[0] = wallLeft();
        walls_global[1] = wallFront();
        walls_global[2] = wallRight();
        currentMoveDone = true;
        //                needMove = true;
        straight = false;
        turn = false;
      }
    }
  }
}

void pivotTurnRight() {
  int errorP;
  int errorD;
  int totalError;
  int oldErrorP;
  int tickCount = 190;
  // Gyro calibrated for each speed or turning is not accurate
  float degreesTraveled = 0;
  const int turnSpeed = 450;
  const float targetDegrees = 150;
  // const int turnSpeed = 45;
  // const int targetDegrees = 85.5
  // const int turnSpeed = 40;
  // const int targetDegrees = 86
  float initialZ;
  //    rightTicks = 0;
  //    leftTicks = 0;
  //    delay(200);

  //    while (rightTicks > -90 || leftTicks < 90) {
  //    }

  getGres();
  gz = (float)readGyroData() * gRes - gyroBias[2];
  initialZ = gz; // May not be necessary
  count = millis();
  setLeftPWM(turnSpeed - 10);
  setRightPWM(-turnSpeed);
  while (degreesTraveled >= -targetDegrees) {
    uint32_t deltat = millis() - count;
    if (deltat > 1) {
      getGres();
      gz = (float)readGyroData() * gRes - gyroBias[2];
      degreesTraveled += 2 * (gz - 0) * 0.001;
      count = millis();
    }
  }
  //
  //    // Needs to deccelerate for the motors to stop correctly
  for (int i = turnSpeed; i >= 0; --i) {
    setLeftPWM(i);
    setRightPWM(-i);
  }
  //    delay(200);
}

void pivotTurnRight90() {
  int errorP;
  int errorD;
  int totalError;
  int oldErrorP;
  int tickCount = 190;
  // Gyro calibrated for each speed or turning is not accurate
  float degreesTraveled = 0;
  const int turnSpeed = 450;
  const float targetDegrees = 76;
  // const int turnSpeed = 45;
  // const int targetDegrees = 85.5
  // const int turnSpeed = 40;
  // const int targetDegrees = 86
  float initialZ;
  //    rightTicks = 0;
  //    leftTicks = 0;
  //    delay(200);

  //    while (rightTicks > -90 || leftTicks < 90) {
  //    }

  getGres();
  gz = (float)readGyroData() * gRes - gyroBias[2];
  initialZ = gz; // May not be necessary
  count = millis();
  setLeftPWM(turnSpeed - 20);
  setRightPWM(-turnSpeed);
  while (degreesTraveled >= -targetDegrees) {
    uint32_t deltat = millis() - count;
    if (deltat > 1) {
      getGres();
      gz = (float)readGyroData() * gRes - gyroBias[2];
      degreesTraveled += 2 * (gz - 0) * 0.001;
      count = millis();
    }
  }
  //
  //    // Needs to deccelerate for the motors to stop correctly
  for (int i = turnSpeed; i >= 0; --i) {
    setLeftPWM(i);
    setRightPWM(-i);
  }

}

void rightTurnFirstCell() {
  const int frontLeftStop = 1900;
  const int frontRightStop = 1900;
  bool leftStop = false;
  bool rightStop = false;
  int tickCount = 180;
  int errorP;
  int errorD;
  int totalError;
  bool front;
  leftBaseSpeed = exploreSpeed;
  rightBaseSpeed = exploreSpeed;
  moveType = NO;

  //Turn Around with no wall in front

  const int tickValue = 50;
  while (leftFront < frontLeftStop && rightFront < frontRightStop) {
    // Only left wall
    // errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100 * (angle - targetAngle);
    getGres();
    gz = (float)readGyroData() * gRes - gyroBias[2];
    angle += 2 * (gz) * 0.001;
    errorP = 20 * (angle) + .5 * (leftSensor - leftWallDist);
    errorD = errorP;

    errorD = errorP;
    totalError = straightKp * errorP + Kd * errorD;

    // Calculate PWM based on Error
    currentLeftPWM = leftBaseSpeed + totalError / 124;
    currentRightPWM = rightBaseSpeed - totalError / 124;

    // Update Motor PWM values
    setLeftPWM(currentLeftPWM);
    setRightPWM(currentRightPWM);

    //TODO (this is a hack and shouldn't be here, but it makes it work)
    haveSensorReading = false;
    while (!haveSensorReading) {
      readSensors();
      delayMicroseconds(80);
    }

  }
  setRightPWM(0);
  setLeftPWM(0);
  delay(200);

  pivotTurnRight90();

  angle = 0.0;
  delay(200);
  setLeftPWM(-150);
  setRightPWM(-150);
  delay(350);
  for (int i = -150; i < 0; ++i) {
    setLeftPWM(i);
    setRightPWM(i);
  }
  delay(200);
  firstCell = true;
  movesBuffer[0] = 'f';

}

void solve() {
  while (!movesDoneAndWallsSet) {
  }
  bool walls[3];
  walls[0] = walls_global[0];
  walls[1] = walls_global[1];
  walls[2] = walls_global[2];
  movesDoneAndWallsSet = false;

    movesBuffer[0] = 'f';
    movesBuffer[1] = 'f';
    movesBuffer[2] = 'f';
    movesBuffer[3] = 'f';
    movesBuffer[4] = 'f';
    movesBuffer[5] = 'r';
    movesBuffer[6] = 'f';
    movesBuffer[7] = 'a';
    movesBuffer[8] = 'f';
    movesBuffer[9] = 'f';
    movesBuffer[10] = 'l';
    movesBuffer[11] = 'f';
    movesBuffer[12] = 'f';
    movesBuffer[13] = 'f';
    movesBuffer[14] = 'f';
    movesBuffer[15] = 'a';
    movesBuffer[16] = 0;
  /*movesBuffer[0] = 'f';
    movesBuffer[1] = 'f';
    movesBuffer[2] = 'f';
    movesBuffer[3] = 'f';
    movesBuffer[4] = 'f';
    movesBuffer[5] = 'r';
    movesBuffer[6] = 'f';
    movesBuffer[7] = 'r';
    movesBuffer[8] = 'r';
    movesBuffer[9] = 'l';
    movesBuffer[10] = 'f';
    movesBuffer[11] = 'f';
    movesBuffer[12] = 'f';
    movesBuffer[13] = 'l';
    movesBuffer[14] = 'f';
    movesBuffer[15] = 'l';
    movesBuffer[16] = 'l';
    movesBuffer[17] = 'r';
    movesBuffer[18] = 'r';
    movesBuffer[19] = 'l';
    movesBuffer[20] = 'f';
    movesBuffer[21] = 'f';
    movesBuffer[22] = 'l';
    movesBuffer[23] = 'f';
    movesBuffer[24] = 'f';
    movesBuffer[25] = 'l';
    movesBuffer[26] = 'f';
    movesBuffer[27] = 'f';
    movesBuffer[28] = 'f';
    movesBuffer[29] = 'f';
    movesBuffer[30] = 'a';
    movesBuffer[31] = 0;
    */

//  if (!walls[0]) {
//    movesBuffer[0] = 'l';
//    movesBuffer[1] = 0;
//  }
//  else if (!walls[1]) {
//    movesBuffer[0] = 'f';
//    movesBuffer[1] = 0;
//  }
//  else if (!walls[2]) {
//    movesBuffer[0] = 'r';
//    movesBuffer[1] = 0;
//  }
//  else {
//    movesBuffer[0] = 'a';
//    movesBuffer[1] = 'f';
//    movesBuffer[2] = 0;
//  }


  movesReady = true;
}

/*void bluetoothPrint() {
  while (!haveSensorReading) {
  }
  int i = 0;
  while (i != 4 || bluetoothBuffer[i] != 0) {
    mySerial.print(bluetoothBuffer[i++]);
  }
  mySerial.println();
}*/

