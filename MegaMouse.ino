// Teensy 3 I2C Library
#include <LedDisplay.h>
#include "Motors.h"
#include "Sensors.h"
#include "Profiles.h"
#include "MackAlgo.h"
mack::MackAlgo algo;

#define dataPin 11              // connects to the display's data in
#define registerSelect 12       // the display's register select pin 
#define clockPin 13             // the display's clock pin
#define enable 10               // the display's chip enable pin
#define reset 3                 // the display's reset pin
#define displayLength 4        // number of characters in the display

// create am instance of the LED display library:
LedDisplay myDisplay = LedDisplay(dataPin, registerSelect, clockPin, enable, reset, displayLength);
int brightness = 15;        // screen brightness


//Thresholds for left and right sensors detecting side walls
#define hasLeftWall 500
#define hasRightWall 500 //used to be 800

//Seperate speeds for explore and solve (not currently implemented)
int exploreSpeed = 200;
int solveSpeed = 200;

int leftBaseSpeed = exploreSpeed;
int rightBaseSpeed = exploreSpeed+30;

//Setpoint for left and right sensors detecting side walls
const int rightWallDist = 1665;
const int leftWallDist = 1565;

// PID Constants
#define straightKp 3
#define turnKp 16
#define Kd 10

/* Variables for interface between drive code and algorithm */
volatile char movesBuffer[256];
char bluetoothBuffer[5];
volatile bool walls_global[3] = {false, false, false}; // Left, Front, Right
volatile bool movesReady = false; // Set to true by algorithm, set to false by drive.
volatile bool movesDoneAndWallsSet = false; // Set to true by drive, set to false by algorithm.
/* End of variables for interface */

//Max speed for acceleration
const int maxSpeed = 500;

bool currentMoveDone = false;
bool firstMove = true;
bool accelerate = true;
bool solving = 1;
int goalSpeed = 0;

volatile bool firstCell = true;
volatile bool afterTurnAround = false;

//Walls currently on left or right
volatile bool rightValid = true;
volatile bool leftValid = true;

volatile bool haveSensorReading = false;

IntervalTimer correctionTimer;
IntervalTimer sensorTimer;
IntervalTimer refreshSensorTimer;
//SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);

//Current angle of the robot
volatile float angle = 0.0;

//Different move types
volatile enum {
  NO = 0,
  FORWARD = 1,
  TURN_LEFT = 2,
  TURN_RIGHT = 3,
  TURN_AROUND = 4
} moveType;

const int buttonPin = 16;

int sensorCounts = 0;

void setup() {
  //Serial.begin(9600);
  Serial.begin(115200);
  //    Serial.begin(115200);
  myDisplay.begin();
  // set the brightness of the display:
  myDisplay.setBrightness(brightness);
  myDisplay.setCursor(0);
  myDisplay.println("Mega");
  // 12 bit ADC resolution
  analogReadResolution(12);

  setupMotors();
  setupSensors();


  //pinMode(buttonPin, INPUT_PULLUP);

  // Wait for Button Press to Start
  //  while (digitalRead(buttonPin) == 1) {
  //  }//TODO Solder Button
  //delay(3000);
  //Start by placing hand in front of sensor (too sensitive)
  //readSensors();
  //while (rightFront < 3300 && rightMiddleValue < 3300 && rightSensor < 3300) {
  //        readSensors();
  //        Serial.println(leftSensor);
  //}
  moveType = NO; //TODO just added 3/1/16
  delay(3000);
  delay(1000);
  myDisplay.setCursor(0);
  myDisplay.clear();
  myDisplay.print("3");
  delay(1000);
  myDisplay.clear();
  myDisplay.setCursor(1);
  myDisplay.print("2");
  delay(1000);
  myDisplay.clear();
  myDisplay.setCursor(2);
  myDisplay.print("1");
  delay(1000);
  myDisplay.clear();
  //Reads a different sensor every 80us
  sensorTimer.priority(250);
  sensorTimer.begin(readSensors, 80);
  while (!haveSensorReading) {
  }

  //read initial sensor values to determine first cell move
  walls_global[0] = wallLeft();
  walls_global[1] = wallFront();
  walls_global[2] = wallRight();

  haveSensorReading = false;
  movesDoneAndWallsSet = true;

  //Runs every 1ms and controls mouse movements
  correctionTimer.priority(255);
  correctionTimer.begin(correction, 1000);

}

void loop() {
  //Solve the maze
  //algo.solve();
  //start to 1st cell: 260
  //1 cell: 323
//  myDisplay.clear();
//  myDisplay.setCursor(0);
 // myDisplay.print((rightTicks + leftTicks) / 2);
  //myDisplay.print(rightMiddleValue);//180 no wall, 820 wall
//  myDisplay.print(leftMiddleValue);// 300 no wall, 700 wall
//  myDisplay.print((leftFront + rightFront) / 2);
//  delay(50);
  solve();
}

//Turns wheels and prints encoder ticks to check difference in speed
void wheelCalib() {
  //  correctionTimer.end();
  //  sensorTimer.end();
  //  refreshSensorTimer.end();
  setLeftPWM(240);
  setRightPWM(240);
  while (1) {
    delay(1000);
    Serial.print(leftTicks);
    Serial.print(" ");
    Serial.println(rightTicks);
    leftTicks = 0;
    rightTicks = 0;
  }
}

//1ms timer
void correction() {
  static int totalForwardCount = 0;
  static int forwardCount = 0;
  static bool in_acceleration = false;
  static byte indexInBuffer = 0;
  static bool movedForward = false;

  readGyro();
  if (!movesReady) {
    // Hoping we never get here, but maybe the algorithm is slow.
    haveSensorReading = false;
    return;
  }

  if (currentMoveDone) {
    movedForward = false;
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
          goalSpeed = (int)solveSpeed * (float)forwardCount;
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
          goalSpeed = 200;
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
      currentMoveDone = true;
      haveSensorReading = false;
      correctionTimer.priority(255);
      correctionTimer.begin(correction, 1000);
      return;
    default:
      moveType = NO;

      // Don't need to do anything here if we're turning around.
  }

  haveSensorReading = false;
}

void moveForward() {
  myDisplay.setCursor(0);
  myDisplay.clear();
  myDisplay.print("Fd");
  if (firstCell) {
    rightTicks = 65;//70
    leftTicks = 65;//70
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

  rightValid = wallRight();
  leftValid = wallLeft();
  moveType = FORWARD;
}

void turnAround() {
  const int frontLeftStop = 500;
  const int frontRightStop = 500;
  bool leftStop = false;
  bool rightStop = false;
  int tickCount = 180;
  int errorP;
  int errorD;
  int totalError;
  bool front;
  leftBaseSpeed = 200;
  rightBaseSpeed = 200;
  moveType = NO;
  if (wallFront()) {
    front = true;
  }
  else {
    afterTurnAround = true;
    front = false;
  }
  if (front) {
    while (rightFront <= frontRightStop || leftFront <= frontLeftStop) {
      if (wallRight() && wallLeft()) {
        errorP = leftSensor - rightSensor + 100; // 100 is the offset between left and right sensor when mouse in the
        // middle of cell
      }
      else if (wallRight()) {
        const int wallDist = rightWallDist; //Make this bigger to move closer to the wall
        // Only right wall
        //read Gyro? TODO
        errorP = 1 * (angle);// - .5 * (rightSensor - wallDist);
        errorD = errorP;
      }
      else if (wallLeft()) {
        const int wallDist = leftWallDist; // Make this bigger to move closer to the wall
        // Only left wall
        // errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100 * (angle - targetAngle);
        //read Gyro? TODO
        errorP = 1 * (angle);// + .5 * (leftSensor - wallDist);
        errorD = errorP;
      }
      else {
        //read Gyro? TODO
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
        errorP = leftSensor - rightSensor + 100; // 100 is the offset between left and right sensor when mouse in the
        // middle of cell
      }
      else if (wallRight()) {
        // Only right wall
        //read Gyro? TODO
        errorP = 20 * (angle) - .5 * (rightSensor - rightWallDist);
        errorD = errorP;
      }
      else if (wallLeft()) {
        // Only left wall
        // errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100 * (angle - targetAngle);
        //read Gyro? TODO
        errorP = 20 * (angle) + .5 * (leftSensor - leftWallDist);
        errorD = errorP;
      }
      else {
        //read Gyro? TODO
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

  pivotTurnRight90();
  delay(300);
  pivotTurnRight90();

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

}

void forwardCorrection() {
  const int oneCellTicks = 327;//327
  const int noWallRight = 250; // check this value (250)
  const int noWallLeft = 85; // check this value (450)

  const int pegWallBack = 800; // check this value
  const int pegNoWalls = 1000;
  const int pegWallFront = 1000;

  const int wallBackTicks = 232;
  const int noWallTicks = 209;
  const int frontWallTicks = 204;

  // encoder tick value when we check walls a cell ahead
  const int readingTicks = 180; // check this value (175)
  // encoder tick value when we switch to next cell's values
  const int newSideTicks = 230; // check this value (200)

  static bool nextRightValid;
  static bool nextLeftValid;
  static bool nextCellDecided = false;
  int errorP;
  int errorD;
  int oldErrorP = 0;
  int totalError;
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
  }

  if ((rightTicks + leftTicks) / 2 >= newSideTicks) {
    leftValid = nextLeftValid;
    rightValid = nextRightValid;
  }

  if (leftValid && rightValid) {
//    myDisplay.setCursor(0);
//    myDisplay.print("lVrV");
    //angle = 0.0;
    // Has both wall, so error correct with both (working, just need to adjust PD constants when final mouse is built)
    angle = 0;//TODO Not sure about this
    errorP = 2 * (leftSensor - rightSensor + 100) + 75 * (rightTicks - leftTicks); // 100 is the offset between left and right sensor when mouse in the
    // middle of cell
    errorD = errorP - oldErrorP;
    //        getGres();
    //        gz = (float)readGyroData() * gRes - gyroBias[2];
    //        straightAngle += 2 * (gz) * 0.001;
  }
  else if (leftValid) {
    // Only left wall
    // errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100 * (angle - targetAngle);
    // TODO - Walls
    errorP = 75 * (rightTicks - leftTicks) + 20 * (-angle) ;//+ .5 * (leftSensor - leftWallDist);
    errorD = errorP - oldErrorP;
  }
  else if (rightValid) {
    // Only right wall
    // TODO - Walls
    errorP = 75 * (rightTicks - leftTicks) + 20 * (-angle);// - .5 * (rightSensor - rightWallDist);
    errorD = errorP - oldErrorP;
  }
  else {
    static int targetAngle = 0;
    // No walls, use gyro to correct
    //read Gyro? TODO
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
    errorP = 75 * (rightTicks - leftTicks);
    //errorP = 20 * (targetAngle - angle);
    //errorP = 150 * (rightTicks - leftTicks); //+ 20 * (targetAngle - angle);

    //    }
    errorD = errorP - oldErrorP;
  }

  //Peg Correction
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
  currentRightPWM = rightBaseSpeed - totalError / 124;//TODO 240 should be left/right base speeds

  // Update Motor PWM values
  setLeftPWM(currentLeftPWM);
  setRightPWM(currentRightPWM);
  //  myDisplay.clear();
  //  myDisplay.setCursor(0);
  //  myDisplay.print(leftBaseSpeed);
}

//Curve Turn
//void turnCorrection() {
//  int errorP;
//  int errorD = 0;
//  int oldErrorP = 0;
//  int totalError;
//  static float targetAngle;
//  static int i = 0;
//  static bool turn = false;
//  static bool straight = 0;
//  const int targetTicks = 140;
//  const int frontOffset = 0; // difference between left and right front sensors when lined up with the wall
//
//  if (!straight) {
//    if (turn) {
//      targetAngle = curve[i];
//      //read Gyro? TODO
//
//      //Smaller value = overturn
//      if (moveType == TURN_RIGHT) {
//        //angle += 1.375 * (gz) * 0.001;
//        errorP = 30 * (angle - targetAngle/2);
//      }
//      else {
//        //angle += 1.375 * (gz) * 0.001;
//        errorP = 30 * (angle + targetAngle/2);
//      }
//
//
//      errorD = errorP - oldErrorP;
//
//      rightTicks = 0;
//      leftTicks = 0;
//      totalError = turnKp * errorP + Kd * errorD;
//      oldErrorP = errorP;
//
//      // Calculate PWM based on Error
//      currentLeftPWM = leftBaseSpeed + totalError / 124;
//      currentRightPWM = rightBaseSpeed - totalError / 124;
//
//      // Update Motor PWM values
//      setLeftPWM(currentLeftPWM);
//      setRightPWM(currentRightPWM);
//      ++i;
//    }
//    //end if (turn)
//    else {
//      const int frontStop = 650;
//      if (wallFront()) {
//        if (wallLeft()) {
//          // Only left wall
//          // errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100 * (angle - targetAngle);
//          //read Gyro? TODO
//          //                    errorP = 20 * (angle - targetAngle) + .5 * (leftSensor - wallDist);
//          errorP = .5 * (leftSensor - leftWallDist) + 3 * (rightFront - leftFront - frontOffset);
////            errorP = .5 * (leftSensor - leftWallDist);
//          //                    errorP = 3 * (rightFront - leftFront - frontOffset);
//          errorD = errorP - oldErrorP;
//        }
//        else if (wallRight()) {
//          // Only right wall
//          //read Gyro? TODO
//          //                    errorP = 20 * (angle - targetAngle) - .5 * (rightSensor - wallDist);
//          //                    errorP = 3 * (rightFront - leftFront - frontOffset);
//          errorP = -.5 * (rightSensor - rightWallDist) + 3 * (rightFront - leftFront - frontOffset);
//          errorD = errorP - oldErrorP;
//        }
//        else {
//          //read Gyro? TODO
//          errorP = 3 * (rightFront - leftFront - frontOffset);
//          //                    errorP = 20 * (angle - targetAngle);
//          errorD = errorP - oldErrorP;
//        }
//        //                getGres();
//        //                gz = (float)readGyroData() * gRes - gyroBias[2];
//        //                angle += 1 * (gz) * 0.001;
//        errorP = 3 * (rightFront - leftFront - frontOffset);
//
//        errorD = errorP - oldErrorP;
//
//        totalError = straightKp * errorP + Kd * errorD;
//        oldErrorP = errorP;
//
//        // Calculate PWM based on Error
//        currentLeftPWM = leftBaseSpeed + totalError / 124;
//        currentRightPWM = rightBaseSpeed - totalError / 124;
//
//        // Update Motor PWM values
//        setLeftPWM(currentLeftPWM);
//        setRightPWM(currentRightPWM);
//        if ((leftFront + rightFront) / 2 >= frontStop) {
//          turn = true;
//          i = 30;
//          rightBaseSpeed = 200;
//          angle = 0;
//        }
//      }
//      else {
//        turn = true;
//        rightBaseSpeed = 200;
//        angle = 0;
//      }
//    }
//  }
//  //end if (!straight)
//  else {
//    const int wallFrontValue = 300;
//    //read Gyro? TODO
//    //        if (rightFront > wallFrontValue && leftFront > wallFrontValue) {
//    //            errorP = 3 * (rightFront - leftFront - frontOffset);
//    //        }
//    if (wallFront()) {
//      if (wallLeft()) {
//        // Only left wall
//
//        //                    errorP = 20 * (angle - targetAngle) + .5 * (leftSensor - wallDist);
//        errorP = .5 * (leftSensor - leftWallDist) + 3 * (rightFront - leftFront - frontOffset);
//        //                    errorP = 3 * (rightFront - leftFront - frontOffset);
//        errorD = errorP - oldErrorP;
//      }
//      else if (wallRight()) {
//        // Only right wall
//        //                    errorP = 20 * (angle - targetAngle) - .5 * (rightSensor - wallDist);
//        //                    errorP = 3 * (rightFront - leftFront - frontOffset);
//        errorP = -.5 * (rightSensor - rightWallDist) + 3 * (rightFront - leftFront - frontOffset);
//        errorD = errorP - oldErrorP;
//      }
//      else {
//
//        errorP = 3 * (rightFront - leftFront - frontOffset);
//        //                    errorP = 20 * (angle - targetAngle);
//        errorD = errorP - oldErrorP;
//      }
//    }
//    else {
//      errorP = 20 * (angle - targetAngle);
//    }
//
//    totalError = straightKp * errorP + Kd * errorD;
//    oldErrorP = errorP;
//
//    // Calculate PWM based on Error
//    currentLeftPWM = leftBaseSpeed + totalError / 124;
//    currentRightPWM = rightBaseSpeed - totalError / 124;
//
//    // Update Motor PWM values
//    setLeftPWM(currentLeftPWM);
//    setRightPWM(currentRightPWM);
//  }
//
//  if (i >= curveTime && !straight) {
//    straight = true;
//
//    leftValid = false;
//    rightValid = false;
//
//    if (moveType == TURN_RIGHT) {
//      targetAngle = 90;
//    }
//    else {
//      targetAngle = -90;
//    }
//  }
//
//  //Detects when turn is done
//  if (straight) {
//    if (wallFront()) {
//      const int frontStop = 500;
//      if ((leftFront + rightFront) / 2 >= frontStop) {
//        i = 0;
//        angle = 0.0;
//        oldErrorP = 0;
//        rightTicks = 0;
//        leftTicks = 0;
//        moveType = NO;
//        walls_global[0] = wallLeft();
//        walls_global[1] = wallFront();
//        walls_global[2] = wallRight();
//        currentMoveDone = true;
//        //                needMove = true;
//        straight = false;
//        turn = false;
//      }
//    }
//    else {
//      if ((rightTicks + leftTicks) / 2 >= targetTicks) {
//        i = 0;
//        angle = 0.0;
//        oldErrorP = 0;
//        rightTicks = 0;
//        leftTicks = 0;
//        moveType = NO;
//        walls_global[0] = wallLeft();
//        walls_global[1] = wallFront();
//        walls_global[2] = wallRight();
//        currentMoveDone = true;
//        //                needMove = true;
//        straight = false;
//        turn = false;
//      }
//    }
//  }
//}

void turnCorrection() {
  double errorP;
  static double oldErrorP;
  double errorD;
  double totalError;
  double Kp = 10;
  //double Kd = 10;
  double targetAngle;
  //long long start = millis();
  //int timeConst = 2 * turnSpeed; //ms 2
  static int i = 0;
  static bool continueTurn = true;
  static bool turn = false;
  static bool straight = false;
  
  if (i == 0) {
    myDisplay.setCursor(0);
    myDisplay.clear();
    if (moveType == TURN_RIGHT) {
      myDisplay.print("RGHT");
    }
    else {
      myDisplay.print("LEFT");
    }
    if (!wallFront()) {
      myDisplay.setCursor(0);
      myDisplay.clear();
      myDisplay.print("NoWF");
      turn = true;
    }
  }
  if (turn == false) {
    targetAngle = 0;
    if ((rightFront + leftFront) / 2 >= 99) {//TODO doesn't account for turns without wallFront
      turn = true;
      i = 1;
    }
    //turn = true;
  }
  else {
    if (moveType == TURN_RIGHT) {
      targetAngle = curve4[i];
    }
    else {
      targetAngle = -curve4[i];
    }
  }

  if (straight == false && abs(targetAngle) == 90) {
    rightTicks = 0;
    leftTicks = 0;
    straight = true;
  }
  if (straight == true) {
    if (moveType == TURN_RIGHT) {
      targetAngle = 90;
    }
    else {
      targetAngle = -90;
    }
    if ((rightTicks + leftTicks) / 2 > 60 || (leftFront + rightFront) / 2 > 99) {//TODO make wallfront variable (this number appears in 2 places in this function)
      continueTurn = false;
    }
  }


  errorP = targetAngle - angle;
  errorD = errorP - oldErrorP;
  totalError = 18 * errorP + 10 * errorD;
  // Calculate PWM based on Error
    currentLeftPWM = leftBaseSpeed + int(totalError);
    currentRightPWM = rightBaseSpeed - int(totalError);
//    myDisplay.setCursor(0);
//    myDisplay.clear();
//    myDisplay.print(totalError);
//    // Update Motor PWM values
    setLeftPWM(currentLeftPWM);
    setRightPWM(currentRightPWM);
    
  if (i == 50) {
    myDisplay.clear();
  }
  if (continueTurn) {
    if (!straight) {
      i++;
    }
  }
  else {
    i = 0;
    angle = 0.0;
    rightTicks = 0;
    leftTicks = 0;
    moveType = NO;
    walls_global[0] = wallLeft();
    walls_global[1] = wallFront();
    walls_global[2] = wallRight();
    currentMoveDone = true;
    //needMove = true;
    //straight = false;
    turn = false;
    straight = false;
    continueTurn = true;
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
  const int turnSpeed = 400;
  const float targetDegrees = 137;
  float initialZ;

  //read Gyro? TODO
  //initialZ = gz; // May not be necessary
  //  count = millis();
  //  setLeftPWM(turnSpeed - 10);
  //  setRightPWM(-turnSpeed);
  //  while (degreesTraveled >= -targetDegrees) {
  //      //read Gyro? TODO
  //      count = millis();
  //      delay(1);
  //  }

  // Needs to deccelerate for the motors to stop correctly
  for (int i = turnSpeed; i >= 0; i -= 50) {
    setLeftPWM(i);
    setRightPWM(-i);
  }
  delay(200);
}

void pivotTurnRight90() {
  int errorP;
  int errorD;
  int totalError;
  int oldErrorP;
  int tickCount = 190;
  int finishedCount = 0;
  // Gyro calibrated for each speed or turning is not accurate
  float degreesTraveled = 0;
  const int turnSpeed = 350;
  const float targetDegrees = 90;
  float targetAngle = 0;
  // const int turnSpeed = 45;
  // const int targetDegrees = 85.5
  // const int turnSpeed = 40;
  // const int targetDegrees = 86
  float initialZ;
  long count = 0;
  //    rightTicks = 0;
  //    leftTicks = 0;
  //    delay(200);

  //    while (rightTicks > -90 || leftTicks < 90) {
  //    }

  angle = 0;
  degreesTraveled = 0;
  count = micros();
  while (targetAngle <= targetDegrees) {
    uint32_t deltat = micros() - count;
    if (deltat > 1000) {
      if (targetAngle < 90) {
        targetAngle += 0.25;
      }
      else {
        finishedCount++;
      }
      readGyro();
      errorP = angle - targetAngle;
      errorD = errorP - oldErrorP;
      totalError = 30 * errorP + 20 * errorD;
      setLeftPWM(0 - totalError);
      setRightPWM(0 + totalError);
      oldErrorP = errorP;
      count = micros();    
      if (finishedCount > 100) {
        break;
        setLeftPWM(0);
        setRightPWM(0);
      }
    }
  }
  //    // Needs to deccelerate for the motors to stop correctly
//  for (int i = turnSpeed; i >= 0; --i) {
//    setLeftPWM(i);
//    setRightPWM(-i);
//  }
}

// If mouse starts facing a wall, this turns in the first cell to face the opening
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
    //read Gyro? TODO
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
  movesBuffer[2] = 'r';
  movesBuffer[3] = 'r';
  movesBuffer[4] = 'f';
  movesBuffer[5] = 'l';
  movesBuffer[6] = 'l';
  movesBuffer[7] = 'f';
  movesBuffer[8] = 'a';
  //    movesBuffer[11] = 'f';
  //    movesBuffer[12] = 'f';
  //    movesBuffer[13] = 'f';
  //    movesBuffer[14] = 'f';
  //    movesBuffer[15] = 'a';
  //    movesBuffer[16] = 0;
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
  //    myDisplay.setCursor(0);
  //    myDisplay.println("left");
  //    movesBuffer[0] = 'l';
  //    movesBuffer[1] = 0;
  //  }
  //  else if (!walls[1]) {
  //    myDisplay.setCursor(0);
  //    myDisplay.println("frwd");
  //    movesBuffer[0] = 'f';
  //    movesBuffer[1] = 0;
  //  }
  //  else if (!walls[2]) {
  //    myDisplay.setCursor(0);
  //    myDisplay.println("rght");
  //    movesBuffer[0] = 'r';
  //    movesBuffer[1] = 0;
  //  }
  //  else {
  //    myDisplay.setCursor(0);
  //    myDisplay.println("arnd");
  //    movesBuffer[0] = 'a';
  //    movesBuffer[1] = 'f';
  //    movesBuffer[2] = 0;
  //  }

  movesReady = true;
}

