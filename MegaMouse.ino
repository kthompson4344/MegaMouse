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
#define hasLeftWall 220
#define hasRightWall 220

//Seperate speeds for explore and solve (mm/s) (not currently implemented)
int exploreSpeed = 400;
int solveSpeed = 1000;

int leftBaseSpeed = exploreSpeed;
int rightBaseSpeed = exploreSpeed;

float leftSpeed;
float rightSpeed;

//Setpoint for left and right sensors detecting side walls
const int rightWallDist = 1150;
const int leftWallDist = 1100;

const float frontStop = 3.7;//3.8
//float gyroZeroVoltage = 1.55;
// PID Constants
#define straightKp 9.2
#define Kd 0

/* Variables for interface between drive code and algorithm */
volatile char movesBuffer[256];
char bluetoothBuffer[5];
volatile bool walls_global[3] = {false, false, false}; // Left, Front, Right
volatile bool movesReady = false; // Set to true by algorithm, set to false by drive.
volatile bool movesDoneAndWallsSet = false; // Set to true by drive, set to false by algorithm.
/* End of variables for interface */

//Max speed for acceleration
const int maxSpeed = 400;

const int maxPWM = 1000;

bool currentMoveDone = false;
bool firstMove = true;
bool accelerate = true;
bool solving = 0;
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
  //Serial.begin(115200);
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
  //  setLeftPWM(0);
  //  setRightPWM(0);
  rightTicks = 0;
  leftTicks = 0;
}

void loop() {
  //Solve the maze
  //algo.solve();
  //start to 1st cell: 260
  //1 cell: 323
  //        myDisplay.clear();
  //        myDisplay.setCursor(0);
  //      myDisplay.print((rightTicks + leftTicks) / 2);
  //   myDisplay.print((rightTicks + leftTicks) / 2);
  //  myDisplay.print(rightMiddleValue);//180 no wall, 820 wall
  //  myDisplay.print(leftMiddleValue);// 300 no wall, 700 wall
  //      myDisplay.print(leftFrontRaw);
  //      myDisplay.print((leftFrontRaw + rightFrontRaw) / 2);
  //  myDisplay.print(rightFront);
  //      myDisplay.print(1 * (rightFrontRaw - leftFrontRaw  +300));
  //myDisplay.print(analogRead(A19) - analogRead(A13));
  //      delay(50);
//      solve();
  algo.solve();
  //  setLeftPWM(int(.5*(2000 - leftFrontRaw)));
  //  setRightPWM(int(.5*(2000-rightFrontRaw)));
  //  delay(1);
  //setLeftPWM(-200);
  //setRightPWM(200);
  //delay(5);
}

void getSpeed() {
  const int timeConst = 10;//ms
  static int count = 0;
  // 0.55mm/tick (16count TODO double check)
  if (moveType == FORWARD) {
    float avgSpeed = (leftSpeed + rightSpeed) / 2.0;
    if (count >= timeConst) {
      leftSpeed = (leftTicks - prevLeftTicks) * 0.55 / (timeConst / 1000.0);
      rightSpeed = (rightTicks - prevRightTicks) * 0.55 / (timeConst / 1000.0);
      //      float avgSpeed = (leftSpeed + rightSpeed) / 2.0;
      if (leftSpeed >= 0 && rightSpeed >= 0) {
        if (goalSpeed > 0) {
          if (avgSpeed < goalSpeed) {
            leftBaseSpeed += 2;
            rightBaseSpeed += 2;
          } else if (avgSpeed > goalSpeed) {
            leftBaseSpeed --;
            rightBaseSpeed --;
          } else {
            //      count = 0;
          }
        }
      }
      //      myDisplay.clear();
      //      myDisplay.setCursor(0);
      //      myDisplay.print(rightBaseSpeed);
      prevLeftTicks = leftTicks;
      prevRightTicks = rightTicks;
      count = 0;
    }
    if (leftBaseSpeed < 105 && goalSpeed > 0) {

      leftBaseSpeed++;
      rightBaseSpeed++;
    }
    //    if (!firstCell && !afterTurnAround && accelerate && goalSpeed - avgSpeed > 50) {
    //      leftBaseSpeed++;
    //      rightBaseSpeed++;
    //    }
    if (goalSpeed == 0 && leftBaseSpeed > 0) {
      leftBaseSpeed -= 2;
      rightBaseSpeed -= 2;
    }
    if (accelerate && avgSpeed - goalSpeed > 50) {
      leftBaseSpeed -= .5;
      rightBaseSpeed -= .5;
    }
    if (goalSpeed == 0) {
      if (leftBaseSpeed < 0) {
        leftBaseSpeed = 0;
      }
      if (rightBaseSpeed < 0) {
        rightBaseSpeed = 0;
      }
    }

  }
  //TODO: Decelleartion
  count++;
  //  }
}

//1ms timer
void correction() {
  static int totalForwardCount = 0;
  static int forwardCount = 0;
  static bool in_acceleration = false;
  static byte indexInBuffer = 0;
  static bool movedForward = false;

  getSpeed();
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
          goalSpeed = (int)exploreSpeed * (int)forwardCount;
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
          goalSpeed = exploreSpeed;//TODO
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
        sensorTimer.end();
        leftTicks = 0;
        rightTicks = 0;
        rightTurnFirstCell();
        correctionTimer.priority(255);
        walls_global[0] = wallLeft();
        walls_global[1] = wallFront();
        walls_global[2] = wallRight();
        haveSensorReading = false;
        correctionTimer.begin(correction, 1000);
        sensorTimer.begin(readSensors, 80);
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
      sensorTimer.end();
      leftTicks = 0;
      rightTicks = 0;
      turnAround();
      currentMoveDone = true;
      haveSensorReading = false;
      correctionTimer.priority(255);
      correctionTimer.begin(correction, 1000);
      sensorTimer.begin(readSensors, 80);
      return;
    default:
      moveType = NO;

      // Don't need to do anything here if we're turning around.
  }

  haveSensorReading = false;
}

void moveForward() {
  //  myDisplay.setCursor(0);
  //  myDisplay.clear();
  //  myDisplay.print("Fd");
  if (firstCell) {
    rightTicks = 70;//70
    leftTicks = 70; //70
    firstCell = false;
    leftBaseSpeed = 0;
    rightBaseSpeed = 0;
    accelerate = true;
  }
  else if (afterTurnAround) {
    leftBaseSpeed = 0;
    rightBaseSpeed = 0;
    accelerate = true;
    rightTicks = 150;//140
    leftTicks = 150;//140
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
  bool stop = false;
  int tickCount = 180;
  int errorP;
  int errorD;
  int totalError;
  bool front;
  //  gyroZeroVoltage = 1.56;
  //  leftBaseSpeed = 200;
  //  rightBaseSpeed = 200;
  if (wallFront()) {
    front = true;
    afterTurnAround = true;
  }
  else {
    afterTurnAround = true;
    front = false;
  }
  if (front) {
    while (1) {
//      refreshSensor();
      getSpeed();
      readGyro();
      const float frontValue = 3.5;
      if ((leftFront + rightFront) / 2 > frontValue && stop == false) {
        stop = true;
        goalSpeed = 0;//TODO DO THE SAME FOR NO WALL
      }
      if (stop == true) {
        if (rightBaseSpeed > 00) {
          rightBaseSpeed--;
          leftBaseSpeed--;
        }
        else {
          leftBaseSpeed = 0;
          rightBaseSpeed = 0;
          setLeftPWM(00);
          setRightPWM(00);
          delay(200);
          break;
        }
      }
      if (wallRight() && wallLeft()) {
        errorP = 1 * (leftSensor - rightSensor + (leftWallDist - rightWallDist)) + 50 * (rightTicks - leftTicks);
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
      currentLeftPWM = leftBaseSpeed + (totalError / 124);
      currentRightPWM = rightBaseSpeed - (totalError / 124);

      if (currentLeftPWM < 0) {
        currentLeftPWM = 0;
      }
      if (currentRightPWM < 0) {
        currentRightPWM = 0;
      }
      // Update Motor PWM values
      setLeftPWM(currentLeftPWM);
      setRightPWM(currentRightPWM);

      //TODO (this is a hack and shouldn't be here, but it makes it work)
      haveSensorReading = false;
      while (!haveSensorReading) {
        readSensors();
        delayMicroseconds(80);
      }
      delayMicroseconds(500);
    }

  }

  //Turn Around with no wall in front
  else {
    while (1) {
//      refreshSensor();
      getSpeed();
      readGyro();
      const int tickValue = 00;
      if ((leftTicks + rightTicks) / 2 > tickValue && stop == false) {

        stop = true;
      }
      if (stop == true) {
        if (rightBaseSpeed > 30) {
          leftBaseSpeed--;
          rightBaseSpeed--;
        }
        else {
          leftBaseSpeed = 0;
          rightBaseSpeed = 0;
          setLeftPWM(00);
          setRightPWM(00);
          delay(200);
          break;
        }
      }
      if (wallRight() && wallLeft()) {
        errorP = 1 * (leftSensor - rightSensor + (leftWallDist - rightWallDist)); // 100 is the offset between left and right sensor when mouse in the
        // middle of cell
      }
      else if (wallRight()) {
        // Only right wall
        //read Gyro? TODO
        errorP = 1 * (angle) - .5 * (rightSensor - rightWallDist);
        errorD = errorP;
      }
      else if (wallLeft()) {
        // Only left wall
        // errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100 * (angle - targetAngle);
        //read Gyro? TODO
        errorP =1 * (angle) + .5 * (leftSensor - leftWallDist);
        errorD = errorP;
      }
      else {
        //read Gyro? TODO
        errorP = 1 * (angle);
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
  rightTicks = 0;
  leftTicks = 0;
  //  delay(5000);
  stop = false;
  leftBaseSpeed = 200;//TODO
  rightBaseSpeed = 200;
  moveType = NO;
  //  myDisplay.clear();
  //  myDisplay.setCursor(0);
  int i = 0;
  delayMicroseconds(80);
  // Read right front sensor, then turn it off.
  rightFrontRaw = analogRead(RX[rf]);
  rightFront = log(rightFrontRaw);
  digitalWriteFast(TX[rf], LOW);
  delayMicroseconds(80);
  if (wallFront()) {
    while (i < 300) {
      //TODO (this is a hack and shouldn't be here, but it makes it work)
      haveSensorReading = false;
      while (!haveSensorReading) {
        readSensors();
        delayMicroseconds(80);
      }
      if (rightFrontRaw < 100) {
        break;
      }
      else {
        setLeftPWM(int(.2 * (3300 - leftFrontRaw)));
        setRightPWM(int(.2 * (3300 - rightFrontRaw)));
      }
      i++;
      delay(1);
    }
    setLeftPWM(0);
    setRightPWM(0);
  }
  pivotTurnRight90();
  //  myDisplay.print("Done");

  //use only for pivotTurn90
  i = 0;
  digitalWriteFast(TX[rf], HIGH);
  delayMicroseconds(80);
  // Read right front sensor, then turn it off.
  rightFrontRaw = analogRead(RX[rf]);
  rightFront = log(rightFrontRaw);
  digitalWriteFast(TX[rf], LOW);
  delayMicroseconds(80);
  if (wallFront()) {
    while (i < 350) {
      //TODO (this is a hack and shouldn't be here, but it makes it work)
      haveSensorReading = false;
      while (!haveSensorReading) {
        readSensors();
        delayMicroseconds(80);
      }
      if ((leftFrontRaw + rightFrontRaw) / 2 < 100) {
        break;
      }
      else {
        setLeftPWM(int(.2 * (3300 - leftFrontRaw)));
        setRightPWM(int(.2 * (3300 - rightFrontRaw)));
      }
      i++;
      delay(1);
    }
    setLeftPWM(0);
    setRightPWM(0);
  }
  pivotTurnRight90();

  //Insert side wall correction here


  angle = 0.0;
}

void forwardCorrection() {
  const int oneCellTicks = 327;//327
  const int noWallRight = 350; // check this value (250)
  const int noWallLeft =  450; // check this value (450)

  //  const int pegWallBack = 800; // check this value
  //  const int pegNoWalls = 1000;
  //  const int pegWallFront = 1000;
  //
  //  const int wallBackTicks = 232;
  //  const int noWallTicks = 209;
  //  const int frontWallTicks = 204;

  // encoder tick value when we check walls a cell ahead
  const int readingTicks = 215; // check this value (175)
  // encoder tick value when we switch to next cell's values
  const int newSideTicks = 260; // check this value (200)

  static bool nextRightValid;
  static bool nextLeftValid;
  static bool nextCellDecided = false;
  int errorP;
  int errorD;
  static int oldErrorP = 0;
  int totalError;
  static int lastTicksL;
  static int lastTicksR;
  static float straightAngle = 0.0;
  static bool endCell = false;
  static bool currentWallLeft = true;
  static bool currentWallRight = true;
  static bool ticksDecided = false;
  static int count = 0;
  static int prevCorrection = 4;

  //  if (accelerate) {

  //  if (leftBaseSpeed == 0) {
  //    leftBaseSpeed = 30;
  //    rightBaseSpeed = 30;
  //  }
  //  }

  //  if ((leftFront + rightFront) / 2 >= frontStop) {
  //    rightTicks = oneCellTicks;
  //    leftTicks = oneCellTicks;
  //    prevRightTicks -= oneCellTicks;
  //    prevLeftTicks -= oneCellTicks;
  //  }
  if (rightSensor < 200) {//450
    rightValid = false;
  }
  if (leftSensor < 200) {
    leftValid = false;
  }
  // Next Cell Wall Detection
  if ((rightTicks + leftTicks) / 2 >= readingTicks && !nextCellDecided) {
    angle = 0;
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
    if (prevCorrection != 0) {
      myDisplay.clear();
      myDisplay.setCursor(0);
      myDisplay.print(0);
      myDisplay.setCursor(3);
      myDisplay.print(0);
    }
    prevCorrection = 0;
    angle = 0.0;
    // Has both wall, so error correct with both (working, just need to adjust PD constants when final mouse is built)
    //    angle = 0.0;//TODO Not sure about this
//    if (afterTurnAround == true) {
//      errorP = 1 * (leftSensor - rightSensor + (leftWallDist - rightWallDist)) + 5 * (rightTicks - leftTicks);
//    }
//    else {
      errorP = .5 * (leftSensor - rightSensor + (leftWallDist - rightWallDist)) + 25 * (rightTicks - leftTicks);
//    }
    // middle of cell
    errorD = errorP - oldErrorP;
    //        getGres();
    //        gz = (float)readGyroData() * gRes - gyroBias[2];
    //        straightAngle += 2 * (gz) * 0.001;
  }
  else if (leftValid) {
    // Only left wall
    if (prevCorrection != 1) {
      myDisplay.clear();
      myDisplay.setCursor(0);
      myDisplay.print(0);
    }
    prevCorrection = 1;
    if (leftMiddleValue > 350 && leftMiddleValue < 650) {
      errorP = 2 * (leftSensor - leftWallDist + (leftWallDist - rightWallDist)) + 2.5 * (leftMiddleValue - leftSensor + 535) + 25 * (rightTicks - leftTicks) + 3 * angle;
    }
    else {
      errorP = 2 * (leftSensor - rightWallDist + (leftWallDist - rightWallDist)) + 25 * (rightTicks - leftTicks) + 3 * angle;
      //      errorP = .5 * (leftSensor - rightWallDist + (leftWallDist - rightWallDist)) + 25 * (rightTicks - leftTicks) + 5 * angle;
    }
    errorD = errorP - oldErrorP;
  }
  else if (rightValid) {
    // Only right wall
    if (prevCorrection != 2) {
      myDisplay.clear();
      myDisplay.setCursor(3);
      myDisplay.print(0);
    }
    prevCorrection = 2;
    if (rightMiddleValue > 350 && rightMiddleValue < 650) {
      errorP = 2 * (leftWallDist - rightSensor + (leftWallDist - rightWallDist)) + 2.5 * (rightSensor - rightMiddleValue - 500) + 25 * (rightTicks - leftTicks) + 3 * angle;//todo left turn version, rightMiddle Threshold
    }
    else {
      errorP = 2 * (leftWallDist - rightSensor + (leftWallDist - rightWallDist)) + 25 * (rightTicks - leftTicks) + 3 * angle;
    }
    errorD = errorP - oldErrorP;
  }
  else {
    // No walls, use gyro to correct
    if (prevCorrection != 3) {
      myDisplay.clear();
    }
    prevCorrection = 3;
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
    errorP = 75 * (rightTicks - leftTicks) + 10 * (angle);
    //errorP = 20 * (targetAngle - angle);
    //errorP = 150 * (rightTicks - leftTicks); //+ 20 * (targetAngle - angle);

    //    }
    errorD = errorP - oldErrorP;
  }

  //Peg Correction
  //#if 0
  //  // No walls in next cell or current cell
  //  if (!currentWallRight && !nextRightValid && !ticksDecided) {
  //    if (rightSensor >= pegNoWalls) {
  //      leftTicks = noWallTicks;
  //      rightTicks = noWallTicks;
  //      ticksDecided = true;
  //    }
  //  }
  //
  //  // No walls in next cell, walls in current cell
  //  else if (!currentWallRight && !nextRightValid && !ticksDecided) {
  //    if (rightSensor <= pegWallBack) {
  //      leftTicks = wallBackTicks;
  //      rightTicks = wallBackTicks;
  //      ticksDecided = true;
  //    }
  //  }
  //
  //  // No walls in current cell, wall in next cell
  //  else if (!currentWallRight && nextRightValid && !ticksDecided) {
  //    if (rightSensor >= pegWallFront) {
  //      leftTicks = frontWallTicks;
  //      rightTicks = frontWallTicks;
  //      ticksDecided = true;
  //    }
  //  }
  //
  //  // No walls in next cell or current cell
  //  if (!currentWallLeft && !nextLeftValid && !ticksDecided) {
  //    if (leftSensor >= pegNoWalls) {
  //      leftTicks = noWallTicks;
  //      rightTicks = noWallTicks;
  //      ticksDecided = true;
  //    }
  //  }
  //
  //  // No walls in next cell, walls in current cell
  //  else if (currentWallLeft && !nextLeftValid && !ticksDecided) {
  //    if (leftSensor <= pegWallBack) {
  //      leftTicks = wallBackTicks;
  //      rightTicks = wallBackTicks;
  //      ticksDecided = true;
  //    }
  //  }
  //
  //  // No walls in current cell, wall in next cell
  //  else if (!currentWallLeft && nextLeftValid && !ticksDecided) {
  //    if (leftSensor >= pegWallFront) {
  //      leftTicks = frontWallTicks;
  //      rightTicks = frontWallTicks;
  //      ticksDecided = true;
  //    }
  //  }
  //#endif

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
    leftTicks = 0;
    rightTicks = 0;
    prevRightTicks -= rightTicks;
    prevLeftTicks -= leftTicks;
    leftTicks = 0;
    rightTicks = 0;
    prevCorrection = 4;
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

void solve() {
  while (!movesDoneAndWallsSet) {
  }
//  bool walls[3];
//  walls[0] = walls_global[0];
//  walls[1] = walls_global[1];
//  walls[2] = walls_global[2];
//  movesDoneAndWallsSet = false;
  //
//  movesBuffer[0] = 'f';
//  movesBuffer[1] = 'f';
//  movesBuffer[2] = 'f';
//  movesBuffer[3] = 'f';
//  movesBuffer[4] = 'f';
//  movesBuffer[5] = 'f';
//  movesBuffer[6] = 'f';
//  movesBuffer[7] = 'f';
//  movesBuffer[8] = 'a';
//  movesBuffer[9] = 'f';
  //      movesBuffer[10] = 'r';
  //      movesBuffer[11] = 'l';
  //      movesBuffer[12] = 'f';
  //      movesBuffer[13] = 'r';
  //      movesBuffer[14] = 'f';
  //  movesBuffer[15] = 'l';
  //    movesBuffer[16] = 'f';
  //    movesBuffer[17] = 'r';
  //    movesBuffer[18] = 'a';
  //    movesBuffer[19] = 'f';
  /*
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

  rightWallFollow();

  movesReady = true;
}

void rightWallFollow() {
  bool walls[3];
  walls[0] = walls_global[0];
  walls[1] = walls_global[1];
  walls[2] = walls_global[2];
  movesDoneAndWallsSet = false;
  if (!walls[2]) {
    //    myDisplay.setCursor(0);
    //    myDisplay.println("right");
    movesBuffer[0] = 'r';
    movesBuffer[1] = 0;
  }
  else if (!walls[1]) {
    //    myDisplay.setCursor(0);
    //    myDisplay.println("frwd");
    movesBuffer[0] = 'f';
    movesBuffer[1] = 0;
  }
  else if (!walls[0]) {
    //    myDisplay.setCursor(0);
    //    myDisplay.println("left");
    movesBuffer[0] = 'l';
    movesBuffer[1] = 0;
  }
  else {
    myDisplay.setCursor(0);
    myDisplay.println("arnd");
    movesBuffer[0] = 'a';
    movesBuffer[1] = 'f';
    movesBuffer[2] = 0;
  }
}

void leftWallFollow() {
  bool walls[3];
  walls[0] = walls_global[0];
  walls[1] = walls_global[1];
  walls[2] = walls_global[2];
  movesDoneAndWallsSet = false;
  if (!walls[0]) {
    //    myDisplay.setCursor(0);
    //    myDisplay.println("left");
    movesBuffer[0] = 'l';
    movesBuffer[1] = 0;
  }
  else if (!walls[1]) {
    //    myDisplay.setCursor(0);
    //    myDisplay.println("frwd");
    movesBuffer[0] = 'f';
    movesBuffer[1] = 0;
  }
  else if (!walls[2]) {
    //    myDisplay.setCursor(0);
    //    myDisplay.println("rght");
    movesBuffer[0] = 'r';
    movesBuffer[1] = 0;
  }
  else {
    myDisplay.setCursor(0);
    myDisplay.println("arnd");
    movesBuffer[0] = 'a';
    movesBuffer[1] = 'f';
    movesBuffer[2] = 0;
  }
}



