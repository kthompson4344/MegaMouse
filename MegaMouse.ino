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

/* Variables for interface between drive code and algorithm */
char movesBuffer[256];
bool walls_global[3] = {false, false, false}; // Needs to be initialized.
volatile bool movesReady = false; // Set to true by algorithm, set to false by drive. // TODO: First move
volatile bool movesDoneAndWallsSet = false; // Set to true by drive, set to false by algorithm.
/* End of variables for interface */

bool currentMoveDone = false;
bool firstMove = true;

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
volatile bool haveSensorReading = false;

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
    //    Serial.begin(9600);
    mySerial.begin(115200);

    // 12 bit ADC resolution
    analogReadResolution(12);

    setupMotors();
    setupSensors();




    pinMode(LED, OUTPUT);

    pinMode(buttonPin, INPUT_PULLUP);

    pinMode(intPin, INPUT);

    // Wait for Button Press to Start
    // readSensors();
    //while (digitalRead(buttonPin) == 1) {
    //}
    //while (rightFront < 3300 && rightMiddleValue < 3300 && rightSensor < 3300) {
    //        readSensors();
    //        Serial.println(leftSensor);
    //    }
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
    movesDoneAndWallsSet = true;
    correctionTimer.priority(255);
    correctionTimer.begin(correction, 1000);
    
}

#include "MackAlgo.h"
mack::MackAlgo algo;
void loop() {

    algo.solve();

}

// Mack calls certain number of move forwards; we add however many ticks for every move forward.

void correction() {

    static byte indexInBuffer = 0;
    
    if (!movesReady) {
        // Hoping we never get here, but maybe the algorithm is slow.
        haveSensorReading = false;
        return;
    }
    
    if (currentMoveDone) {
        indexInBuffer += 1;
        currentMoveDone = false;
        if (movesBuffer[indexInBuffer] == 0) {
            // Make sure walls_global is set by the time we get here (STILL NEED TO DO IN TURN AROUND).
            movesReady = false;
            movesDoneAndWallsSet = true;
            indexInBuffer = 0;
            haveSensorReading = false;
            return;
        }
    }

    switch (movesBuffer[indexInBuffer]) {
        case 'f':
            forwardCorrection();
            break;
        case 'r':
            if(firstMove) {
                //do our special move
                firstMove = false;
                currentMoveDone = true; 
            }
        case 'l': // Fall-through"
            turnCorrection();
            break;
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
    }
    else {
        rightTicks = 0;
        leftTicks = 0;
    }
    leftBaseSpeed = 240;//240
    rightBaseSpeed = 240;//240

    rightValid = wallRight();
    leftValid = wallLeft();
    moveType = FORWARD;
    //    needMove = false;
}

void turnRight() {
    leftBaseSpeed = 240;
    rightBaseSpeed = 240;
    moveType = TURN_RIGHT;
    //    needMove = false;
}

void turnLeft() {
    leftBaseSpeed = 240;
    rightBaseSpeed = 240;
    moveType = TURN_LEFT;
    //    needMove = false;
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

    while (leftFront < frontLeftStop || rightFront < frontRightStop) {
        errorP = leftSensor - rightSensor - 100; // 100 is the offset between left and right sensor when mouse in the
        // middle of cell
        errorD = errorP;
        totalError = straightKp * errorP + Kd * errorD;

        // Calculate PWM based on Error
        currentLeftPWM = leftBaseSpeed + totalError / 124;
        currentRightPWM = rightBaseSpeed - totalError / 124;

        // Update Motor PWM values
        setLeftPWM(currentLeftPWM);
        setRightPWM(currentRightPWM);
#if 0
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
#endif
    }
    setRightPWM(0);
    setLeftPWM(0);
    delay(200);
    leftStop = false;
    rightStop = false;

    pivotTurnRight();

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
    //    needMove = true;
}

void forwardCorrection() {
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
    const int newSideTicks = 200; // check this value (200)

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
        const int wallDist = 1900; // Make this bigger to move closer to the wall
        // Only left wall
        // errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100 * (angle - targetAngle);
        getGres();
        gz = (float)readGyroData() * gRes - gyroBias[2];
        angle += 2 * (gz) * 0.001;
        errorP = 20 * (angle) + .5 * (leftSensor - wallDist);
        errorD = errorP - oldErrorP;

    }
    else if (rightValid) {
        const int wallDist = 1900; //Make this bigger to move closer to the wall
        // Only right wall
        getGres();
        gz = (float)readGyroData() * gRes - gyroBias[2];
        angle += 2 * (gz) * 0.001;
        errorP = 20 * (angle) - .5 * (rightSensor - wallDist);
        errorD = errorP - oldErrorP;
    }
    else {
        //        digitalWriteFast(LED1, LOW);
        //        digitalWriteFast(LED2, LOW);
        //        Serial.println("Has None");
        //        if (changeR < 0 || changeL < 0) {
        //            errorP = 0;
        //            errorD = 0;
        //        }
        //        else {
        //            errorP = 1000*(changeR - changeL);
        //        }
        // No walls, use encoders to correct
        getGres();
        gz = (float)readGyroData() * gRes - gyroBias[2];
        angle += 2 * (gz) * 0.001;
        errorP = 20 * (angle);
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
    const int frontOffset = 50; // difference between left and right front sensors when lined up with the wall

    if (!straight) {
        if (turn) {
#if 0
            if (i < 30) {
                if (wallLeft() && wallRight()) {
                    angle = 0.0;
                    targetAngle = 0;
                    // Has both wall, so error correct with both (working, just need to adjust PD constants when final mouse is built)
                    errorP = leftSensor - rightSensor - 100; // 100 is the offset between left and right sensor when mouse in the
                    // middle of cell
                    errorD = errorP - oldErrorP;
                    // getGres();
                    // gz = (float)readGyroData() * gRes - gyroBias[2];
                    // straightAngle += 2 * (gz) * 0.001;
                }
                else if (wallLeft()) {
                    const int wallDist = 1900; // Make this bigger to move closer to the wall
                    // Only left wall
                    // errorP = 2 * (leftMiddleValue - leftSensor + 1200) + 100 * (angle - targetAngle);
                    getGres();
                    gz = (float)readGyroData() * gRes - gyroBias[2];
                    angle += 2 * (gz) * 0.001;
                    errorP = 20 * (angle - targetAngle) + .5 * (leftSensor - wallDist);
                    errorD = errorP - oldErrorP;
                }
                else if (wallRight()) {
                    const int wallDist = 2000; //Make this bigger to move closer to the wall
                    // Only right wall
                    getGres();
                    gz = (float)readGyroData() * gRes - gyroBias[2];
                    angle += 2 * (gz) * 0.001;
                    errorP = 20 * (angle - targetAngle) - .5 * (rightSensor - wallDist);
                    errorD = errorP - oldErrorP;
                }
                else {
                    getGres();
                    gz = (float)readGyroData() * gRes - gyroBias[2];
                    angle += 2 * (gz) * 0.001;
                    errorP = 20 * (angle - targetAngle);
                    errorD = errorP - oldErrorP;
                }
            }
#endif

            targetAngle = curve2[i];
            getGres();

            gz = (int)readGyroData() * gRes - gyroBias[2];

            if (moveType == TURN_RIGHT) {
                angle += 1.37 * (gz) * 0.001;
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
        else {
            const int frontStop = 700;
            if (wallFront()) {
                getGres();
                gz = (float)readGyroData() * gRes - gyroBias[2];
                angle += 1 * (gz) * 0.001;
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
                }
            }
            else {
                turn = true;
            }
        }
    }
    else {
        const int wallFrontValue = 350;
        getGres();
        gz = (float)readGyroData() * gRes - gyroBias[2];
        angle += 1 * (gz) * 0.001;
        if (rightFront > wallFrontValue && leftFront > wallFrontValue) {
            errorP = 3 * (rightFront - leftFront - frontOffset);
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
            targetAngle = 120;
        }
    }

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
    const float targetDegrees = 160;
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
            Serial.println(degreesTraveled);
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

#if (0)
void solve() {
    while (!movesDoneAndWallsSet) {
    }
    bool walls[3];
    memcpy(walls, walls_global, 3 * sizeof(bool)); // walls = walls_global
    movesDoneAndWallsSet = false;
    /* TODO: Figure out next moves and put them into moveBuffer. */

    // 'f' - forward
    // 'l' - left
    // 'r' - right
    // 'a' - turn around
    // Then put a null character at the end of moveBuffer.
    if (!walls[2]) {
        movesBuffer[0] = 'r';
    }
    else if (!walls[1]) {
        movesBuffer[0] = 'f';
    }
    else if (!walls[0]) {
        movesBuffer[0] = 'l';
    }
    else {
        movesBuffer[0] = 'a';
    }
    movesBuffer[1] = 0;
    movesReady = true;
}
#endif

void accelerate(int numCells) {
    int cellNumb = 1;
    static bool cellDecided = false;
    if ((rightTicks + leftTicks) / 2 >= 320 && !cellDecided) {
        ++cellNumb;
        cellDecided = true;
    }

    if ((rightTicks + leftTicks) / 2 <= 10) {
        cellDecided = false;
    }
    if (cellNumb == 1) {
        static int i = 0;
        ++i;
        if (i >= 10) {
            leftBaseSpeed += 1;
            rightBaseSpeed += 1;
            i = 0;
        }
        if (leftBaseSpeed >= 400) {
            rightBaseSpeed = 400;
            leftBaseSpeed = 400;
        }
    }
    if (cellNumb == numCells - 1 || cellNumb == numCells) {
        static int i = 0;
        if (i >= 10) {
            leftBaseSpeed -= 1;
            rightBaseSpeed -= 1;
            i = 0;
        }
        if (leftBaseSpeed < 240 || rightBaseSpeed < 240) {
            leftBaseSpeed = 240;
            rightBaseSpeed = 240;
        }
    }
}
