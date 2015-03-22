// Teensy 3 I2C Library
#include <i2c_t3.h>
#include "Gyro.h"
#include "Motors.h"
#include "Sensors.h"

#define hasLeftWall 400
#define hasRightWall 800

// PID Constants
#define Kp 30
#define Kd 20

const int buttonPin = 24;

const int LED2 = 16;

void setup() {
  Serial.begin(115200);

  // 12 bit ADC resolution
  analogReadResolution(12);
  
  setupMotors();
  setupSensors();

  pinMode(LED2, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(intPin, INPUT);

  // Wait for Button Press to Start
  readSensors();
//  while (digitalRead(buttonPin) == 1) {
//  }
  while (rightFront < 3300 && rightMiddleValue < 3300 && rightSensor < 3300) {
    readSensors();
  }
  delay(2000);
  
  setupGyro();
  
//  turnRight();
//turnLeft();

}

void loop() {
  //  moveForward()
//readSensors();
  //  if (wallLeft())
  //    Serial.println("Wall Left");
  //    if (wallRight())
  //    Serial.println("Wall Right");
  //  if (wallFront())
  //  Serial.println("Wall Front");
  //  delay(100);
  //  Serial.print(leftSensor);
  //  Serial.print(" ");
  //  Serial.println(rightSensor);

  //    if (!wallRight()) {
  //      delay(100);
  //      turnRight();
  //    }
  //    while (wallFront()) {
  //      turnLeft();
  //    }
//      moveForward();
      delay(100);
      if (wallFront()) {
        for (int i = currentLeftPWM; i > 0; i-=2) {
          setLeftPWM(i);
          setRightPWM(i);
          
        }
        while(1) {
        setLeftPWM(0);
        setRightPWM(0);
        }
      }
}

//mack calls certain number of move forwards, we add however many ticks for every move forward

void moveForward()
{
  const int tickCount = 554;
  int errorP;
  int errorD;
  int oldErrorP;
  int totalError;
  int leftBaseSpeed = 30;
  int rightBaseSpeed = 40;
  //  leftTicks = 0;
  //  rightTicks = 0;
  //  while (leftTicks < tickCount || rightTicks < tickCount) {
//  readSensors();
  if (wallLeft() && wallRight()) //has both walls
  { //ccw direction is positive
          Serial.println("Has Both");
    errorP = leftSensor - rightSensor + 100;//1326 is the offset between left and right sensor when mouse in the middle of cell
    errorD = errorP - oldErrorP;
  }
  //try not using threshold value, detect with diags when cell behind

  else if (wallLeft()) //only has left wall
  {
      Serial.println("Has Left");
//      errorP = 2 * (leftMiddleValue – leftSensor);
//      errorD = errorP – oldErrorP;
          errorP = 0;//(leftEncoder – rightEncoder*1005/1000)*3;
   errorD = 0;
  }
  else if (wallRight()) //only has right wall
  {
    Serial.println("Has Right");
//     errorP =  2*(rightSensor - rightMiddleValue);
//     errorP =  3000-rightSensor;
//     errorD = errorP - oldErrorP;
     errorP = 0;
     errorD = 0;
  }
  else //no wall, use encoder or gyro
  {
  Serial.println("Has None");
    errorP = 0;//(leftEncoder – rightEncoder*1005/1000)*3;
    errorD = 0;
  }
  //Calculate total error
  totalError = Kp * errorP + Kd * errorD;
  oldErrorP = errorP;

  // Calculate PWM based on Error
  currentLeftPWM = leftBaseSpeed + totalError / 1000;
  currentRightPWM = rightBaseSpeed - totalError / 1000;

  Serial.print(currentLeftPWM);
  Serial.print(" ");
  Serial.println(currentRightPWM);

  // Update Motor PWM values
  setLeftPWM(currentLeftPWM);
  setRightPWM(currentRightPWM);

  delay(1);// Replace with Timer Interrupt
  //    Serial.println((rightTicks + leftTicks)/2);
  //  }
  //  setLeftPWM(0);
  //  setRightPWM(0);
  //  delay(200);

  //  rightTicks = 0;
  //  leftTicks = 0;
}

void turnRight() {
  
  
  //  int tickCount = 190;
  // Gyro calibrated for each speed or turning is not accurate
  float degreesTraveled = 0;
  const int turnSpeed = 50;
  const int targetDegrees = 82;
  //  const int turnSpeed = 45;
  //  const int targetDegrees = 85.5
  //  const int turnSpeed = 40;
  //  const int targetDegrees = 86
  float initialZ;

  //  rightTicks = 0;
  //  leftTicks = 0;
  delay(200);
  getGres(); 
  gz = (float)readGyroData()*gRes - gyroBias[2];
  initialZ = gz;// May not be necessary
  count = millis();
  setLeftPWM(turnSpeed);
  setRightPWM(-turnSpeed);
  while (degreesTraveled >= -targetDegrees) {
    uint32_t deltat = millis() - count;
    if (deltat > 1) {
  getGres(); 
  gz = (float)readGyroData()*gRes - gyroBias[2];
  degreesTraveled += 2*(gz - initialZ) * 0.001;
      count = millis();

    }
  }

  // Needs to deccelerate for the motors to stop correctly
  for (int i = turnSpeed; i >= 0; i--) {
    setLeftPWM(i);
    setRightPWM(-i);
    
  }
  delay(200);
}

void turnLeft() {
  
  
  //  int tickCount = 190;
  // Gyro calibrated for each speed or turning is not accurate
  float degreesTraveled = 0;
  const int turnSpeed = 50;
  const int targetDegrees = 82;
  //  const int turnSpeed = 45;
  //  const int targetDegrees = 85.5
  //  const int turnSpeed = 40;
  //  const int targetDegrees = 86
  float initialZ;

  //  rightTicks = 0;
  //  leftTicks = 0;
  delay(200);
  getGres(); 
  gz = (float)readGyroData()*gRes - gyroBias[2];
  initialZ = gz;// May not be necessary
  count = millis();
  setLeftPWM(-turnSpeed);
  setRightPWM(turnSpeed);
  while (degreesTraveled <= targetDegrees) {
    uint32_t deltat = millis() - count;
    if (deltat > 1) {
  getGres(); 
  gz = (float)readGyroData()*gRes - gyroBias[2];
  degreesTraveled += 2*(gz - initialZ) * 0.001;
      count = millis();

    }
  }

// Needs to deccelerate for the motors to stop correctly
  for (int i = turnSpeed; i >= 0; i--) {
    setLeftPWM(-i);
    setRightPWM(i);
    
  }
  delay(200);
}

boolean wallFront() {
  return (leftFront>2900 && rightFront>2900);
}

boolean wallLeft() {
//  readSensors();
  return (leftSensor > hasLeftWall);
}

boolean wallRight() {
//  readSensors();
  return (rightSensor > hasRightWall);
}






