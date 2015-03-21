// Teensy 3 I2C Library
#include <i2c_t3.h>

// Gyro Registers
#define XGOFFS_TC        0x00 // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD                 
#define YGOFFS_TC        0x01
#define ZGOFFS_TC        0x02
#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B
#define SELF_TEST_X      0x0D
#define SELF_TEST_Y      0x0E
#define SELF_TEST_Z      0x0F
#define SELF_TEST_A      0x10
#define XG_OFFS_USRH     0x13  // User-defined trim values for gyroscope; supported in MPU-6050?
#define XG_OFFS_USRL     0x14
#define YG_OFFS_USRH     0x15
#define YG_OFFS_USRL     0x16
#define ZG_OFFS_USRH     0x17
#define ZG_OFFS_USRL     0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define FF_THR           0x1D  // Free-fall
#define FF_DUR           0x1E  // Free-fall
#define MOT_THR          0x1F  // Motion detection threshold bits [7:0]
#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL   0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU6050 0x75 // Should return 0x68

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

// Specify sensor full scale
int Gscale = GFS_2000DPS;
int Ascale = AFS_2G;
float aRes, gRes;

int16_t accelCount[3];           // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;                // Stores the real accel value in g's
int16_t gyroCount[3];            // Stores the 16-bit signed gyro sensor output
float gx, gy, gz;                // Stores the real gyro value in degrees per seconds
float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
int16_t tempCount;               // Stores the internal chip temperature sensor output
float temperature;               // Scaled temperature in degrees Celsius
float SelfTest[6];               // Gyro and accelerometer self-test sensor output
uint32_t count = 0;

#define MPU6050_ADDRESS 0x68  // Device address when ADO = 0

#define left 0
#define diagl 1
#define lf 2
#define rf 3
#define right 4
#define diagr 5
#define ML1 20
#define ML2 6
#define MR1 21
#define MR2 5
#define ER1 8
#define ER2 7
#define EL1 26
#define EL2 31
#define hasLeftWall 160
#define hasRightWall 160

// PID Constants
#define Kp 16
#define Kd 10
// ADD I

// IR Pair Pins
int TX[6] = {25, 10,   9,  22,  32, 10};
int RX[6] = {A4, A17, A16, A14, A5, A3};
const int diagHighPower = 23;

// Motor Pins
const int motorPins[] = {MR1, MR2, ML1, ML2};

// Encoder Pins
const int encoderPins[] = {ER1, ER2, EL1, EL2};

const int buttonPin = 24;

// Gyro Interrupt Pin (Not Used)
const int intPin = 13;

const int LED2 = 16;

int leftSensor;
int rightSensor;
int rightMiddleValue;
int leftMiddleValue;
int leftFront;
int rightFront;

int currentRightPWM = 0;
int currentLeftPWM = 0;

// Encoder Ticks
int rightTicks = 0;
int leftTicks = 0;

void setup() {
  Serial.begin(115200);

  // 12 bit ADC resolution
  analogReadResolution(12);

  // Begin I2C using Teensy's seconds I2C bus (Wire1) on pins 29 and 30 with a speed of 400kHz
  Wire1.begin(I2C_MASTER, 0x68, I2C_PINS_29_30, I2C_PULLUP_INT, I2C_RATE_400);

  // Define Motor Pins as Outputs
  for (int i = 0; i < 4; i++) {
    pinMode(motorPins[i], OUTPUT);
  }

  // Define IR Pair TXs as Outputs
  for (int i = 0; i < 6; i++) {
    pinMode(TX[i], OUTPUT);
  }
  pinMode(diagHighPower, OUTPUT);

  pinMode(LED2, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  // Define Encoder Pins as Inputs
  for (int i = 0; i < 4; i++) {
    pinMode(encoderPins[i], INPUT);
  }

  pinMode(intPin, INPUT);

  // Encoder Interrupt Setups
  attachInterrupt(ER1, rightEncoder, CHANGE);
  attachInterrupt(ER2, rightEncoder, CHANGE);
  attachInterrupt(EL1, leftEncoder, CHANGE);
  attachInterrupt(EL2, leftEncoder, CHANGE);

  // Wait for Button Press to Start
  readSensors();
//  while (digitalRead(buttonPin) == 1) {
//  }
  while (rightFront < 3300 && rightMiddleValue < 3300 && rightSensor < 3300) {
    readSensors();
  }
  delay(2000);

  // Calibrate gyro and accelerometers, load biases in bias registers
  calibrateMPU6050(gyroBias, accelBias);
  //Initialize Gyro
  initMPU6050();
  
//  turnRight();
//turnLeft();

}

void loop() {
  //  moveForward()
readSensors();
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
      moveForward();
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
//    errorP = 2 * (leftMiddleValue - leftSensor + 30);
//    errorD = errorP - oldErrorP;
    //      errorP = 0;//(leftEncoder – rightEncoder*1005/1000)*3;
    //      errorD = 0;
  }
  else if (wallRight()) //only has right wall
  {
    Serial.println("Has Right");
    //    errorP =   2*(rightSensor - rightMiddleValue);
    //    errorD = errorP - oldErrorP;
    errorP = 0;
    errorD = 0;
  }
  else if (!wallLeft() && !wallRight()) //no wall, use encoder or gyro
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

//  Serial.print(currentLeftPWM);
//  Serial.print(" ");
//  Serial.println(currentRightPWM);

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
  return (leftFront>3000 && rightFront>3000);
}

boolean wallLeft() {
//  readSensors();
  return (leftSensor > hasLeftWall);
}

boolean wallRight() {
//  readSensors();
  return (rightSensor > hasRightWall);
}

int readPair(int pair) {
  // TODO Shorten Times
  int reading;
  digitalWrite(TX[pair], HIGH);
  delayMicroseconds(100);
  reading = analogRead(RX[pair]);
  digitalWrite(TX[pair], LOW);
  delayMicroseconds(100);
  return reading;
}

//void readSensors() {
//  // TODO Calibrate Better
//  leftSensor = 1390.9 * (1 / getLn(readPair(left))) - 211.64;
//  rightSensor = 7169.4 * (1 / getLn(readPair(right))) - 862.45;
//  leftMiddleValue = 792.18 * (1 / getLn(readPair(diagl))) - 110.27 + 15; //15?
//  rightMiddleValue = 391.17 * (1 / getLn(readPair(diagr))) - 63.22 + 15; //15?
//}

void readSensors() {
  digitalWriteFast(TX[lf],HIGH);
  delayMicroseconds(60);
  leftFront = analogRead(RX[lf]);
  digitalWriteFast(TX[lf],LOW);
  delayMicroseconds(80);
  
  digitalWriteFast(TX[rf],HIGH);
  delayMicroseconds(60);
  rightFront = analogRead(RX[rf]);
  digitalWriteFast(TX[rf],LOW);
  delayMicroseconds(80);
  
  digitalWriteFast(TX[left],HIGH);
  digitalWriteFast(TX[right],HIGH);
  delayMicroseconds(60);
  leftSensor = analogRead(RX[left]);
  rightSensor = analogRead(RX[right]);
  digitalWriteFast(TX[left],LOW);
  digitalWriteFast(TX[right],LOW);
  delayMicroseconds(80);
  
  digitalWriteFast(TX[diagl],HIGH);
  digitalWriteFast(23, HIGH);
  delayMicroseconds(60);
  leftMiddleValue = analogRead(RX[diagl]);
  rightMiddleValue = analogRead(RX[diagr]);
  digitalWriteFast(TX[diagl],LOW);
  digitalWriteFast(23, LOW);
}

void setLeftPWM(int value) {
  if (value >= 0) {
    analogWrite(ML1, value);
    digitalWrite(ML2, LOW);
  }

  else if (value == 0) {
    digitalWrite(ML1, LOW);
    digitalWrite(ML2, LOW);
  }
  else {
    analogWrite(ML2, abs(value));
    digitalWrite(ML1, LOW);
  }
}

void setRightPWM(int value) {
  if (value >= 0) {
    analogWrite(MR1, value);
    digitalWrite(MR2, LOW);
  }
  else if (value == 0) {
    digitalWrite(MR1, LOW);
    digitalWrite(MR2, LOW);
  }
  else {
    analogWrite(MR2, abs(value));
    digitalWrite(MR1, LOW);
  }
}

// Right Encoder ISR
void rightEncoder() {
  rightTicks++;
}

// Left Encoder ISR
void leftEncoder() {
  leftTicks++;
}
