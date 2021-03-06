
void readGyro() {
  int gyroPin = A19;               //Gyro is connected to analog pin 19
  float gyroVoltage = 3.3;         //Gyro is running at 3.3V
  float gyroZeroVoltage = 1.554;   //Gyro is zeroed at 1.542V TODO - 1.56 Works well for pivot turns
  float gyroSensitivity = .002675;  //.00265 (right)
  float rotationThreshold = 5;   //Minimum deg/sec to keep track of - helps with gyro drifting
  //This line converts the 0-1023 signal to 0-3.3V
  float gyroRate = (analogRead(gyroPin) * gyroVoltage) / 4096.0;
  //This line finds the voltage offset from sitting still
  gyroRate -= gyroZeroVoltage;
  //Serial.println(gyroRate);
  //This line divides the voltage we found by the gyro's sensitivity
  if (moveType == TURN_RIGHT) {
//    gyroSensitivity = .00265;//TODO Not sure about this
  }
  gyroRate /= gyroSensitivity;

  //Ignore the gyro if our angular velocity does not meet our threshold
  if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold) {
    //This line divides the value by 100 since we are running in a 10ms loop (1000ms/10ms)
    gyroRate /= 500.0;
    angle += gyroRate;
  }
}

