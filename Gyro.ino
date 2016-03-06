
void readGyro() {
  int gyroPin = A19;               //Gyro is connected to analog pin 19
  float gyroVoltage = 3.3;         //Gyro is running at 3.3V
  float gyroZeroVoltage = 1.542;   //Gyro is zeroed at 1.55V
  float gyroSensitivity = .00265;  //4.1mV/deg/sec
  float rotationThreshold = 4;   //Minimum deg/sec to keep track of - helps with gyro drifting
  //This line converts the 0-1023 signal to 0-3.3V
  float gyroRate = (analogRead(gyroPin) * gyroVoltage) / 4096.0;
  //This line finds the voltage offset from sitting still
  gyroRate -= gyroZeroVoltage;
  //Serial.println(gyroRate);
  //This line divides the voltage we found by the gyro's sensitivity
  gyroRate /= gyroSensitivity;

  //Ignore the gyro if our angular velocity does not meet our threshold
  if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold) {
    //This line divides the value by 100 since we are running in a 10ms loop (1000ms/10ms)
    gyroRate /= 500.0;
    angle += gyroRate;
  }
}

