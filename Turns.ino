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
  static bool wallInFront = false;

  if (i == 0) {
    angle = 0.0;
    myDisplay.clear();
    //    myDisplay.setCursor(0);
    //    myDisplay.clear();
    //    if (moveType == TURN_RIGHT) {
    //      //      myDisplay.print("RGHT");
    //    }
    //    else {
    //      //      myDisplay.print("LEFT");
    //    }
  }
  if (turn == false) {
    if (wallFront()) {
      wallInFront = true;
      targetAngle = 0;// TODO do forward correction during this part?
      if ((rightFront + leftFront) / 2 > frontStop) {
        turn = true;
        i = 1;
        //        i = round(pow(11.346 * abs(angleConst*(rightFront - leftFront)),0.5492));//how does R-L relate to angle?
        //        angle = angleConst*(rightFront-leftFront);
//        angle = 0.0;
      }
    }
    else {
      myDisplay.setCursor(0);
      myDisplay.clear();
      myDisplay.print("NoWF");
      if ((leftTicks + rightTicks) / 2 >= 35) {//TODO Find this value, add correction
        if ((moveType == TURN_RIGHT && !wallRight()) || moveType == TURN_LEFT && !wallLeft()) {
          turn = true;
          i = 1;
//          angle = 0.0;
        }
      }
    }
    //turn = true;
  }
  else {
    if (moveType == TURN_RIGHT) {
      targetAngle = curve5[i];
    }
    else {
      targetAngle = -curve5[i];
    }
  }

  if (straight == false && abs(targetAngle) == 90) {
    //    prevRightTicks-= rightTicks;
    //    prevLeftTicks-= leftTicks;
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
    if ((rightTicks + leftTicks) / 2 > 35  || (leftFront + rightFront) / 2 > frontStop) {//TODO CHECK THIS TICK VALUE
      continueTurn = false;
    }
  }

  if ((wallInFront == true && turn == false) || (straight == true && wallFront())) {
//    errorP = .02 * (rightFrontRaw - leftFrontRaw + 300) + targetAngle - angle;//seems to work well but sometimes freaks out
    //    errorP = 2 * (rightFront - leftFront);
        errorP = targetAngle - angle;
//        errorP = .5 * (rightFront - leftFront - 0.0) + targetAngle - angle;
  }
  else {
    errorP = targetAngle - angle;
  }
  errorD = errorP - oldErrorP;
  totalError = 18 * errorP + 10 * errorD;
  // Calculate PWM based on Error
  currentLeftPWM = leftBaseSpeed + int(totalError);
  currentRightPWM = rightBaseSpeed - int(totalError);
  //    myDisplay.setCursor(0);
  //    myDisplay.clear();
  //    myDisplay.print(totalError);
  //    // Update Motor PWM values
  setLeftPWM(currentLeftPWM);//TODO, multiply by .5 and you get a nice pivot turn
  setRightPWM(currentRightPWM);

  if (i == 50) {
    //    myDisplay.clear();
  }
  if (continueTurn || !buttonPressed) {
    if (!straight) {
      i++;
    }
  }
  else {
    i = 0;
    //    angle = 0.0;
    //    prevRightTicks -= rightTicks;
    //    prevLeftTicks -= leftTicks;
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
    wallInFront = false;
  }
}

void pivotTurnRight180() {
  int errorP;
  int errorD;
  int totalError;
  int oldErrorP;
  int tickCount = 190;
  int finishedCount = 0;
  // Gyro calibrated for each speed or turning is not accurate
  float degreesTraveled = 0;
  const int turnSpeed = 350;
  const float targetDegrees = 180;
  float targetAngle = 0;

  float initialZ;
  long count = 0;
  int i = 0;
  //    rightTicks = 0;
  //    leftTicks = 0;
  //    delay(200);

  //    while (rightTicks > -90 || leftTicks < 90) {
  //    }
  leftTicks = 0;
  rightTicks = 0;
  angle = 0;
  degreesTraveled = 0;
  count = micros();
  while (targetAngle < targetDegrees) {
    uint32_t deltat = micros() - count;
    if (deltat > 1000) {
      getSpeed();
      if (targetAngle < 180) {
        targetAngle = 2*curve4[i];
        i++;
      }
      else {
        finishedCount++;
      }
      readGyro();
      errorP = angle - targetAngle;// + .1 * (leftSpeed + rightSpeed);
      errorD = errorP - oldErrorP;
      totalError = 40 * errorP + 20 * errorD;
      currentLeftPWM = -totalError;
      currentRightPWM = totalError;
      setLeftPWM(currentLeftPWM);// + (int)(.5 * (abs(rightTicks) - leftTicks)));
      setRightPWM(currentRightPWM);
      oldErrorP = errorP;
      count = micros();
      if (finishedCount > 5) {
        //        while (currentLeftPWM > 0 && currentRightPWM > 0) {
        //          if (currentLeftPWM > 0) {
        //            setLeftPWM(currentLeftPWM--);
        //          }
        //          if (currentRightPWM < 0) {
        //            setRightPWM(currentRightPWM++);
        //          }
        //        }
        //delay(100);
        break;
      }
    }

  }
  while (currentLeftPWM > 0 && currentRightPWM < 0) {
    if (currentLeftPWM > 0) {
      currentLeftPWM--;
      setLeftPWM(currentLeftPWM);
    }
    if (currentRightPWM < 0) {
      currentRightPWM++;
      setRightPWM(currentRightPWM);
    }
    delay(1);
  }
  setLeftPWM(0);
  setRightPWM(0);
  //  prevLeftTicks -= leftTicks;
  //  prevRightTicks -= rightTicks;
  leftTicks = 0;
  rightTicks = 0;
  delay(400);
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
  int i = 0;
  //    rightTicks = 0;
  //    leftTicks = 0;
  //    delay(200);

  //    while (rightTicks > -90 || leftTicks < 90) {
  //    }
  leftTicks = 0;
  rightTicks = 0;
  angle = 0;
  degreesTraveled = 0;
  count = micros();
  while (targetAngle < targetDegrees) {
    uint32_t deltat = micros() - count;
    if (deltat > 1000) {
      getSpeed();
      if (targetAngle < 90) {
        targetAngle = curve4[i];
        i++;
      }
      else {
        finishedCount++;
      }
      readGyro();
      errorP = angle - targetAngle;// + .1 * (leftSpeed + rightSpeed);
      errorD = errorP - oldErrorP;
      totalError = 39 * errorP + 20 * errorD;
      currentLeftPWM = -totalError;
      currentRightPWM = totalError;
      setLeftPWM(currentLeftPWM);// + (int)(.5 * (abs(rightTicks) - leftTicks)));
      setRightPWM(currentRightPWM);
      oldErrorP = errorP;
      count = micros();
      if (finishedCount > 5) {
        //        while (currentLeftPWM > 0 && currentRightPWM > 0) {
        //          if (currentLeftPWM > 0) {
        //            setLeftPWM(currentLeftPWM--);
        //          }
        //          if (currentRightPWM < 0) {
        //            setRightPWM(currentRightPWM++);
        //          }
        //        }
        //delay(100);
        break;
      }
    }

  }
  while (currentLeftPWM > 0 && currentRightPWM < 0) {
    if (currentLeftPWM > 0) {
      currentLeftPWM--;
      setLeftPWM(currentLeftPWM);
    }
    if (currentRightPWM < 0) {
      currentRightPWM++;
      setRightPWM(currentRightPWM);
    }
    delay(1);
  }
  setLeftPWM(0);
  setRightPWM(0);
  //  prevLeftTicks -= leftTicks;
  //  prevRightTicks -= rightTicks;
  leftTicks = 0;
  rightTicks = 0;
  delay(400);
  //    // Needs to deccelerate for the motors to stop correctly
  //  for (int i = turnSpeed; i >= 0; --i) {
  //    setLeftPWM(i);
  //    setRightPWM(-i);
  //  }
}

void pivotTurnLeft90() {
  int errorP;
  int errorD;
  int totalError;
  int oldErrorP;
  int tickCount = 190;
  int finishedCount = 0;
  // Gyro calibrated for each speed or turning is not accurate
  float degreesTraveled = 0;
  const int turnSpeed = 350;
  const float targetDegrees = -90;
  float targetAngle = 0;
  // const int turnSpeed = 45;
  // const int targetDegrees = 85.5
  // const int turnSpeed = 40;
  // const int targetDegrees = 86
  float initialZ;
  long count = 0;
  int i = 0;
  //    rightTicks = 0;
  //    leftTicks = 0;
  //    delay(200);

  //    while (rightTicks > -90 || leftTicks < 90) {
  //    }
  leftTicks = 0;
  rightTicks = 0;
  angle = 0;
  degreesTraveled = 0;
  count = micros();
  while (targetAngle > targetDegrees) {
    uint32_t deltat = micros() - count;
    if (deltat > 1000) {
      getSpeed();
      if (targetAngle > -90) {
        targetAngle = -curve4[i];
        i++;
      }
      else {
        finishedCount++;
      }
      readGyro();
      errorP = angle - targetAngle;// + .1 * (leftSpeed + rightSpeed);
      errorD = errorP - oldErrorP;
      totalError = 39 * errorP + 20 * errorD;
      currentLeftPWM = -totalError;
      currentRightPWM = totalError;
      setLeftPWM(currentLeftPWM);// + (int)(.5 * (abs(rightTicks) - leftTicks)));
      setRightPWM(currentRightPWM);
      oldErrorP = errorP;
      count = micros();
      if (finishedCount > 5) {
        //        while (currentLeftPWM > 0 && currentRightPWM > 0) {
        //          if (currentLeftPWM > 0) {
        //            setLeftPWM(currentLeftPWM--);
        //          }
        //          if (currentRightPWM < 0) {
        //            setRightPWM(currentRightPWM++);
        //          }
        //        }
        //delay(100);
        break;
      }
    }

  }
  while (currentRightPWM > 0 && currentLeftPWM < 0) {
    if (currentRightPWM > 0) {
      currentRightPWM--;
      setRightPWM(currentRightPWM);
    }
    if (currentLeftPWM < 0) {
      currentLeftPWM++;
      setLeftPWM(currentLeftPWM);
    }
    delay(1);
  }
  setLeftPWM(0);
  setRightPWM(0);
  //  prevLeftTicks -= leftTicks;
  //  prevRightTicks -= rightTicks;
  leftTicks = 0;
  rightTicks = 0;
  delay(400);
  //    // Needs to deccelerate for the motors to stop correctly
  //  for (int i = turnSpeed; i >= 0; --i) {
  //    setLeftPWM(i);
  //    setRightPWM(-i);
  //  }
}

// If mouse starts facing a wall, this turns in the first cell to face the opening
void rightTurnFirstCell() {
  int i = 0;
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
  pivotTurnRight90();
  rightTicks = 0;
  leftTicks = 0;
  prevRightTicks = 0;
  prevLeftTicks = 0;
  leftSpeed = 0;
  rightSpeed = 0;
}
