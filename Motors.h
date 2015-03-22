#pragma once

const int ML1 = 20;
const int ML2 = 6;
const int MR1 = 21;
const int MR2 = 5;
const int ER1 = 8;
const int ER2 = 7;
const int EL1 = 26;
const int EL2 = 31;

// Motor Pins
const int motorPins[] = {MR1, MR2, ML1, ML2};

// Encoder Pins
const int encoderPins[] = {ER1, ER2, EL1, EL2};

int currentRightPWM = 0;
int currentLeftPWM = 0;

// Encoder Ticks
int rightTicks = 0;
int leftTicks = 0;
