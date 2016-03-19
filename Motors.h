#pragma once

//Left and right may need to be flipped
const int ML1 = 21;
const int ML2 = 5; //63,62, MLA, MRB
const int MR1 = 20; //61, 64, MRA,MLB
const int MR2 = 6;

//Left and right may need to be flipped
const int ER1 = 29;
const int ER2 = 24;
const int EL1 = 26;
const int EL2 = 31;

// Motor Pins
const int motorPins[] = {MR1, MR2, ML1, ML2};

// Encoder Pins
const int encoderPins[] = {ER1, ER2, EL1, EL2};

int currentRightPWM = 0;
int currentLeftPWM = 0;

// Encoder Ticks
volatile int rightTicks = 0;
volatile int leftTicks = 0;

int prevRightTicks;
int prevLeftTicks;
