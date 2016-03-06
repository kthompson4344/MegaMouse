#pragma once

#define left 0
#define diagl 1
#define lf 2
#define rf 3
#define right 4
#define diagr 5

// IR Pair Pins
const int TX[6] = {25, 22,   9,  15,  32, 22};
const int RX[6] = {A4, A17, A16, A14, A5, A3};
const int diagHighPower = 23;

volatile int leftSensor;
volatile int rightSensor;
volatile int rightMiddleValue;
volatile int leftMiddleValue;
volatile int leftFront;
volatile int rightFront;
