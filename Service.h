#ifndef _SERVICE_H
#define _SERVICE_H
#include <Arduino.h>
#define DEBUG 0

/*
      ----------########---------
     |1 GND | 3  DATA PIN6  |5 CLK  PIN5 |
     |____________________________
     |
     |2 5V  | 4   PROG      |6  CSN  PIN7|
     ----------------------------

     Encoder Black (GND)   1 <-> Arduino GND
     Encoder Red (Power +) 2 <-> Arduino +5V
     Encoder Blue (Clock)  5 <-> Arduino Pin 5
     Encoder Green (DATA)  3 <-> Arduino Pin 6
     Encoder Yellow (CS)   6 <-> Arduino Pin 7

*/

const int CLOCK_PIN = 5; // Blue Pin
const int DATA_PIN = 6; // Green Pin
const int CS_PIN = 7; // Yellow Pin
const int BIT_COUNT = 10; // 12 Bit Mode

void F_giro(float , float );
bool Rotazione(float , float );
#endif
