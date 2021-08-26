#ifndef RTD_H
#define RTD_H

#include <Adafruit_MAX31865.h>
#include "TeensyThreads.h"
#include <SD.h>
#include <Arduino.h>

// Adafruit_MAX31865 RTD 
#define RREF      430.0 // The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RNOMINAL  100.0 // 'nominal' 0-degrees-C resistance of the sensor 100.0 for PT100, 1000.0 for PT1000
#define CS_PIN1 10
#define CS_PIN2 37
#define CS_PIN3 36

// log data 
#define RTD_CSV_NAME    "RTD.csv"
#define RTD_HEADER_CSV  "DateTime,RTD1_(C),RTD2_(C),RTD3_(C)\n"

// Period for polling sensor data 
#define RTD_SAMPLE_PERIOD 2000

float GetTemp(Adafruit_MAX31865 *thermo);

#endif 
