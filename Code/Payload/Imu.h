#ifndef IMU_H
#define IMU_H

#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "TeensyThreads.h"
#include <TimeLib.h>
#include <Arduino.h>


// SparkFun 9DoF ICM_20948 IMU     
#define AD0_VAL   1     // The value of the last bit of the I2C address. On the SparkFun 9DoF IMU breakout the default is 1, and when 

// log data 
#define IMU_CSV_NAME    "IMU.csv"
#define IMU_HEADER_CSV  "DateTime,Scaled_Acc_X_(mg),Scaled_Acc_Y_(mg),Scaled_Acc_Z_(mg),IIR_Acc_X_(mg),IIR_Acc_Y_(mg),IIR_Acc_Z_(mg),Gyr_X_(DPS),Gyr_Y_(DPS),Gyr_Z_(DPS),Mag_X_(uT),Mag_Y_(uT),Mag_Z_(uT),Tmp_(C)\n"

// Period for polling sensor data 
#define IMU_SAMPLE_PERIOD 12.5f

String LogFormattedFloat(float val, uint8_t leading, uint8_t decimals);
String TimeStr();

#endif 
