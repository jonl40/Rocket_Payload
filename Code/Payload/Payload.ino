/****************************************************************
 * Example1_Basics.ino
 * ICM 20948 Arduino Library Demo 
 * Use the default configuration to stream 9-axis IMU data
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 * 
 * This code is beerware; if you see me (or any other SparkFun employee) at the
 * local, and you've found our code helpful, please buy us a round!
 * 
 * Distributed as-is; no warranty is given.
 ***************************************************************/
// #include "TeensyThreads.h"
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <SD.h>
#include <TimeLib.h>

//const int chipSelect = 10;
#define IMU_CSV_NAME "imu.csv"
#define IMU_HEADER_CSV "DateTime,Scaled_Acc_X_(mg),Scaled_Acc_Y_(mg),Scaled_Acc_Z_(mg),Gyr_X_(DPS),Gyr_Y_(DPS),Gyr_Z_(DPS),Mag_X_(uT),Mag_Y_(uT),Mag_Z_(uT),Tmp_(C)\n"

//#define USE_SPI       // Uncomment this to use SPI


#define SERIAL_PORT Serial

#define SPI_PORT SPI    // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2        // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL   1     // The value of the last bit of the I2C address. 
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when 
                        // the ADR jumper is closed the value becomes 0

#ifdef USE_SPI
  ICM_20948_SPI myICM;  // If using SPI create an ICM_20948_SPI object
#else
  ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
#endif


bool Init_Imu(bool flag)
{
  if (flag)
  {
    SERIAL_PORT.begin(115200);
    while(!SERIAL_PORT){};
  
    #ifdef USE_SPI
        SPI_PORT.begin();
    #else
        WIRE_PORT.begin();
        WIRE_PORT.setClock(400000);
    #endif
      
      bool initialized = false;
      while( !initialized ){
    
    #ifdef USE_SPI
        myICM.begin( CS_PIN, SPI_PORT ); 
    #else
        myICM.begin( WIRE_PORT, AD0_VAL );
    #endif
    
        SERIAL_PORT.print( F("Initialization of the sensor returned: ") );
        SERIAL_PORT.println( myICM.statusString() );
        if( myICM.status != ICM_20948_Stat_Ok ){
          SERIAL_PORT.println( "Trying again..." );
          delay(500);
        }else{
          initialized = true;
        }
      }

  return initialized;
  }

  return false;

}


void Init_Sd_Card(bool flag, bool imu_flag)
{
  // wait for Serial Monitor to connect. Needed for native USB port boards only:
  if (flag)
  {
  while (!Serial);
  Serial.print("Initializing SD card...");

    if (!SD.begin(BUILTIN_SDCARD)) 
    {
      Serial.println("initialization failed. Things to check:");
      Serial.println("1. is a card inserted?");
      Serial.println("2. is your wiring correct?");
      Serial.println("Note: press reset or reopen this serial monitor after fixing your issue!");
      while (true);
    }
    // write header of csv
    Logger(IMU_HEADER_CSV);

    /*
    if(imu_flag) 
    {
      threads.addThread(ImuThreadLogger); 
    }
    */

  Serial.println("initialization done."); 
  
  }
}


void setup() {
  SERIAL_PORT.begin(115200);
  
  bool imu_is_initialized = Init_Imu(true);
  Init_Sd_Card(true, imu_is_initialized); 

}

void loop() {

  if( myICM.dataReady() )
  {
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
    //printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    //printScaledAGMT( myICM.agmt);   // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    LogCsvScaledAGMT(myICM.agmt);
    delay(30);

  }
  else
  {
    Serial.println("Waiting for data");
    delay(500);
  }

}

/*
void ImuThreadLogger() 
{
  while(1)
  {
    if( myICM.dataReady() )
    {
      myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
      //printScaledAGMT( myICM.agmt);   // This function takes into account the sclae settings from when the measurement was made to calculate the values with units
      LogCsvScaledAGMT(myICM.agmt);
      threads.delay(30);
  }
    else
    {
      Serial.println("Thread: Waiting for data");
      threads.delay(500);
    }
    threads.yield();
  }
}
*/


void Logger(String dataString)
{
 // log data 
 File dataFile = SD.open(IMU_CSV_NAME, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
  dataFile.print(dataString);
  dataFile.close();
  // print to the serial port too:
  //Serial.print(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening IMU_CSV_NAME");
  }
}


void LogFormattedFloat(float val, uint8_t leading, uint8_t decimals){
  float aval = abs(val);
  if(val < 0){
    Logger("-");
  }else{
    Logger(" ");
  }
  for( uint8_t indi = 0; indi < leading; indi++ ){
    uint32_t tenpow = 0;
    if( indi < (leading-1) ){
      tenpow = 1;
    }
    for(uint8_t c = 0; c < (leading-1-indi); c++){
      tenpow *= 10;
    }
    if( aval < tenpow){
      Logger("0");
    }else{
      break;
    }
  }
  if(val < 0){
    //SERIAL_PORT.print(-val, decimals);
    Logger(String(-val, decimals));
  }else{
    //SERIAL_PORT.print(val, decimals);
    Logger(String(val, decimals));
  }
}


// "DateTime,Scaled_Acc_X_(mg),Scaled_Acc_Y_(mg),Scaled_Acc_Z_(mg),Gyr_X_(DPS),Gyr_Y_(DPS),Gyr_Z_(DPS),Mag_X_(uT),Mag_Y_(uT),Mag_Z_(uT),Tmp_(C)"
void LogCsvScaledAGMT(ICM_20948_AGMT_t agmt)
{
  // log date time 
  Logger(year()); 
  Logger("-");
  Logger(month());
  Logger("-");
  Logger(day());
  Logger(" ");
  Logger(hour());
  Logger(":");
  Logger(minute());
  Logger(":");
  Logger(second());
  Logger(",");
  // Scaled Acc (mg)
  LogFormattedFloat( myICM.accX(), 5, 2 );
  Logger(",");
  LogFormattedFloat( myICM.accY(), 5, 2 );
  Logger(",");
  LogFormattedFloat( myICM.accZ(), 5, 2 );
  // gyr(DPS)
  Logger(", ");
  LogFormattedFloat( myICM.gyrX(), 5, 2 );
  Logger(",");
  LogFormattedFloat( myICM.gyrY(), 5, 2 );
  Logger(",");
  LogFormattedFloat( myICM.gyrZ(), 5, 2 );
  // mag (uT)
  Logger(",");
  LogFormattedFloat( myICM.magX(), 5, 2 );
  Logger(",");
  LogFormattedFloat( myICM.magY(), 5, 2 );
  Logger(",");
  LogFormattedFloat( myICM.magZ(), 5, 2 );
  // tmp (C)
  Logger(",");
  LogFormattedFloat( myICM.temp(), 5, 2 );
  Logger("\n");
  Serial.println("Logging");
}


// Below here are some helper functions to print the data nicely!
void printPaddedInt16b( int16_t val ){
  if(val > 0){
    SERIAL_PORT.print(" ");
    if(val < 10000){ SERIAL_PORT.print("0"); }
    if(val < 1000 ){ SERIAL_PORT.print("0"); }
    if(val < 100  ){ SERIAL_PORT.print("0"); }
    if(val < 10   ){ SERIAL_PORT.print("0"); }
  }else{
    SERIAL_PORT.print("-");
    if(abs(val) < 10000){ SERIAL_PORT.print("0"); }
    if(abs(val) < 1000 ){ SERIAL_PORT.print("0"); }
    if(abs(val) < 100  ){ SERIAL_PORT.print("0"); }
    if(abs(val) < 10   ){ SERIAL_PORT.print("0"); }
  }
  SERIAL_PORT.print(abs(val));
}


void printFormattedFloat(float val, uint8_t leading, uint8_t decimals){
  float aval = abs(val);
  if(val < 0){
    SERIAL_PORT.print("-");
  }else{
    SERIAL_PORT.print(" ");
  }
  for( uint8_t indi = 0; indi < leading; indi++ ){
    uint32_t tenpow = 0;
    if( indi < (leading-1) ){
      tenpow = 1;
    }
    for(uint8_t c = 0; c < (leading-1-indi); c++){
      tenpow *= 10;
    }
    if( aval < tenpow){
      SERIAL_PORT.print("0");
    }else{
      break;
    }
  }
  if(val < 0){
    SERIAL_PORT.print(-val, decimals);
  }else{
    SERIAL_PORT.print(val, decimals);
  }
}


void printScaledAGMT( ICM_20948_AGMT_t agmt){
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat( myICM.accX(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.accY(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.accZ(), 5, 2 );
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat( myICM.gyrX(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.gyrY(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.gyrZ(), 5, 2 );
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat( myICM.magX(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.magY(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.magZ(), 5, 2 );
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat( myICM.temp(), 5, 2 );
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}
