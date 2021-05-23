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
#include <Adafruit_MAX31865.h>
#include <SD.h>
#include <TimeLib.h>

//const int chipSelect = 10;
#define IMU_CSV_NAME "IMU.csv"
#define IMU_HEADER_CSV "DateTime,Scaled_Acc_X_(mg),Scaled_Acc_Y_(mg),Scaled_Acc_Z_(mg),Gyr_X_(DPS),Gyr_Y_(DPS),Gyr_Z_(DPS),Mag_X_(uT),Mag_Y_(uT),Mag_Z_(uT),Tmp_(C)\n"
#define RTD_CSV_NAME "RTD.csv"
#define RTD_HEADER_CSV "RTD1_(C),RTD2_(C),RTD3_(C)\n"

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

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000

#define RNOMINAL  100.0
#define CS_PIN1 10
#define CS_PIN2 37
#define CS_PIN3 36

Adafruit_MAX31865 thermo1 = Adafruit_MAX31865(CS_PIN1);
Adafruit_MAX31865 thermo2 = Adafruit_MAX31865(CS_PIN2);
Adafruit_MAX31865 thermo3 = Adafruit_MAX31865(CS_PIN3);


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
    LogToCSV(IMU_HEADER_CSV);
    Write_Header();

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

  Serial.println("Adafruit MAX31865 PT100 Sensor Test!");
  thermo1.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  thermo2.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  thermo3.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
}

void loop() {
  LogSensorData();
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

void LogSensorData()
{
  float t1, t2, t3;
  
  t1 = GetTemp(&thermo1, 1);
  t2 = GetTemp(&thermo2, 2);
  t3 = GetTemp(&thermo3, 3);
  LogTemp(t1, t2, t3);
  
  if( myICM.dataReady() )
  {
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
    //printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    //printScaledAGMT( myICM.agmt);   // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    LogTime();
    LogScaledAGMT(myICM.agmt);
    delay(30);
  }
  else
  {
    Serial.println("Waiting for data");
    delay(500);
  }
}


void LogToCSV(String dataString)
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
    LogToCSV("-");
  }else{
    LogToCSV(" ");
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
      LogToCSV("0");
    }else{
      break;
    }
  }
  if(val < 0){
    //SERIAL_PORT.print(-val, decimals);
    LogToCSV(String(-val, decimals));
  }else{
    //SERIAL_PORT.print(val, decimals);
    LogToCSV(String(val, decimals));
  }
}

void LogTime()
{
  // log date time 
  LogToCSV(year()); 
  LogToCSV("-");
  LogToCSV(month());
  LogToCSV("-");
  LogToCSV(day());
  LogToCSV(" ");
  LogToCSV(hour());
  LogToCSV(":");
  LogToCSV(minute());
  LogToCSV(":");
  LogToCSV(second());
  LogToCSV(".");
  LogToCSV(millis());
  LogToCSV(",");
}


// "Tmp_(C),Scaled_Acc_X_(mg),Scaled_Acc_Y_(mg),Scaled_Acc_Z_(mg),Gyr_X_(DPS),Gyr_Y_(DPS),Gyr_Z_(DPS),Mag_X_(uT),Mag_Y_(uT),Mag_Z_(uT)"
void LogScaledAGMT(ICM_20948_AGMT_t agmt)
{
  // Scaled Acc (mg)
  LogFormattedFloat( myICM.accX(), 5, 2 );
  LogToCSV(",");
  LogFormattedFloat( myICM.accY(), 5, 2 );
  LogToCSV(",");
  LogFormattedFloat( myICM.accZ(), 5, 2 );
  // gyr(DPS)
  LogToCSV(", ");
  LogFormattedFloat( myICM.gyrX(), 5, 2 );
  LogToCSV(",");
  LogFormattedFloat( myICM.gyrY(), 5, 2 );
  LogToCSV(",");
  LogFormattedFloat( myICM.gyrZ(), 5, 2 );
  // mag (uT)
  LogToCSV(",");
  LogFormattedFloat( myICM.magX(), 5, 2 );
  LogToCSV(",");
  LogFormattedFloat( myICM.magY(), 5, 2 );
  LogToCSV(",");
  LogFormattedFloat( myICM.magZ(), 5, 2 );
  // tmp (C)
  LogToCSV(",");
  LogFormattedFloat( myICM.temp(), 5, 2 );
  LogToCSV("\n");
  Serial.println("Logging IMU");
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

float GetTemp(Adafruit_MAX31865 *thermo, int num)
{
  uint16_t rtd = thermo->readRTD();
  float temperature = thermo->temperature(RNOMINAL, RREF);

  Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  Serial.print("Temperature = "); Serial.println(thermo->temperature(RNOMINAL, RREF));
  Serial.print("RTD: "); Serial.println(num); 

  // Check and print any faults
  uint8_t fault = thermo->readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo->clearFault();
  }
  Serial.println();
  return temperature;
  //delay(1000);
}


// write header to csv file 
void Write_Header()
{
  File dataFile = SD.open(RTD_CSV_NAME, FILE_WRITE);
  dataFile.println(RTD_HEADER_CSV);
  dataFile.close();
}

// write temperature data to csv file
void LogTemp(float t1, float t2, float t3)
{
  File dataFile = SD.open(RTD_CSV_NAME, FILE_WRITE);
  // log date time 
  dataFile.print(year()); 
  dataFile.print("-");
  dataFile.print(month());
  dataFile.print("-");
  dataFile.print(day());
  dataFile.print(" ");
  dataFile.print(hour());
  dataFile.print(":");
  dataFile.print(minute());
  dataFile.print(":");
  dataFile.print(second());
  dataFile.print(".");
  dataFile.print(millis());
  dataFile.print(",");

  // log rtd
  dataFile.print(t1);
  dataFile.print(",");
  dataFile.print(t2);
  dataFile.print(",");
  dataFile.print(t3);
  dataFile.println();
  Serial.println("Logging RTD");
  
  dataFile.close();
}
