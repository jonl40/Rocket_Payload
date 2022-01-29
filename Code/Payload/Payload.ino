/****************************************************************
 * Sensor Libraries used:
 *    ICM 20948 Arduino Library by Owen Lyke @ SparkFun Electronics
 *    Adafruit PT100/P1000 RTD Sensor w/MAX31865 library by Limor Fried/Ladyada @ Adafruit Industries
 * 
 ***************************************************************/
#include "Queue.h"
#include "Imu.h"
#include "Rtd.h"
#include "Filter.h"

#include "TeensyThreads.h"
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <Adafruit_MAX31865.h>
#include <SD.h>
#include <TimeLib.h>

#define SERIAL_MONITOR false
#define SD_INSERTED true 
#define START_RTD true
#define START_IMU true 

#define IIR_ACC_X_ALPHA 0.1f
#define IIR_ACC_Y_ALPHA 0.1f
#define IIR_ACC_Z_ALPHA 0.1f

// SparkFun 9DoF ICM_20948 IMU     
ICM_20948_I2C myICM;    // create an ICM_20948_I2C object


// Adafruit_MAX31865 RTD 
Adafruit_MAX31865 thermo1 = Adafruit_MAX31865(CS_PIN1);
Adafruit_MAX31865 thermo2 = Adafruit_MAX31865(CS_PIN2);
Adafruit_MAX31865 thermo3 = Adafruit_MAX31865(CS_PIN3);


// wrap around after 50 days
uint32_t RTD_TIME = 0; 
uint32_t IMU_TIME = 0; 


// queue for polling and logging sensor data 
Queue_rtd rtd_queue; 
Queue_imu imu_queue; 
// Queue_imu imu_queue_fir; 
Queue_imu imu_queue_iir; 


/*
// FIR Filters 
FIR_Filter fir_acc_x; 
FIR_Filter fir_acc_y; 
FIR_Filter fir_acc_z; 
*/


// IIR Filters 
IIR_Filter iir_acc_x(IIR_ACC_X_ALPHA, 0); 
IIR_Filter iir_acc_y(IIR_ACC_Y_ALPHA, 0); 
IIR_Filter iir_acc_z(IIR_ACC_Z_ALPHA, 1000); 


bool Init_RTD(bool flag)
{
  if(flag)
  {
    thermo1.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
    thermo2.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
    thermo3.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
    Serial.println("Adafruit MAX31865 PT100 Sensors initialized");
    return true;
  }

  return false;
} // unlock at destruction


bool Init_Imu(bool flag)
{
  if (flag)
  {
    // wait for Serial 
    // remove !Serial code does not run when powered on unless connected to usb 
    while(!Serial)
    {
      Serial.println("Waiting for Serial to initialize..."); 
      delay(1000);
    }
    
    Wire.begin();
    Wire.setClock(400000);

    bool initialized = false;
    while( !initialized )
    {
      myICM.begin( Wire, AD0_VAL );
  
      //Serial.print( F("Initialization of the sensor returned: ") );
      Serial.println( myICM.statusString() );
      if( myICM.status != ICM_20948_Stat_Ok )
      {
        Serial.println("Waiting for IMU to initialize..."); 
        delay(500);
      }
      else
      {
        Serial.println("IMU initialized");
        initialized = true;
      }
    }

  return initialized;
  }
  
  return false;
}


void LogToCSV(String dataString, const char* csv_name)
{
 // log data 
 File dataFile = SD.open(csv_name, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
  dataFile.print(dataString);
  dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening file");
  }
}


void Init_Sd_Card(bool flag)
{
  // wait for Serial Monitor to connect. Needed for native USB port boards only:
  if (flag)
  {
    // wait for serial to get ready 
    while (!Serial)
    {
      Serial.println("Waiting for SD card to initialize..."); 
      delay(1000);
    }

    // wait for SD card to get ready 
    while (!SD.begin(BUILTIN_SDCARD))
    {
      Serial.println("SD card initialization failed. Check:");
      Serial.println("1. is SD card inserted?");
      Serial.println("2. is your wiring correct?");
      Serial.println("Note: press reset or reopen this serial monitor after fixing your issue!");
      Serial.println("");
      delay(1000);
    }
  
    // write header of csv
    LogToCSV(IMU_HEADER_CSV, IMU_CSV_NAME);
    LogToCSV(RTD_HEADER_CSV, RTD_CSV_NAME);
    Serial.println("SD Card initialization done"); 
  }
}


void setup() 
{
  Serial.begin(115200);

  bool rtd_is_initialized = Init_RTD(START_RTD);
  bool imu_is_initialized = Init_Imu(START_IMU);
  // wait for 1.5 sec
  delay(1500);
  Init_Sd_Card(SD_INSERTED); 

  // create thread to poll IMU
  if (imu_is_initialized)
  {
      std::thread t_imu(PollIMU);
      t_imu.detach();
  }

  // create thread to poll RTD 
  if (rtd_is_initialized)
  {
      std::thread t_rtd(PollRTD);
      t_rtd.detach();
  }

}


void loop() 
{
  // log every "IMU_SAMPLE_PERIOD" ms
  if (millis() >= IMU_TIME + IMU_SAMPLE_PERIOD && SD_INSERTED)
  {
    // Serial.println("  IMULogger");
    LogIMU(IMU_CSV_NAME);
    IMU_TIME += IMU_SAMPLE_PERIOD;
  }

  // log every "RTD_SAMPLE_PERIOD" ms
  if (millis() >= RTD_TIME + RTD_SAMPLE_PERIOD && SD_INSERTED)
  {
    // Serial.println("  RTDLogger");
    LogRTD(RTD_CSV_NAME);
    RTD_TIME += RTD_SAMPLE_PERIOD;
  }
}


void PollIMU()
{

  while(true)
  {
    // Serial.println("PollIMU");
    if( myICM.dataReady() )
    {
      myICM.getAGMT();                
      String date = TimeStr();
      // "DateTime,Scaled_Acc_X_(mg),Scaled_Acc_Y_(mg),Scaled_Acc_Z_(mg),Gyr_X_(DPS),Gyr_Y_(DPS),Gyr_Z_(DPS),Mag_X_(uT),Mag_Y_(uT),Mag_Z_(uT),Tmp_(C)\n"
      float acc_x = myICM.accX();
      float acc_y = myICM.accY();
      float acc_z = myICM.accZ();
      struct imu sensor = {date,
                          acc_x,                    // Scaled_Acc
                          acc_y,
                          acc_z,
                          iir_acc_x.update(acc_x),  // Scaled_Acc IIR Filtered
                          iir_acc_y.update(acc_y),
                          iir_acc_z.update(acc_z),
                          myICM.gyrX(),         // Gyr
                          myICM.gyrY(),
                          myICM.gyrZ(),
                          myICM.magX(),         // Mag
                          myICM.magY(),
                          myICM.magZ(),
                          myICM.temp()};        // temp
      imu_queue.enqueue(sensor);
      
      // serial plotter 
      if (SERIAL_MONITOR)
      {
        // raw data 
        Serial.print(sensor.acc_x);
        Serial.print("\t");
        Serial.print(sensor.acc_y);
        Serial.print("\t");
        Serial.print(sensor.acc_z);

        // filtered data 
        Serial.print(sensor.acc_iir_x);
        Serial.print("\t");
        Serial.print(sensor.acc_iir_y);
        Serial.print("\t");
        Serial.println(sensor.acc_iir_z);
      }

      // threads.delay(IMU_SAMPLE_PERIOD);
      
    }
    
    else
    {
      Serial.println("Waiting for data");
      threads.delay(500);
    }
    threads.yield();
  }
} 


void PollRTD()
{

  float t1, t2, t3;
  String date; 
  
  while(true)
  {
    // Serial.println("PollRTD");
    t1 = GetTemp(&thermo1);
    t2 = GetTemp(&thermo2);
    t3 = GetTemp(&thermo3);
    date = TimeStr();
    struct rtd temps = {date,t1,t2,t3,false};
    rtd_queue.enqueue(temps);
    threads.delay(RTD_SAMPLE_PERIOD);
    threads.yield();
  }
} 


void LogIMU(const char *csv_name)
{
  while(!imu_queue.empty())
  {
    // Serial.println("LogIMU while loop");
    // struct imu sensor = dequeue_imu();
    struct imu sensor = imu_queue.dequeue();
    if(sensor.error == false)
    {
      // log data 
      File dataFile = SD.open(csv_name, FILE_WRITE);

      if (dataFile) 
      {
        // yyyy-mm-dd hh:mm:ss
        dataFile.print(sensor.date);
        dataFile.print(",");
        // Scaled Acc (mg)
        dataFile.print(LogFormattedFloat( sensor.acc_x, 5, 2));
        dataFile.print(",");
        dataFile.print(LogFormattedFloat( sensor.acc_y, 5, 2));
        dataFile.print(",");
        dataFile.print(LogFormattedFloat( sensor.acc_z, 5, 2));
        dataFile.print(",");
        // Scaled_Acc IIR Filtered
        dataFile.print(LogFormattedFloat( sensor.acc_iir_x, 5, 2));
        dataFile.print(",");
        dataFile.print(LogFormattedFloat( sensor.acc_iir_y, 5, 2));
        dataFile.print(",");
        dataFile.print(LogFormattedFloat( sensor.acc_iir_z, 5, 2));
        dataFile.print(",");
        // gyr(DPS)
        dataFile.print(LogFormattedFloat( sensor.gyr_x, 5, 2));
        dataFile.print(",");
        dataFile.print(LogFormattedFloat( sensor.gyr_y, 5, 2));
        dataFile.print(",");
        dataFile.print(LogFormattedFloat( sensor.gyr_z, 5, 2));
        dataFile.print(",");
        // mag (uT)
        dataFile.print(LogFormattedFloat( sensor.mag_x, 5, 2));
        dataFile.print(",");
        dataFile.print(LogFormattedFloat( sensor.mag_y, 5, 2));
        dataFile.print(",");
        dataFile.print(LogFormattedFloat( sensor.mag_z, 5, 2));
        dataFile.print(",");
        // tmp (C)
        dataFile.print(LogFormattedFloat( sensor.tmp, 5, 2));
        dataFile.print("\n");
        dataFile.close();
    
        // Serial.println("      Logging IMU!!!");
      }
      // if the file isn't open, pop up an error:
      else 
        Serial.println("error opening IMU_CSV_NAME");
    }
  }
} 


void LogRTD(const char *csv_name)
{
    struct rtd temps = rtd_queue.dequeue();
    if(temps.error == false)
    {
      // log data 
      File dataFile = SD.open(csv_name, FILE_WRITE);
      // if the file is available, write to it:
      if (dataFile) 
      {
        // yyyy-mm-dd hh:mm:ss
        dataFile.print(temps.date);
        dataFile.print(",");
        dataFile.print(temps.t1);
        dataFile.print(",");
        dataFile.print(temps.t2);
        dataFile.print(",");
        dataFile.print(temps.t3);
        dataFile.print("\n");
        dataFile.close();
  
        // Serial.println("                  Logging RTD !!!");  
      }
      // if the file isn't open, pop up an error:
      else 
        Serial.println("error opening RTD_CSV_NAME");
    }
} 
