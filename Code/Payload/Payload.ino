/****************************************************************
 * Sensor Libraries used:
 *    ICM 20948 Arduino Library by Owen Lyke @ SparkFun Electronics
 *    Adafruit PT100/P1000 RTD Sensor w/MAX31865 library by Limor Fried/Ladyada @ Adafruit Industries
 * 
 ***************************************************************/
#include "TeensyThreads.h"
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <Adafruit_MAX31865.h>
#include <SD.h>
#include <TimeLib.h>


// SparkFun 9DoF ICM_20948 IMU     
#define AD0_VAL   1     // The value of the last bit of the I2C address. On the SparkFun 9DoF IMU breakout the default is 1, and when 
ICM_20948_I2C myICM;    // create an ICM_20948_I2C object


// Adafruit_MAX31865 RTD 
#define RREF      430.0 // The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RNOMINAL  100.0 // 'nominal' 0-degrees-C resistance of the sensor 100.0 for PT100, 1000.0 for PT1000
#define CS_PIN1 10
#define CS_PIN2 37
#define CS_PIN3 36

Adafruit_MAX31865 thermo1 = Adafruit_MAX31865(CS_PIN1);
Adafruit_MAX31865 thermo2 = Adafruit_MAX31865(CS_PIN2);
Adafruit_MAX31865 thermo3 = Adafruit_MAX31865(CS_PIN3);


// Log Data 
#define IMU_CSV_NAME    "IMU.csv"
#define IMU_HEADER_CSV  "DateTime,Scaled_Acc_X_(mg),Scaled_Acc_Y_(mg),Scaled_Acc_Z_(mg),Gyr_X_(DPS),Gyr_Y_(DPS),Gyr_Z_(DPS),Mag_X_(uT),Mag_Y_(uT),Mag_Z_(uT),Tmp_(C)\n"
#define RTD_CSV_NAME    "RTD.csv"
#define RTD_HEADER_CSV  "DateTime,RTD1_(C),RTD2_(C),RTD3_(C)\n"


// Period for polling sensor data 
#define RTD_SAMPLE_PERIOD 2000
#define IMU_SAMPLE_PERIOD 30
// wrap around after 50 days
uint32_t RTD_TIME = 0; 
uint32_t IMU_TIME = 0; 


// Queue Size, vars for RTD and IMU data 
#define MAX_RTD 30
#define MAX_IMU 30

volatile int FRONT_RTD = 0;
volatile int REAR_RTD = -1;
volatile int FRONT_IMU = 0;
volatile int REAR_IMU = -1;

// mutex for RTD and IMU Queues
std::mutex m_rtd;
std::mutex m_imu;

struct rtd
{
  String date;   
  float t1; 
  float t2;
  float t3;
  bool error;
};

// RTD Queue
struct rtd QUEUE_RTD[MAX_RTD];


// FIFO First In First Out 
void enqueue_rtd(struct rtd temps)
{
  std::lock_guard<std::mutex> lock(m_rtd); // lock on creation
  if(REAR_RTD == MAX_RTD-1)
  {
      //Serial.println("RTD QUEUE IS FULL!");
      // wrap around overwrite front of queue
      FRONT_RTD = 0; 
      REAR_RTD = -1;
  }

  // increment rear then assign value to index
  QUEUE_RTD[++REAR_RTD] = temps; 
} // unlock at destruction


struct rtd dequeue_rtd()
{ 
  std::lock_guard<std::mutex> lock(m_rtd); // lock on creation
  struct rtd temps = {-1,-1,-1,-1,true};

  if(REAR_RTD == -1 || FRONT_RTD > REAR_RTD)
  {
      //Serial.println("RTD QUEUE IS EMPTY!");
      return temps;
  }

  temps = QUEUE_RTD[FRONT_RTD];
  FRONT_RTD++;
  return temps;
} // unlock at destruction


bool is_rtd_queue_empty()
{
  std::lock_guard<std::mutex> lock(m_rtd); // lock on creation
  if(REAR_RTD == -1 || FRONT_RTD > REAR_RTD)
    return true;
  else
    return false; 
} // unlock at destruction


struct imu
{
  String date;   
 
  // Scaled_Acc
  float acc_x; 
  float acc_y;
  float acc_z;
  // Gyr
  float gyr_x;
  float gyr_y;
  float gyr_z;
  // Mag
  float mag_x;
  float mag_y;
  float mag_z;
  //temp
  float tmp; 
  
  bool error;
};

// IMU Queue
struct imu QUEUE_IMU[MAX_IMU];


// FIFO First In First Out 
void enqueue_imu(struct imu sensor)
{
  std::lock_guard<std::mutex> lock(m_imu); // lock on creation
  if(REAR_IMU == MAX_IMU-1)
  {
      Serial.println("IMU QUEUE IS FULL!");
      // wrap around overwrite front of queue
      FRONT_IMU = 0; 
      REAR_IMU = -1;
  }

  // increment rear then assign value to index
  QUEUE_IMU[++REAR_IMU] = sensor; 
} // unlock at destruction


struct imu dequeue_imu()
{
  std::lock_guard<std::mutex> lock(m_imu); // lock on creation
  struct imu sensor = {-1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,true};
  if(REAR_IMU == -1 || FRONT_IMU > REAR_IMU)
  {   
      m_imu.unlock();
      Serial.println("IMU QUEUE IS EMPTY!");
      return sensor;
  }
  
  sensor = QUEUE_IMU[FRONT_IMU];
  FRONT_IMU++;
  return sensor;
} // unlock at destruction


bool is_imu_queue_empty()
{
  std::lock_guard<std::mutex> lock(m_imu); // lock on creation
  if(REAR_IMU == -1 || FRONT_IMU > REAR_IMU)
  {
    return true;
  }
  else
  {
    return false;
  }
} // unlock at destruction


bool Init_RTD(bool flag)
{
  if(flag)
  {
    Serial.println("Adafruit MAX31865 PT100 Sensor Test!");
    thermo1.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
    thermo2.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
    thermo3.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
    return true;
  }

  return false;
} // unlock at destruction


bool Init_Imu(bool flag)
{
  if (flag)
  {
    Serial.begin(115200);
    // wait for Serial 
    while(!Serial)
    {
      Serial.println("Waiting for Serial to initialize..."); 
    }
    
    Wire.begin();
    Wire.setClock(400000);

    bool initialized = false;
    while( !initialized )
    {
      myICM.begin( Wire, AD0_VAL );
  
      Serial.print( F("Initialization of the sensor returned: ") );
      Serial.println( myICM.statusString() );
      if( myICM.status != ICM_20948_Stat_Ok )
      {
        Serial.println( "Trying again..." );
        delay(500);
      }
      else
      {
        initialized = true;
      }
    }

  return initialized;
  }
  
  return false;
}


void Init_Sd_Card(bool flag)
{
  // wait for Serial Monitor to connect. Needed for native USB port boards only:
  if (flag)
  {
    while (!Serial)
    {
      Serial.println("Waiting for SD card to initialize..."); 
    }
  
    if (!SD.begin(BUILTIN_SDCARD)) 
    {
      Serial.println("initialization failed. Things to check:");
      Serial.println("1. is a card inserted?");
      Serial.println("2. is your wiring correct?");
      Serial.println("Note: press reset or reopen this serial monitor after fixing your issue!");
      while (true);
    }
  
    // write header of csv
    LogToCSV(IMU_HEADER_CSV, IMU_CSV_NAME);
    LogToCSV(RTD_HEADER_CSV, RTD_CSV_NAME);
    Serial.println("initialization done."); 
  }
}


void setup() 
{
  Serial.begin(115200);

  bool rtd_is_initialized = Init_RTD(true);
  bool imu_is_initialized = Init_Imu(true);
  // wait for 1.5 sec
  delay(1500);
  Init_Sd_Card(true); 

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
  if (millis() >= IMU_TIME + IMU_SAMPLE_PERIOD)
  {
    Serial.println("  IMULogger");
    LogIMU(IMU_CSV_NAME);
    IMU_TIME += IMU_SAMPLE_PERIOD;
  }

  // log every "RTD_SAMPLE_PERIOD" ms
  if (millis() >= RTD_TIME + RTD_SAMPLE_PERIOD)
  {
    Serial.println("  RTDLogger");
    LogRTD(RTD_CSV_NAME);
    RTD_TIME += RTD_SAMPLE_PERIOD;
  }
}


void PollIMU()
{

  while(true)
  {
    Serial.println("PollIMU");
    if( myICM.dataReady() )
    {
      myICM.getAGMT();                
      GetAGMTstruct(myICM.agmt);
      threads.delay(IMU_SAMPLE_PERIOD);
    }
    else
    {
      Serial.println("Waiting for data");
      threads.delay(500);
      //delay(500);
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
    Serial.println("PollRTD");
    t1 = GetTemp(&thermo1);
    t2 = GetTemp(&thermo2);
    t3 = GetTemp(&thermo3);
    date = TimeStr();
    struct rtd temps = {date,t1,t2,t3,false};
    enqueue_rtd(temps);
    threads.delay(RTD_SAMPLE_PERIOD);
    threads.yield();
  }
} 


float GetTemp(Adafruit_MAX31865 *thermo)
{
  uint16_t rtd = thermo->readRTD();
  float temperature = thermo->temperature(RNOMINAL, RREF);

  // Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  
  /*
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  Serial.print("Temperature = "); Serial.println(thermo->temperature(RNOMINAL, RREF));
  Serial.print("RTD: "); 
  */

  // Check and print any faults
  /*
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
  */
  //delay(1000);
  return temperature;
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


String LogFormattedFloat(float val, uint8_t leading, uint8_t decimals){
  float aval = abs(val);
  String s = "";
  if(val < 0){
    s = s + "-";
  }else{
    s = s + " ";
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
      s = s + "0";
    }else{
      break;
    }
  }
  if(val < 0){
    //Serial.print(-val, decimals);
    s = s + String(-val, decimals);
  }else{
    //Serial.print(val, decimals);
    s = s + String(val, decimals);
  }
  return s;
}


String TimeStr()
{
  // yyyy-mm-dd hh:mm:ss.ms
  int num = millis()%1000;
  String mill;
  
  if(num < 10)
    mill = "0" + String(num);
  if(num < 100)
    mill = "0" + String(num);
  else
    mill = String(num);
  
  String datetime = String(year()) + "-" + String(month()) + "-" + String(day()) + " " + String(hour()) + ":" + String(minute()) + ":" + String(second()) + "." + mill;
  return datetime;
}


// "DateTime,Scaled_Acc_X_(mg),Scaled_Acc_Y_(mg),Scaled_Acc_Z_(mg),Gyr_X_(DPS),Gyr_Y_(DPS),Gyr_Z_(DPS),Mag_X_(uT),Mag_Y_(uT),Mag_Z_(uT),Tmp_(C)\n"
void GetAGMTstruct(ICM_20948_AGMT_t agmt)
{
  String date = TimeStr();
  struct imu sensor = {date,myICM.accX(),myICM.accY(),myICM.accZ(),myICM.gyrX(),myICM.gyrY(),myICM.gyrZ(),myICM.magX(),myICM.magY(),myICM.magZ(),myICM.temp()};
  enqueue_imu(sensor);
} 


void LogRTD(const char *csv_name)
{
    struct rtd temps = dequeue_rtd();
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
  
        Serial.println("                  Logging RTD !!!");  
      }
      // if the file isn't open, pop up an error:
      else 
        Serial.println("error opening RTD_CSV_NAME");
    }
} 


void LogIMU(const char *csv_name)
{
  while(!is_imu_queue_empty())
  {
    //Serial.println("LogIMU while loop");
    struct imu sensor = dequeue_imu();
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
    
        Serial.println("      Logging IMU!!!");
      }
      // if the file isn't open, pop up an error:
      else 
        Serial.println("error opening IMU_CSV_NAME");
    }
  }
} 
