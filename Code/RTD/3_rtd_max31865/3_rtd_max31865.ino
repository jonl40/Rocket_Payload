/*************************************************** 
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Adafruit_MAX31865.h>
#include <SD.h>
#include <TimeLib.h>

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0
#define CS_PIN1 10
#define CS_PIN2 37
#define CS_PIN3 36
#define CSVNAME "RTD.csv"
#define CSVHEADER "DateTime,RTD1_(C),RTD2_(C),RTD3_(C)"

Adafruit_MAX31865 thermo1 = Adafruit_MAX31865(CS_PIN1);
Adafruit_MAX31865 thermo2 = Adafruit_MAX31865(CS_PIN2);
Adafruit_MAX31865 thermo3 = Adafruit_MAX31865(CS_PIN3);

void Init_Sd_Card(bool start_sd)
{
  // wait for Serial Monitor to connect. Needed for native USB port boards only:
  if (start_sd)
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

  Serial.println("initialization done."); 
  
  }
}

// write header to csv file 
void Write_Header()
{
  File dataFile = SD.open(CSVNAME, FILE_WRITE);
  dataFile.println(CSVHEADER);
  dataFile.close();
}

void setup() 
{
  //Serial.begin(115200);
  Serial.begin(9600);

  Init_Sd_Card(true);
  Write_Header();
  
  Serial.println("Adafruit MAX31865 PT100 Sensor Test!");
  thermo1.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  thermo2.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  thermo3.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
}


void loop() 
{ 
  float t1, t2, t3;
  
  t1 = GetTemp(&thermo1, 1);
  t2 = GetTemp(&thermo2, 2);
  t3 = GetTemp(&thermo3, 3);
  Write_Temp_data(t1, t2, t3);
  delay(1000);
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

// write temperature data to csv file
void Write_Temp_data(float t1, float t2, float t3)
{
  File dataFile = SD.open(CSVNAME, FILE_WRITE);
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
  dataFile.print(",");

  // log rtd
  dataFile.print(t1);
  dataFile.print(",");
  dataFile.print(t2);
  dataFile.print(",");
  dataFile.print(t3);
  dataFile.println();
  
  dataFile.close();
}
