#include "Imu.h"


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
