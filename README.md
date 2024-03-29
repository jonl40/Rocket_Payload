# Rocket_Payload
C/C++ code for teensy 4.1 microcontroller to poll and log sensor data (IMU, 3 RTDs) using multithreading   
Python pandas, matplotlib for data visualization  

## References / Filter Design  
https://github.com/pms67/HadesFCS/blob/master/Filtering/C%20Code/FIR.h  
https://www.youtube.com/watch?v=uNNNj9AZisM  
https://www.youtube.com/watch?v=QRMe02kzVkA  
http://t-filter.engineerjs.com/  

## Libraries Used 
Multithreading library by Fernando Trias https://github.com/ftrias/TeensyThreads  
ICM 20948 Arduino Library by Owen Lyke @ SparkFun Electronics  https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary  
Adafruit PT100/P1000 RTD Sensor w/MAX31865 library by Limor Fried/Ladyada @ Adafruit Industries  

## Set up
https://www.arduino.cc/en/software    
https://www.arduino.cc/en/Guide/Linux  
https://playground.arduino.cc/Linux/All/#Permission  
https://www.pjrc.com/teensy/td_download.html    
In Arduino IDE Tools -> manage libraries search and install Adafruit_MAX31865  
Install ICM-20948 https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary  
OR Install ICM-20948 with http://librarymanager/All#SparkFun_ICM_20948_IMU in IDE     
Install Multithreading library https://github.com/ftrias/TeensyThreads    
In Arduino IDE Tools -> Board -> Teensyduino -> Teensy 4.1  
In Arduino IDE Tools -> Port -> "port connected to Teensy 4.1"    




# RTD Plot 
![RTD](https://user-images.githubusercontent.com/33404359/129466876-585bf53b-315d-4299-883d-0c9c754bc83a.JPG)

# IMU Plot
![IMU](https://user-images.githubusercontent.com/33404359/129467033-4e8aa38c-db83-4ecd-926f-7d3d571b0804.jpg)

# Acceleration Plot 
![ACC](https://user-images.githubusercontent.com/33404359/129467042-5e4564a6-8920-44d4-bbf5-e866baaaad65.JPG)

# Gyro Plot 
![GYR](https://user-images.githubusercontent.com/33404359/129467062-f5bd9b1b-12a0-4f54-b63e-1ebc8cd224ec.jpg)

# Mag Plot
![MAG](https://user-images.githubusercontent.com/33404359/129467082-6518a433-2a82-4ee8-b767-40426fbdc436.jpg)

# Schematic 
![Schematic_Payload_2021-08-14](https://user-images.githubusercontent.com/33404359/129466893-313b346b-e958-4478-bb6f-6e1c83d26b1c.png)


