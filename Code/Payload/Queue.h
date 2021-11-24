#ifndef QUEUE_H
#define QUEUE_H

#include <Arduino.h>
#include "TeensyThreads.h"


// Queue Size, vars for RTD and IMU data 
#define MAX_RTD 30
#define MAX_IMU 200


struct rtd
{
  String date;   
  float t1; 
  float t2;
  float t3;
  bool error;
};


struct imu
{
  String date;   
 
  // Scaled_Acc
  float acc_x; 
  float acc_y;
  float acc_z;
  // Scaled_Acc IIR Filtered
  float acc_iir_x; 
  float acc_iir_y;
  float acc_iir_z;
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


// Base class
class Queue {
  
  protected:
    std::mutex m;
    int front;
    int rear;

  public:
    Queue();
    bool empty();
   
}; 

// Derived class
class Queue_rtd: public Queue
{

  public: 
    Queue_rtd();
    void enqueue(struct rtd temps);
    struct rtd dequeue();
    
  private:
    struct rtd buff[MAX_RTD];
    
};

// Derived class
class Queue_imu: public Queue
{

  public: 
    Queue_imu();
    void enqueue(struct imu sensor);
    struct imu dequeue();

  private:
    struct imu buff[MAX_IMU];
    
};

#endif 
