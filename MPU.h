#ifndef _MPU_h_
#define _MPU_h_


/*
This is my first library and is and debugging and upgrade of code developed by Joop Brokking
Website: http://www.brokking.net/imu.html
Youtube: https://youtu.be/4BoIE8YQwM8
Version: 1.0 (May 5, 2016)
// Please support suscribing to his channel and website
*/

#include <inttypes.h>
#include <Wire.h>


class MPU{

public:

MPU();

void begin();
void read();
void calibrate(int n);
double getAngle(int a);
double getAcc();

private:

struct Acc{

long value[3];
long vector;
long cal;
double angle[3];

}acc;

struct Gyro{
  
long cal[3];
double spin[3];
double angle[3];

}gyro;

struct Output{
	
double angle[3];
double radial[3];       // not used yet

}output;

long temp;
long loopTimer;
boolean set;

};

#endif
