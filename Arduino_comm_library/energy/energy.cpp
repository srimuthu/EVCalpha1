#include "energy.h"
#include "Common.h"
#include "Arduino.h"
#include "Timer.h"
//#include "Definitions.cpp"


#define INFO_ENERGY(message) Serial.println(message)


void callbackReportEnergy0(void){
    
    energySolar.readProcedure();
    energyPi.readProcedure();
  };

energy::energy(int sensorPin,int numReadings, int interval,int sensorNum){

  INFO_ENERGY("REGISTER SENSOR");
  
  _sensorPin = sensorPin;
  _numReadings = numReadings;
  _interval = interval;
  _sensorNum = sensorNum;
  _sumEnergy = 0;

  if(sensorNum ==0){
     t.every(_interval, callbackReportEnergy0) ;
     
  }
  
  };

void energy::dummy(void){INFO_ENERGY("dummy_test");};

void energy::readProcedure(void){
  
  long sumReadings = 0;
  for(int reading = 0;reading< (1<<_numReadings);reading++)
  {
    sumReadings += analogRead(_sensorPin);
    delay(1);
   
  }
  
  sumReadings = (sumReadings >> _numReadings); // average current current & remove offset
  sumReadings -=512;
  if(sumReadings > 189)
	  sumReadings = 189;
  if(sumReadings < -189)
	  sumReadings = -189;
  
  sumReadings = map(sumReadings,-189,189,-5000,5000);
  sumReadings = (sumReadings << 2) ;
  sumReadings -= (sumReadings * 0.06) ;
  
  if(sumReadings < 0)
    sumReadings = 0;
  unsigned long timeNow = millis();
  Serial.println("SENSOR: "+String(_sensorNum)+" AVG:"+String(sumReadings) + "dT: " + String(timeNow-_lastReading) + "add E: " + String((long)((sumReadings * (timeNow-_lastReading))>>10)));
  _sumEnergy += (long)((sumReadings * (timeNow-_lastReading))>>10); // A milisecond
  Serial.println("sumEnergy:"+String(_sumEnergy));
  
  _lastReading = timeNow;
   ("Current current:" + String(sumReadings) + "total E: "+String(_sumEnergy));
  
  };

