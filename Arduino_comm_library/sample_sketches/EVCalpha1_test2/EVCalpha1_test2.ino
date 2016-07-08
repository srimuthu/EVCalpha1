#include <energy.h>
#include <Timer.h>


// current sensor
#define PIN_ENERGY_SOLAR  1
#define PIN_ENERGY_PI     0
#define ENERGY_SOLAR_INTERVAL 1000 // interval in ms
#define ENERGY_SOLAR_NUMREADINGS 7 // number of readings for  averaging (2^n!!!)

#define ENERGY_PI_INTERVAL 1000 // interval in ms
#define ENERGY_PI_NUMREADINGS 7 // number of readings for averaging (2^n!!!)


Timer t;
energy energySolar(PIN_ENERGY_SOLAR  ,ENERGY_SOLAR_NUMREADINGS, ENERGY_SOLAR_INTERVAL,0);
energy energyPi(PIN_ENERGY_PI  ,ENERGY_SOLAR_NUMREADINGS, ENERGY_SOLAR_INTERVAL,1);

void setup()
{
  Serial.begin(9600); 
}

void loop()
{
  t.update();
}

