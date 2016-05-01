#include <EVCalpha1.h>

EVCalpha1 comms(9600);

void setup() 
{
  Serial.begin(9600);
  comms.setLed(13);
}

void loop() 
{
   delay(500);
   comms.readIncomingMessage();
}


