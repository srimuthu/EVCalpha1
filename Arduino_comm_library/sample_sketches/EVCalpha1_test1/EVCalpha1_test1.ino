#include <EVCalpha1.h>
//#include <Servo.h>
//
//Servo panServo;
//Servo tiltServo;
EVCalpha1 comms(9600);

void setup() 
{
  Serial.begin(9600);
//  panServo.attach(10);
//  tiltServo.attach(11);
//  comms.setLed(13);
  attachInterrupt(0, leftEncoderEvent, CHANGE);
  attachInterrupt(1, rightEncoderEvent, CHANGE);
}

void loop() 
{
   //delay(50);
   if(Serial)
   {
    comms.readIncomingMessage();
   }
}


