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
  comms.setLed(13);

}

void loop() 
{
   delay(50);
   comms.readIncomingMessage();
}


