/*
  EVCalpha1.h - Library for serial messaging protocol 
  				for Embedded Visual Control project
  Created by SMN Balasubramanian, May 1, 2016.
*/


/*
!!!!!!!!!!!!!!!!!!!!!!!!!!!!CAUTION!!!!!!!!!!!!!!!!!!!!!!!!!
Initiate serial in void setup() of the sketch
Make sure that the class EVCalpha1 is instantiated only once
*/

#ifndef EVCalpha1_h
#define EVCalpha1_h

#include "Arduino.h"


//Common defines
#define MSG_SIZE	5


//Commands
#define BLINK_LED	0x01

//TO BE IMPLEMENTED
/*
#define FORWARD 	0x02
#define REVERSE 	0x03
*/

//Internal command
#define NO_PENDING	0xFE

class EVCalpha1
{
	public:
		//Constructor
		EVCalpha1(int baud);
		
		//Initialization functions

		void setLed(int led_pin);

		//TO BE IMPLEMENTED
/*		void setMotor1(int motor1_pin);
		void setMotor2(int motor2_pin);*/
		


		//Core public functions
		void readIncomingMessage(void);


	private:

		//Privates
		int _baud;
/*		int _motor1_pin;
		int _motor2_pin;*/
		int _led_pin;
		int _msg[MSG_SIZE];
		
		//Core private functions
		void doAction();
		void commandLed();

		//TO BE IMPLEMENTED
/*		void commandForward();
		void commandReverse();
		void commandSpotLeft();
		void commandSpotRight();
		void commandArcLeft();
		void commandArcRight();*/

		//Private actions
		void blinkLed(int times, int delay);

};


#endif