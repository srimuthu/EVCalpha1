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
#define FORWARD 	0x02
#define REVERSE 	0x03
#define STOP		0x04
#define ARC_TURN	0x05



//Internal command
#define NO_PENDING	0xFE

//Motor Setup
#define PWM_L 9
#define PWM_R 5

#define EN_L_BWD 8
#define EN_L_FWD 4

#define EN_R_BWD 6
#define EN_R_FWD 7

#define CURRENT_CHARGE 0
#define CURRENT_LOAD 1

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
		int _led_pin;
		int _msg[MSG_SIZE];
		
		//Core private functions
		
		void doAction();
		
		void setMotor(const unsigned char cucPWM, const unsigned char cucFWD , const unsigned char cucBWD, const int ciSpeed);

		void commandLed();
		void commandForward();
		void commandReverse();
		void commandStop();
		void commandArcTurn();


/*		
		
		void commandSpotLeft();
		void commandSpotRight();
		*/

		//Private actions
		void blinkLed(int times, int delay);
		void forward(int pwm, int delay);
		void reverse(int pwm, int delay);
		void stop();
		void arcTurn(int left_pwm, int right_pwm, int delay);

};


#endif