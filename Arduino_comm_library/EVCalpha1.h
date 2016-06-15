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
#define MAX_SERVO_ANGLE	180
#define SERVO_LOOP	30

//Commands
#define BLINK_LED	0x01
#define FORWARD 	0x02
#define REVERSE 	0x03
#define STOP		0x04
#define ARC_LEFT	0x05
#define ARC_RIGHT	0x06
#define PAN_SERVO	0x07
#define TILT_SERVO	0x08


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

#define PAN		12
#define TILT	13



class EVCalpha1
{
	public:
		//Constructor
		EVCalpha1(int baud);
		
		//Initialization functions

		void setLed(int led_pin);

		//Core public functions
		void readIncomingMessage(void);


	private:

		//Privates
		int _baud;
		int _led_pin;
		int _msg[MSG_SIZE];

		int _servo_default;
		int _lenMicroSecondsOfPeriod = 25 * 1000; // 25 milliseconds (ms)
		float _servo_lut[4] = {0.0,0.7,180.0,2.2};

		//Core private functions
		
		void doAction();
		
		void setMotor(const unsigned char cucPWM, const unsigned char cucFWD , const unsigned char cucBWD, const int ciSpeed);

		void commandLed();
		void commandForward();
		void commandReverse();
		void commandStop();
		void commandArcLeft();
		void commandArcRight();
		void commandPan();
		void commandTilt();



/*		
		
		void commandSpotLeft();
		void commandSpotRight();
		*/

		//Private actions
		int interpolate(int val);

		void blinkLed(int times, int delay);
		void forward(int pwm, int delay);
		void reverse(int pwm, int delay);
		void stop();
		void arcLeft(int left_pwm, int right_pwm, int delay);
		void arcRight(int left_pwm, int right_pwm, int delay);
		void pan(int value);
		void tilt(int value);


};


#endif