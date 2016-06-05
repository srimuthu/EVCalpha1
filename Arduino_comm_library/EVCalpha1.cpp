/*
  EVCalpha1.cpp - Library for serial messaging protocol 
  				for Embedded Visual Control project
  Created by SMN Balasubramanian, May 1, 2016.
*/

#include "Arduino.h"
#include "EVCalpha1.h"

EVCalpha1::EVCalpha1(int baud)
{
	_baud = baud;
	_msg[0] = NO_PENDING;
	analogWrite(PWM_R, 0);  
    analogWrite(PWM_L, 0);
    digitalWrite(EN_L_FWD, LOW);
    digitalWrite(EN_L_BWD, LOW);
    digitalWrite(EN_R_FWD, LOW);
    digitalWrite(EN_R_BWD, LOW);

    pinMode(PWM_L, OUTPUT);
    pinMode(PWM_R, OUTPUT);
    pinMode(EN_L_FWD, OUTPUT);
    pinMode(EN_L_BWD, OUTPUT);
    pinMode(EN_R_FWD, OUTPUT);
    pinMode(EN_R_BWD, OUTPUT);
}

void EVCalpha1::setLed(int led_pin)
{
	pinMode(led_pin, OUTPUT);
	_led_pin = led_pin;
}

void EVCalpha1::readIncomingMessage()
{
	if( Serial.available()>= MSG_SIZE) 
	{  
		// command length is 5 bytes
	   _msg[0] = Serial.read();
	   _msg[1] = Serial.read();
	   _msg[2] = Serial.read();
	   _msg[3] = Serial.read();
	   _msg[4] = Serial.read();
   }

   doAction();
}

void EVCalpha1::doAction()
{
	switch (_msg[0])
	{
		case NO_PENDING:
			break;

		case BLINK_LED:
			commandLed();
			_msg[0] = NO_PENDING;
			break;
		
		case FORWARD:
			commandForward();
			_msg[0] = NO_PENDING;
			break;		
		
		case REVERSE:
			commandReverse();
			_msg[0] = NO_PENDING;
			break;

		case STOP:
			commandStop();
			_msg[0] = NO_PENDING;
			break;

		case ARC_TURN:
			commandArcTurn();
			_msg[0] = NO_PENDING;
			break;
						


		default:
			break;
	}
}


void EVCalpha1::setMotor(const unsigned char cucPWM, const unsigned char cucFWD , const unsigned char cucBWD, const int ciSpeed)
{
	if (ciSpeed < 0)
	  {
	    digitalWrite(cucFWD, LOW); 
	    digitalWrite(cucBWD, LOW);  
	    digitalWrite(cucFWD, LOW); 
	    digitalWrite(cucBWD, HIGH);  
	  }
  	else
	  {
	    digitalWrite(cucFWD, LOW); 
	    digitalWrite(cucBWD, LOW);  
	    digitalWrite(cucFWD, HIGH); 
	    digitalWrite(cucBWD, LOW);  
	  }

  	analogWrite(cucPWM, abs(ciSpeed)); 
}

void EVCalpha1::commandLed()
{
	blinkLed(_msg[1], _msg[2]);
}

void EVCalpha1::commandForward()
{
	forward(_msg[1], (_msg[2]*_msg[3]));
}

void EVCalpha1::commandReverse()
{
	reverse(_msg[1], (_msg[2]*_msg[3]));
}

void EVCalpha1::commandStop()
{
	stop();
}

void EVCalpha1::commandArcTurn()
{
	arcTurn(_msg[1], _msg[2], (_msg[3]*_msg[4]));
}


void EVCalpha1::blinkLed(int num_times, int delay_time)
{
	int i;
	for(i=0; i<num_times; i++)
	{
		digitalWrite(_led_pin,HIGH);
		delay(delay_time);
		digitalWrite(_led_pin,LOW);
		delay(delay_time);
	}
}

void EVCalpha1::forward(int pwm, int delay)
{
	setMotor(PWM_L,EN_L_FWD,EN_L_BWD,pwm);
	setMotor(PWM_R,EN_R_FWD,EN_R_BWD,pwm);
}

void EVCalpha1::reverse(int pwm, int delay)
{
	setMotor(PWM_L,EN_L_FWD,EN_L_BWD,(pwm*-1));
	setMotor(PWM_R,EN_R_FWD,EN_R_BWD,(pwm*-1));
}

void EVCalpha1::stop()
{
	setMotor(PWM_L,EN_L_FWD,EN_L_BWD,0);
	setMotor(PWM_R,EN_R_FWD,EN_R_BWD,0);
}

void EVCalpha1::arcTurn(int left_pwm, int right_pwm, int delay)
{
	setMotor(PWM_L,EN_L_FWD,EN_L_BWD,left_pwm);
	setMotor(PWM_R,EN_R_FWD,EN_R_BWD,right_pwm);
}
