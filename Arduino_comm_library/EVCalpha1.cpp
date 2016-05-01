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
		
		default:
			break;
	}
}

void EVCalpha1::commandLed()
{
	blinkLed(_msg[1], _msg[2]);
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

