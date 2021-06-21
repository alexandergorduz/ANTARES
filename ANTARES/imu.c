#include <avr/io.h>
#include "imu.h"

float xAcc, yAcc, zAcc;
float rollRateIn, pitchRateIn, yawRateIn;
int temp;

void readIMU()
{
	twiStart();
	twiSend(0b11010000);
	twiSend(0b00111011);
	twiStart();
	twiSend(0b11010001);
	
	xAcc = (twiRead()<<8 | twiRead()) / 4096.0;
	yAcc = (twiRead()<<8 | twiRead()) / 4096.0;
	zAcc = (twiRead()<<8 | twiRead()) / 4096.0;
	temp = twiRead()<<8 | twiRead();
	rollRateIn = (twiRead()<<8 | twiRead()) / 65.5;
	pitchRateIn = (twiRead()<<8 | twiRead()) / 65.5;
	yawRateIn = (twiRead()<<8 | twiReadLast()) / 65.5;
	
	twiStop();
}

void initIMURegs()
{
	twiStart();
	twiSend(0b11010000);
	twiSend(0b01101011);
	twiSend(0b00000000);
	twiStop();
	
	twiStart();
	twiSend(0b11010000);
	twiSend(0b00011011);
	twiSend(0b00001000);
	twiStop();
	
	twiStart();
	twiSend(0b11010000);
	twiSend(0b00011100);
	twiSend(0b00010000);
	twiStop();
	
	twiStart();
	twiSend(0b11010000);
	twiSend(0b00011010);
	twiSend(0b00000011);
	twiStop();
}

void twiStart()
{
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
}

void twiStop()
{
	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
	while(TWCR & (1<<TWSTO));
}

void twiSend(unsigned char c)
{
	TWDR = c;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
}

unsigned char twiRead()
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));
	return TWDR;
}

unsigned char twiReadLast()
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	return TWDR;
}

void initIMU()
{
	TWBR = 0b00001100;
}