#define F_CPU 16000000UL
#include <avr/io.h>
#include "basic.h"
#include "imu.h"
#include "buzzer.h"
#include "receiver.h"
#include "pid.h"

float roll, pitch, yaw;
float accRoll, accPitch;
float rollRate, pitchRate, yawRate;
float rollRateInitErr, pitchRateInitErr, yawRateInitErr;

int batV;
uint8_t ready;

int escfr, escrr, escrl, escfl;
unsigned long escfrTmr, escrrTmr, escrlTmr, escflTmr, escsCurrTime;

unsigned long loopTmr;

int main(void)
{
	initBasic();
	initIMU();
	initBuzz();
	
	DDRD |= 0b11110000;
	DDRB |= 0b00110000;
	
	PORTB |= (1<<5);
	
	initIMURegs();
	
	for (int i = 0; i < 1250; i++)
	{
		if (i % 30 == 0) tone();
		if (i % 60 == 0) noTone();
		
		PORTD |= 0b11110000;
		delayMicros(1000);
		PORTD &= 0b00001111;
		delayMicros(3000);
	}
	noTone();
	
	for (int i = 0; i < 1000; i++)
	{
		if (i % 15 == 0) tone();
		if (i % 30 == 0) noTone();
		
		readIMU();
		
		rollRateInitErr += rollRateIn;
		pitchRateInitErr += pitchRateIn;
		yawRateInitErr += yawRateIn;
		
		PORTD |= 0b11110000;
		delayMicros(1000);
		PORTD &= 0b00001111;
		delayMicros(3000);
	}
	noTone();
	
	rollRateInitErr /= 1000;
	pitchRateInitErr /= 1000;
	yawRateInitErr /= 1000;
	
	initRec();
	
	ready = 0;
	
	batV = (analogRead(0) + 65) * 1.2317;
	
	PORTB &= ~(1<<5);
	
	loopTmr = currentTime();
	
    while (1) 
    {
		accRoll = atan(yAcc / sqrt(pow(xAcc, 2) + pow(zAcc, 2))) * 57.296;
		accPitch = -atan(xAcc / sqrt(pow(yAcc, 2) + pow(zAcc, 2))) * 57.296;
		
		roll = ((rollRateIn - rollRateInitErr) * 0.004 + roll) * 0.9995 + accRoll * 0.0005;
		pitch = ((pitchRateIn - pitchRateInitErr) * 0.004 + pitch) * 0.9995 + accPitch * 0.0005;
		yaw = (yawRateIn - yawRateInitErr) * 0.004 + yaw;
		
		roll += pitch * sin((yawRateIn - yawRateInitErr) * 0.0000698);
		pitch -= roll * sin((yawRateIn - yawRateInitErr) * 0.0000698);
		
		rollRate = rollRate * 0.7 + (rollRateIn - rollRateInitErr) * 0.3;
		pitchRate = pitchRate * 0.7 + (pitchRateIn - pitchRateInitErr) * 0.3;
		yawRate = yawRate * 0.7 + (yawRateIn - yawRateInitErr) * 0.3;
		
		if (recThrottle < 1050 && recYaw > 1950) ready = 1;
		
		if (recThrottle < 1050 && recYaw < 1550 && ready == 1)
		{
			ready = 2;
			
			roll = accRoll;
			pitch = accPitch;
			yaw = 0;
			
			rollRateInt = 0;
			rollRateLastErr = 0;
			pitchRateInt = 0;
			pitchRateLastErr = 0;
			yawRateInt = 0;
			yawRateLastErr = 0;
		}
		
		if (recThrottle < 1050 && recYaw < 1050 && ready == 2) ready = 0;
		
		calcSetps();
		calcPID();
		
		batV = batV * 0.92 + (analogRead(0) + 65) * 0.09853;
		
		if (batV < 1000 && batV > 600) tone();
		
		if (ready == 2)
		{
			if (recThrottle > 1800) throttleControl = 1800;
			
			escfr = throttleControl - pitchControl - rollControl - yawControl;
			escrr = throttleControl + pitchControl - rollControl + yawControl;
			escrl = throttleControl + pitchControl + rollControl - yawControl;
			escfl = throttleControl - pitchControl + rollControl + yawControl;
			
			if (batV < 1240 && batV > 800)
			{
				escfr += escfr * ((1240 - batV) / 3500.0);
				escrr += escrr * ((1240 - batV) / 3500.0);
				escrl += escrl * ((1240 - batV) / 3500.0);
				escfl += escfl * ((1240 - batV) / 3500.0);
			}
			
			if (escfr < 1100) escfr = 1100;
			if (escrr < 1100) escrr = 1100;
			if (escrl < 1100) escrl = 1100;
			if (escfl < 1100) escfl = 1100;
			
			if (escfr > 2000) escfr = 2000;
			if (escrr > 2000) escrr = 2000;
			if (escrl > 2000) escrl = 2000;
			if (escfl > 2000) escfl = 2000;
		} else
		{
			escfr = 1000;
			escrr = 1000;
			escrl = 1000;
			escfl = 1000;
		}
		
		while (currentTime() - loopTmr < 4000);
		loopTmr = currentTime();
		
		PORTD |= 0b11110000;
		
		escfrTmr = escfr + loopTmr;
		escrrTmr = escrr + loopTmr;
		escrlTmr = escrl + loopTmr;
		escflTmr = escfl + loopTmr;
		
		readIMU();
		readRec();
		
		while (PORTD >= 16)
		{
			escsCurrTime = currentTime();
			if (escfrTmr <= escsCurrTime) PORTD &= ~(1<<4);
			if (escrrTmr <= escsCurrTime) PORTD &= ~(1<<5);
			if (escrlTmr <= escsCurrTime) PORTD &= ~(1<<6);
			if (escflTmr <= escsCurrTime) PORTD &= ~(1<<7);
		}
    }
}