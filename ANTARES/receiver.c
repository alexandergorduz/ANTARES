#include <avr/io.h>
#include <avr/interrupt.h>
#include "basic.h"
#include "receiver.h"

#define pwmMin 1000
#define pwmMax 2000
#define recRollCorr 0
#define recPitchCorr 0
#define recThrottleCorr 0
#define recYawCorr 9
#define setpAngleMin -40
#define setpAngleMax 40

volatile unsigned long timeCounterCh[6], recCurrTime;
volatile uint8_t stCh[6];
volatile int recInCh[6];

int recRoll, recPitch, recThrottle, recYaw;
float rollSetp, pitchSetp, yawSetp;

ISR (PCINT0_vect)
{
	recCurrTime = currentTime();
	
	if (PINB & 0b00000001)
	{
		if (stCh[0] == 0)
		{
			stCh[0] = 1;
			timeCounterCh[0] = recCurrTime;
		}
	} else if (stCh[0] == 1)
	{
		stCh[0] = 0;
		recInCh[0] = recCurrTime - timeCounterCh[0];
	}
	
	if (PINB & 0b00000010)
	{
		if (stCh[1] == 0)
		{
			stCh[1] = 1;
			timeCounterCh[1] = recCurrTime;
		}
	} else if (stCh[1] == 1)
	{
		stCh[1] = 0;
		recInCh[1] = recCurrTime - timeCounterCh[1];
	}
	
	if (PINB & 0b00000100)
	{
		if (stCh[2] == 0)
		{
			stCh[2] = 1;
			timeCounterCh[2] = recCurrTime;
		}
	} else if (stCh[2] == 1)
	{
		stCh[2] = 0;
		recInCh[2] = recCurrTime - timeCounterCh[2];
	}
	
	if (PINB & 0b00001000)
	{
		if (stCh[3] == 0)
		{
			stCh[3] = 1;
			timeCounterCh[3] = recCurrTime;
		}
	} else if (stCh[3] == 1)
	{
		stCh[3] = 0;
		recInCh[3] = recCurrTime - timeCounterCh[3];
	}
}

ISR (PCINT2_vect)
{
	recCurrTime = currentTime();
	
	if (PIND & 0b00000100)
	{
		if (stCh[4] == 0)
		{
			stCh[4] = 1;
			timeCounterCh[4] = recCurrTime;
		}
	} else if (stCh[4] == 1)
	{
		stCh[4] = 0;
		recInCh[4] = recCurrTime - timeCounterCh[4];
	}
	
	if (PIND & 0b00001000)
	{
		if (stCh[5] == 0)
		{
			stCh[5] = 1;
			timeCounterCh[5] = recCurrTime;
		}
	} else if (stCh[5] == 1)
	{
		stCh[5] = 0;
		recInCh[5] = recCurrTime - timeCounterCh[5];
	}
}

void readRec()
{
	recRoll = recRollCorr + pwmMin + (recInCh[0] - 1000.0) * (pwmMax - pwmMin) / (2000.0 - 1000.0);
	recPitch = recPitchCorr + pwmMin + (recInCh[1] - 1000.0) * (pwmMax - pwmMin) / (2000.0 - 1000.0);
	recThrottle = recThrottleCorr + pwmMin + (recInCh[2] - 1000.0) * (pwmMax - pwmMin) / (2000.0 - 1000.0);
	recYaw = recYawCorr + pwmMin + (recInCh[3] - 1000.0) * (pwmMax - pwmMin) / (2000.0 - 1000.0);
}

void calcSetps()
{
	rollSetp = 0;
	if (recRoll < 1490 || recRoll > 1510)
	{
		rollSetp = setpAngleMin + (recRoll - 1000.0) * (setpAngleMax - setpAngleMin) / (2000.0 - 1000.0);
	}
	
	pitchSetp = 0;
	if (recPitch < 1490 || recPitch > 1510)
	{
		pitchSetp = setpAngleMin + (recPitch - 1000.0) * (setpAngleMax - setpAngleMin) / (2000.0 - 1000.0);
	}
	
	yawSetp = 0;
	if ((recYaw < 1490 || recYaw > 1510) && recThrottle > 1050)
	{
		yawSetp = setpAngleMax + (recYaw - 1000.0) * (setpAngleMin - setpAngleMax) / (2000.0 - 1000.0);
	}
}

void initRec()
{
	SREG |= (1<<7);
	
	PCICR |= (1<<PCIE0);
	PCMSK0 |= (1<<PCINT0);
	PCMSK0 |= (1<<PCINT1);
	PCMSK0 |= (1<<PCINT2);
	PCMSK0 |= (1<<PCINT3);
	
	PCICR |= (1<<PCIE2);
	PCMSK2 |= (1<<PCINT18);
	PCMSK2 |= (1<<PCINT19);
}