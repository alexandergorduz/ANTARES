#include <avr/io.h>
#include <avr/interrupt.h>
#include "basic.h"

#define microsPerTIMER0Overflow (64 * 256 / 16)
#define millisInc (microsPerTIMER0Overflow / 1000)
#define fractInc ((microsPerTIMER0Overflow % 1000) >> 3)
#define fractMax (1000 >> 3)

volatile unsigned long timer0OverflowCount = 0;
volatile unsigned long timer0Millis = 0;
static unsigned char timer0Fract = 0;

ISR (TIMER0_OVF_vect)
{
	unsigned long m = timer0Millis;
	unsigned char f = timer0Fract;
	
	m += millisInc;
	f += fractInc;
	if (f >= fractMax)
	{
		f -= fractMax;
		m += 1;
	}
	
	timer0Fract = f;
	timer0Millis = m;
	timer0OverflowCount++;
}

unsigned long currentTime()
{
	unsigned long m;
	uint8_t oldSREG = SREG, t;
	
	SREG &= ~(1<<7);
	m = timer0OverflowCount;
	t = TCNT0;
	
	if ((TIFR0 & (1<<TOV0)) && (t < 255)) m++;
	
	SREG = oldSREG;
	
	return ((m << 8) + t) * 4;
}

void delayMicros(unsigned int us)
{
	if (us <= 1) return;
	
	us <<= 2;
	
	us -= 5;
	
	__asm__ __volatile__ (
	"1: sbiw %0,1" "\n\t"
	"brne 1b" : "=w" (us) : "0" (us)
	);
}

int analogRead(uint8_t pin)
{
	uint8_t low, high;
	
	ADMUX = (1<<REFS0) | (pin & 0x07);
	
	ADCSRA |= (1<<ADSC);
	
	while (ADCSRA & (1<<ADSC));
	
	low = ADCL;
	high = ADCH;
	
	return (high << 8) | low;
}

void initBasic()
{
	SREG |= (1<<7);
	
	TCCR0B |= (1<<CS01);
	TCCR0B |= (1<<CS00);
	TIMSK0 |= (1<<TOIE0);
	
	ADCSRA |= (1<<ADPS2);
	ADCSRA |= (1<<ADPS1);
	ADCSRA |= (1<<ADPS0);
	ADCSRA |= (1<<ADEN);
}