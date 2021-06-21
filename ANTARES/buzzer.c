#include <avr/io.h>
#include <avr/interrupt.h>
#include "buzzer.h"

ISR (TIMER1_OVF_vect)
{
	TCNT1H = 0b11100000;
	TCNT1L = 0b00000000;
	PORTB ^= (1<<PB4);
}

void tone()
{
	TCCR1B |= (1<<CS10);
}

void noTone()
{
	TCCR1B &= ~(1<<CS10);
}

void initBuzz()
{
	SREG |= (1<<7);
	
	TIMSK1 |= (1<<TOIE1);
}