#include <stdio.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "iob.h"

#define ARROW_KEYS (_BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5))

volatile uint8_t iob_state;
volatile int16_t iob_delta;
iob_orientation button_orientation;

void iob_init()
{
	iob_state = 0;
	button_orientation = IOB_EAST;
	// Disable JTAG

	MCUCR |= _BV(JTD);
	MCUCR |= _BV(JTD);

	// Setup LEDs

	DDRB |= _BV(PB7);
	DDRD |= _BV(PD4) | _BV(PD6);

	// Set D0-1 inputs with pullup (for reading center & arrow keys)

	DDRD &= ~(_BV(PD0) | _BV(PD1));
	PORTD |= _BV(PD0) | _BV(PD1);
}

void iob_set_orientation(iob_orientation orientation)
{
    button_orientation = orientation;
}

void iob_setup_wheel_timer()
{
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS01) | _BV(CS00); // F_CPU / 64

	OCR0A = (uint8_t) (F_CPU / (64.0 * 1000) - 0.5); // 1ms

	TIMSK0 |= _BV(OCIE0A);
}

void iob_setup_button_timer()
{
	TCCR2A = _BV(WGM21);
	TCCR2B = _BV(CS20) | _BV(CS21) | _BV(CS22);

	OCR2A = (uint8_t) (F_CPU / (1024.0 * 1000) - 0.5) * 20; // 20ms

	TIMSK2 |= _BV(OCIE2A);
}
int8_t max(int8_t val1, int8_t val2)
{
    if(val1 > val2) return val1;
    return val2;
}

uint8_t orient(uint8_t state)
{
    if(state == 0) return 0;
    uint8_t centre = state & _BV(IOB_CENTER);
    state = state >> 1;

    uint8_t lower = state & (_BV(max(-(int8_t)button_orientation, 0)) - 1);
	uint8_t newState = (button_orientation < 0) ? (state >> -button_orientation) : (state << button_orientation);
    uint8_t upper = newState & ((_BV(max(button_orientation, 0)) - 1) << 4);

	return (((newState & 0xF) | (lower << 3) | (upper >> 4)) << 1) | centre;
}

uint8_t iob_get_state()
{
	return orient(iob_state);
}

int16_t iob_get_delta()
{
	int16_t val;

	cli();
	val = iob_delta;
	iob_delta &= 1;
	sei();

	return val >> 1;
}

void iob_wait_for_button(uint8_t button)
{
	while(!iob_read(button));
}

int16_t iob_wait_for_wheel()
{
	int16_t delta;

	while(!(delta = iob_get_delta()));

	return delta;
}

uint8_t iob_read(uint8_t button)
{
	switch(button)
	{
		case IOB_ANY:
			return orient(iob_state);

		case IOB_DIRECTION:
			return iob_state > 1;

		default:
			return (orient(iob_state) >> button) & 1;
	}
}

void iob_button_isr()
{
	uint8_t ddrc = DDRC, portc = PORTC;

	// Set C2-5 output (Left, Up, Right, Down)

	DDRC |= ARROW_KEYS;

	// Shift a low bit from C5 to C2, reading D0 each time (leaving C0-1,6-7 unchanged)

	uint8_t i;
	for(i = 4; i; i--)
	{
		PORTC = (PORTC | ARROW_KEYS) & ~_BV(i + 1);

		_delay_us(1);

		if(PIND & _BV(PD0))
		{
			iob_state &= ~_BV(i);
		}
		else
		{
			iob_state |= _BV(i);
		}
	}

	// Set C4 low to read D1

	PORTC &= ~_BV(PC4);

	_delay_us(1);

	if(PIND & _BV(PD1))
	{
		iob_state &= ~_BV(IOB_CENTER);
	}
	else
	{
		iob_state |= _BV(IOB_CENTER);
	}

	// Restore port c

	DDRC = ddrc;
	PORTC = portc;
}

void iob_wheel_isr()
{
	static int8_t last;

	cli();

	uint8_t ddrc = DDRC, portc = PORTC, ddrd = DDRD, portd = PORTD;

	// Set C2,3 to input with pullup

	DDRC &= ~(_BV(PC2) | _BV(PC3));
	PORTC |= _BV(PC2) | _BV(PC3);

	// Set D0 to input no pullup, D1 to output low

	DDRD = (DDRD & ~_BV(PD0)) | _BV(PD1);
	PORTD &= ~(_BV(PD0) | _BV(PD1));

	_delay_us(1);

	/*
	Adapted from Peter Dannegger's code available at:
	http://www.mikrocontroller.net/attachment/40597/ENCODE.C
	*/

	int8_t new = 0, wheel = PINC, diff;

	if(wheel & _BV(PC3))
	{
		new = 3;
	}

	if(wheel & _BV(PC2))
	{
		new ^= 1; // convert gray to binary
	}

	diff = last - new;

	if(diff & 1)
	{
		last = new;
		iob_delta += (diff & 2) - 1;	// bit 1 = direction (+/-)
	}

	DDRC = ddrc;
	PORTC = portc;
	DDRD = ddrd;
	PORTD = portd;

	sei();
}
