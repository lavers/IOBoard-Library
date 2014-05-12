
#define IOB_CENTER 		0
#define IOB_LEFT			1
#define IOB_UP 			2
#define IOB_RIGHT 		3
#define IOB_DOWN			4
#define IOB_DIRECTION	5
#define IOB_ANY 			6

#define IOB_RED_ON 		PORTB |= _BV(PB7);
#define IOB_RED_OFF		PORTB &= ~_BV(PB7);

#define IOB_YELLOW_ON 	PORTD |= _BV(PD4);
#define IOB_YELLOW_OFF 	PORTD &= ~_BV(PD4);

#define IOB_GREEN_ON 	PORTD |= _BV(PD6);
#define IOB_GREEN_OFF 	PORTD &= ~_BV(PD6);

#include <stdint.h>

typedef enum {IOB_NORTH = 1, IOB_EAST = 0, IOB_SOUTH = -1, IOB_WEST = 2} iob_orientation;

void iob_init();
void iob_set_orientation(iob_orientation orientation);
void iob_wheel_isr();
void iob_button_isr();

uint8_t iob_read(uint8_t button);
uint8_t iob_get_state();
int16_t iob_get_delta();
int16_t iob_wait_for_wheel();

void iob_wait_for_button(uint8_t button);

void iob_setup_wheel_timer();
void iob_setup_button_timer();
