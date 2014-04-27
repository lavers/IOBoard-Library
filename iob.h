
#define IO_CENTER 		0
#define IO_LEFT			1
#define IO_UP 			2
#define IO_RIGHT 		3
#define IO_DOWN			4
#define IO_DIRECTION	5
#define IO_ANY 			6

#define IO_RED_ON 		PORTB |= _BV(PB7);
#define IO_RED_OFF		PORTB &= ~_BV(PB7);

#define IO_YELLOW_ON 	PORTD |= _BV(PD4);
#define IO_YELLOW_OFF 	PORTD &= ~_BV(PD4);

#define IO_GREEN_ON 	PORTD |= _BV(PD6);
#define IO_GREEN_OFF 	PORTD &= ~_BV(PD6);

void iob_init();

void iob_wheel_isr();
void iob_button_isr();

uint8_t iob_read(uint8_t button);
uint8_t iob_get_state();
int16_t iob_get_delta();
int16_t iob_wait_for_wheel();

void iob_wait_for_button(uint8_t button);

void iob_setup_wheel_timer();
void iob_setup_button_timer();

