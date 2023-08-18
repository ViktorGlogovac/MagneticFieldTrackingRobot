// This program shows how to measure the period of a signal using timer 1 free running counter.

#define F_CPU 16000000UL
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart.h"

#define PWM_FREQ 		114100L // Interrupt service routine tick is 100 us
#define TCNT0_RELOAD 	(uint8_t) (256 - (F_CPU/PWM_FREQ)) // no-prescaler

#define READ_FREQ		35200L // 2 * DEF_FREQ 16195 
#define TCNT1_RELOAD	(uint16_t) (65536 - (F_CPU/READ_FREQ)) // no-prescaler 

#define JOY_Y_UPPER_LIM		295L
#define JOY_Y_LOWER_LIM		 95L
#define JOY_Y_CENTER		195L
#define JOY_Y_CONV_RATE		(512/(JOY_Y_UPPER_LIM - JOY_Y_CENTER))

#define JOY_X_UPPER_LIM		285L
#define JOY_X_LOWER_LIM		 85L
#define JOY_X_CENTER		185L
#define JOY_X_CONV_RATE		(512/(JOY_X_UPPER_LIM - JOY_X_CENTER))

#define DEAD_ZONE			40L
#define MODE_THRES			1000L

#define PIN_PERIOD (PINB & (1<<1)) // PB1 Pin connected to 0 cross

volatile uint16_t on_period = 0;
volatile uint16_t off_period = 0;
volatile uint8_t state = 0;
volatile uint8_t flags = 0;

#define FOLLOW_MODE (1<<0)

volatile short joy_x = 0;
volatile short joy_y = 0;

volatile uint8_t PWM_counter = 0;
volatile short L_PWM = 0;
volatile short R_PWM = 0;


void wait_1ms(void) { // alphaghetti
	    //setup Timer 2
    TCNT2 = 193;		//load count value for 1ms time delay
	TCCR2A = 0x00;
   	TCCR2B = 0b00000110;	//normal mode with 256 pre-scalar
     
   	while(!(TIFR2 & (1<<TOV2)));	//wait until TOV0 flag is set
   	TCCR2B = 0;		//turn off timer 2
   	TIFR2 |= (1<<TOV2);	//clear TOV2 flag
}

void waitms(int ms) {
	while(ms--) wait_1ms();
}

void PrintNumber(long int N, int Base, int digits)
{ 
	char HexDigit[]="0123456789ABCDEF";
	int j;
	#define NBITS 32
	char buff[NBITS+1];
	buff[NBITS]=0;

	j=NBITS-1;
	while ( (N>0) | (digits>0) )
	{
		buff[j--]=HexDigit[N%Base];
		N/=Base;
		if(digits!=0) digits--;
	}
	usart_pstr(&buff[j+1]);
}

void calc_L_PWM(short x, short y) {
	short l = (y - x);
	if (l <= DEAD_ZONE && l >= -DEAD_ZONE) 
		l = 0; 
	L_PWM = l;
}

void calc_R_PWM(short x, short y) {
	short r = (y - x);
	if (r <= DEAD_ZONE && r >= -DEAD_ZONE) 
		r = 0; 
	R_PWM = r;
}

void calc_x(short x) {
	joy_x = (x - JOY_X_CENTER) * JOY_X_CONV_RATE;
}

void calc_y(short y) {
	joy_y = (y - JOY_Y_CENTER) * JOY_Y_CONV_RATE;
}



ISR (TIMER1_OVF_vect)    // Timer1 ISR
{
	PORTB ^= (1 << 0);
	TIMSK1 &= ~(1 << TOIE1);
	if (PIN_PERIOD) {
		on_period++;

	} else {
		off_period++;
	}
	if (on_period > MODE_THRES) {
		//flags |= (1<<FOLLOW_MODE);
		flags = 1;
	} else {
		//flags &= ~(1<<FOLLOW_MODE);
		flags = 0;
		// if (on_period == 100) {
		// 	PORTB |= (1<<0);
		// } else {
		// 	PORTB &= ~(1<<0);
		// }
		if (PIN_PERIOD && state == 1) {
			// digital
			if (on_period > (JOY_Y_CENTER + DEAD_ZONE) && off_period > (JOY_X_CENTER + DEAD_ZONE)) { // forward right
				PORTD |= (1 << 5); 							// L 100
				PORTD &= ~((1 << 6) | (1 << 3) | (1 << 4));	// R stop
				PORTC = 1;
			} else if (on_period > (JOY_Y_CENTER + DEAD_ZONE) && off_period < (JOY_X_CENTER - DEAD_ZONE)) { // forward left
				PORTD |= (1 << 6); 							// R 100
				PORTD &= ~((1 << 5) | (1 << 3) | (1 << 4));	// L stop
				PORTC = 2;
			} else if (on_period < (JOY_Y_CENTER - DEAD_ZONE) && off_period > (JOY_X_CENTER + DEAD_ZONE)) { // back right
				PORTD |= (1 << 3); 							// L -100
				PORTD &= ~((1 << 5) | (1 << 3) | (1 << 4));	// R stop
				PORTC = 3;
			} else if (on_period < (JOY_Y_CENTER - DEAD_ZONE) && off_period < (JOY_X_CENTER - DEAD_ZONE)) { // back left
				PORTD |= (1 << 4); 							// R -100
				PORTD &= ~((1 << 6) | (1 << 3) | (1 << 4));	// L stop
				PORTC = 4;
			} else if (on_period > (JOY_Y_CENTER + DEAD_ZONE)) { // forward
				PORTD |= (1 << 5) | (1 << 6);				// R 100
				PORTD &= ~((1 << 3) | (1 << 4));			// L 100
				PORTC = 5;
			} else if (on_period < (JOY_Y_CENTER - DEAD_ZONE)) { // back
				PORTD |= (1 << 3) | (1 << 4);				// R -100
				PORTD &= ~((1 << 5) | (1 << 6));			// L -100
				PORTC = 6;
			} else if (off_period > (JOY_X_CENTER + DEAD_ZONE)) { // right
				PORTD |= (1 << 5) | (1 << 4);				// R -100
				PORTD &= ~((1 << 3) | (1 << 6));			// L 100
				PORTC = 7;
			} else if (off_period < (JOY_X_CENTER - DEAD_ZONE)) { // left
				PORTD |= (1 << 3) | (1 << 6);				// R 100
				PORTD &= ~((1 << 5) | (1 << 4));			// L -100
				PORTC = 8;
			} else { //stop
				PORTD &= ~((1 << 3) | (1 << 4) | (1 << 5) | (1 << 6));
				PORTC = 0;
			}

			joy_x = off_period;
			// joy_x = (off_period - JOY_X_CENTER) * JOY_X_CONV_RATE;
			// if (joy_x < DEAD_ZONE && joy_x > -DEAD_ZONE)
			// 	joy_x = 0;

			
			// short l_pow = joy_y + joy_x;
			// if (l_pow < 0) {
			// 	l_pow = - l_pow;
			// 	TCCR0A |= (1 << COM0A0); // set inverting mode
			// 	PORTD &= ~(1<<DDD3);
			// } else {
			// 	TCCR0A &= (1 << COM0A0); // set non-inverting mode
			// 	PORTD |= (1<<DDD3);
			// }

			// if (l_pow > 255) {
			// 	l_pow = 255;
			// }
			// OCR0A = (uint8_t) l_pow;
			// L_PWM = l_pow;

			// short r_pow = joy_y - joy_x;
			// if (r_pow < 0) {
			// 	r_pow = - r_pow;
			// 	TCCR0B |= (1 << COM0B0); // set inverting mode
			// 	PORTD &= ~(1<<DDD4);
			// } else {
			// 	TCCR0B &= (1 << COM0B0); // set non-inverting mode
			// 	PORTD |= (1<<DDD4);
			// }

			// if (r_pow > 255) {
			// 	r_pow = 255;
			// }
			// OCR0B = (uint8_t) r_pow;
			// R_PWM = r_pow;


			off_period = 0;
			on_period = 0;
			state = 0;
			PORTD &= ~(1 << 7);

		} else if (!PIN_PERIOD && state == 0) {
			// joy_y = (on_period - JOY_Y_CENTER) * JOY_Y_CONV_RATE;
			// if (joy_y < DEAD_ZONE && joy_y > -DEAD_ZONE)
			// 	joy_y = 0;
			joy_y = on_period;
			

			//on_period = 0;
			state = 1;
			PORTD |= (1 << 7);
			OCR0A = 128; // set PWM for 50% duty cycle
    		OCR0B = 64;
		}
	}
	
	TIMSK1 |= (1 << TOIE1);
	TCNT1 = TCNT1_RELOAD;   // reload timer 1
}

// ISR (TIMER0_OVF_vect)
// {
// 	TIMSK0 &= ~(1 << TOIE0);
// 	if (flags != 0) {//flags&FOLLOW_MODE) {
// 		// follow mode shit
// 		PORTB |= (1<<0);
// 	} else {
// 		if (PWM_counter == 0) {
// 			calc_L_PWM(joy_x, joy_y);
// 			calc_R_PWM(joy_x, joy_y);
// 			PORTB ^= (1<<0);
// 		}
// 		if (PWM_counter < L_PWM) {
// 			PORTD |= (1<<4);
// 			PORTD &= ~(1<<3);
// 		} else if (PWM_counter < - L_PWM) {
// 			PORTD &= ~(1<<4);
// 			PORTD |= (1<<3);
// 		} else {
// 			PORTD &= ~((1<<4) | (1<<3));
// 		}
// 		if (PWM_counter < R_PWM) {
// 			PORTD |= (1<<6);
// 			PORTD &= ~(1<<5);
// 		} else if (PWM_counter < - R_PWM) {
// 			PORTD &= ~(1<<6);
// 			PORTD |= (1<<5);
// 		} else {
// 			PORTD &= ~((1<<6) | (1<<5));
// 		}
// 	}
// 	TIMSK0 |= (1 << TOIE0);
// 	TCNT0 = TCNT0_RELOAD;
// 	PWM_counter++;
// }




/* Pinout for DIP28 ATMega328P:

                           -------
     (PCINT14/RESET) PC6 -|1    28|- PC5 (ADC5/SCL/PCINT13)
       (PCINT16/RXD) PD0 -|2    27|- PC4 (ADC4/SDA/PCINT12)
       (PCINT17/TXD) PD1 -|3    26|- PC3 (ADC3/PCINT11)
      (PCINT18/INT0) PD2 -|4    25|- PC2 (ADC2/PCINT10)
 (PCINT19/OC2B/INT1) PD3 -|5    24|- PC1 (ADC1/PCINT9)
    (PCINT20/XCK/T0) PD4 -|6    23|- PC0 (ADC0/PCINT8)
                     VCC -|7    22|- GND
                     GND -|8    21|- AREF
(PCINT6/XTAL1/TOSC1) PB6 -|9    20|- AVCC
(PCINT7/XTAL2/TOSC2) PB7 -|10   19|- PB5 (SCK/PCINT5)
   (PCINT21/OC0B/T1) PD5 -|11   18|- PB4 (MISO/PCINT4)
 (PCINT22/OC0A/AIN0) PD6 -|12   17|- PB3 (MOSI/OC2A/PCINT3)
      (PCINT23/AIN1) PD7 -|13   16|- PB2 (SS/OC1B/PCINT2)
  (PCINT0/CLKO/ICP1) PB0 -|14   15|- PB1 (OC1A/PCINT1)
                           -------
*/

void config_PWM (void)
{
	OCR0A = 0;
    OCR0B = 0;
	TCCR0A |= (1 << COM0A1) | (1 << COM0B1); // set none-inverting mode
    TCCR0A |= (1 << WGM01) | (1 << WGM00); // set fast PWM Mode
    TCCR0B |= (1 << CS00); // set prescaler to none and starts PWM
}

void ConfigurePins (void)
{
	DDRB  &= 0b11111101; // Configure PB1 as input
	PORTB |= 0b00000010; // Activate pull-up in PB1
	
	DDRD  |= 0b11111100; // PD[7..2] configured as outputs
	PORTD &= 0b00000011; // PD[7..2] = 0
	
	DDRB  |= 0b00000001; // PB0 configured as output
	PORTB &= 0x11111110; // PB0 = 0
}

int main(void)
{
	ConfigurePins();
	usart_init(); // configure the usart and baudrate
	// timer_init1();
	DDRB |= 0x01;
	PORTB |= 0x01;

	TCNT1 = 63974;   // for 1 sec at 16 MHz	

	TCCR1A = 0x00;
	TCCR1B |= _BV(CS10);  // no prescaler
	TIMSK1 = (1 << TOIE1);   // Enable timer1 overflow interrupt(TOIE1)

	// TCNT0 = TCNT0_RELOAD;		//load count value for 1ms time delay
   	// TCCR0A = 0x00;
   	// TCCR0B |= _BV(CS00);	// no prescale
	// TIMSK0 = (1 << TOIE0);
	// config_PWM();
	sei();        // Enable global interrupts by setting global interrupt enable bit in SREG

	waitms(500); // Wait for putty to start

	usart_pstr("\x1b[2J\x1b[1;1H\r\n\r\n"); // Clear screen using ANSI escape sequence.


	
	// DDRD |= 0x07;
	// PORTB |= 0x07;

	while(1) 
	{
		waitms(200);
		usart_pstr("X axis: ");
		PrintNumber(joy_x, 10, 10);
		usart_pstr(" Y axis: ");
		PrintNumber(joy_y, 10, 10);
		usart_pstr("L_PWM: ");
		PrintNumber(L_PWM, 10, 10);
		usart_pstr(" R_PWM: ");
		PrintNumber(R_PWM, 10, 10);
		usart_pstr("\r\n");
		
	}
	return 0;
}