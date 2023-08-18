// This program shows how to measure the period of a signal using timer 1 free running counter.

#define F_CPU 16000000UL
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart.h"

#define PWM_FREQ 		114100L // Interrupt service routine tick is 100 us
#define TCNT0_RELOAD 	(uint8_t) (256 - (F_CPU/PWM_FREQ)) // no-prescaler

#define READ_FREQ		17600L
#define TCNT1_RELOAD	(uint16_t) (65536 - (F_CPU/READ_FREQ)) // no-prescaler 

#define MIN_TH				43
#define TRACK_TH			48
#define R_TH				53
#define F_TH				56
#define L_TH				61
#define B_TH				65
#define STOP_TH				69
#define F_R_TH				73
#define F_L_TH				77
#define B_R_TH				81
#define B_L_TH				85
#define LIGHTS_TH			90

#define L1					(1 << 3)
#define L2					(1 << 4)
#define R1					(1 << 6)
#define R2					(1 << 5)

#define DEAD_ZONE			200L

#define PIN_PERIOD (PINB & (1<<1)) // PB1 Pin connected to 0 cross

volatile uint8_t on_period = 0;
volatile uint8_t off_period = 0;
volatile uint8_t period = 0;
volatile uint8_t state = 0;

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

void adc_init(void)
{
    ADMUX = (1<<REFS0);
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

uint16_t adc_read(int channel)
{
    channel &= 0x7;
    ADMUX = (ADMUX & 0xf8)|channel;
     
    ADCSRA |= (1<<ADSC);
     
    while(ADCSRA & (1<<ADSC)); //as long as ADSC pin is 1 just wait.
     
    return (ADCW);
}

uint8_t motor_power (uint8_t period, uint8_t v_threshold) {
	uint8_t mode;
	if (period < MIN_TH) {	// stop
		//PORTD &= ~(L1 | L2 | R1 | R2);
		mode = 0;
	}
	else if (period < TRACK_TH) {	// track
		uint16_t adc;
		uint16_t vL;
		uint16_t vR;
		adc=adc_read(0);
		vL=(adc*5000L)/1023L;

		adc=adc_read(1);
		vR=(adc*5000L)/1023L;

		if		(vL <  vR - DEAD_ZONE)
		{//Left
			PORTD |= L2 | R1;			// R 100
			PORTD &= ~(L1 | R2);		// L -100	
			mode = 13;
		}
		else if (vL > vR + DEAD_ZONE)
		{//Right
			PORTD |= L1 | R2;			// R -100
			PORTD &= ~(L2 | R1);		// L 100
			mode = 14;
		}
		else if (vL > (v_threshold * 1000 + DEAD_ZONE) && (vR > (v_threshold * 1000 + DEAD_ZONE)))
		{//Forward
			PORTD |= L1 | R1;			// R 100
			PORTD &= ~(L2 | R2);		// L 100
			mode = 15;
		}
		else if (vL < (v_threshold * 1000 - DEAD_ZONE) && (vR < (v_threshold * 1000 - DEAD_ZONE)))
		{//Backward
			PORTD |= L2 | R2;			// R -100
			PORTD &= ~(L1 | R1);		// L -100
			mode = 16;
		}
		else
		{// stop
			PORTD &= ~(L1 | L2 | R1 | R2);
			mode = 1;
		}
		
	}
	else if (period < R_TH) {		// right
		PORTD |= L1 | R2;			// R -100
		PORTD &= ~(L2 | R1);		// L 100
		mode = 2;
	}
	else if (period < F_TH) {		// forward
		PORTD |= L1 | R1;			// R 100
		PORTD &= ~(L2 | R2);		// L 100
		mode = 3;
	}
	else if (period < L_TH) {		// left
		PORTD |= L2 | R1;			// R 100
		PORTD &= ~(L1 | R2);		// L -100
		mode = 4;
	}
	else if (period < B_TH) {		// back
		PORTD |= L2 | R2;			// R -100
		PORTD &= ~(L1 | R1);		// L -100
		mode = 5;
	}
	else if (period < STOP_TH) {	// stop
		PORTD &= ~(L1 | L2 | R1 | R2);
		mode = 6;
	}
	else if (period < F_R_TH) {		// forward right
		PORTD |= L1; 				// L 100
		PORTD &= ~(L2 | R1 | R2);	// R stop
		mode = 7;
	}
	else if (period < F_L_TH) {		// forward left
		PORTD |= R1; 				// R 100
		PORTD &= ~(L1 | L2 | R2);	// L stop
		mode = 8;
	}
	else if (period < B_R_TH) {		// back right
		PORTD |= L2; 				// L -100
		PORTD &= ~(L1 | R1 | R2);	// R stop
		mode = 9;
	}
	else if (period < B_L_TH) {		// back left
		PORTD |= R2; 				// R -100
		PORTD &= ~(L1 | L2 | R1);	// L stop
		mode = 10;
	}
	else if (period < LIGHTS_TH) {
		mode = 11;
	}
	else {
		PORTD &= ~(L1 | L2 | R1 | R2);
		mode = 12;
	}
	return mode;
}
uint8_t calc_period (uint8_t on, uint8_t off) {
	return on + off;
}

ISR (TIMER1_OVF_vect)    // Timer1 ISR
{
	//overflow++;
	PORTB ^= (1 << 0);
	TIMSK1 &= ~(1 << TOIE1);
	if (PIN_PERIOD) {
		on_period++;

	} else {
		off_period++;
	}
	
	
	if (PIN_PERIOD && state == 1) {
		period = calc_period(on_period, off_period);
		off_period = 0;
		on_period = 0;
		state = 0;
		PORTD |= (1 << 7);
	} else if (!PIN_PERIOD && state == 0) {
		state = 1;
		PORTD &= ~(1 << 7);
	} else {
		
	}
	TIMSK1 |= (1 << TOIE1);
	TCNT1 = TCNT1_RELOAD;   // reload timer 1

}

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
	uint16_t i = 0;
	uint8_t mode;

	ConfigurePins();
	usart_init(); // configure the usart and baudrate
	// timer_init1();
	DDRB |= 0x01;
	PORTB |= 0x01;

	TCNT1 = TCNT1_RELOAD;   	

	TCCR1A = 0x00;
	TCCR1B |= _BV(CS10);  // no prescaler
	TIMSK1 = (1 << TOIE1);   // Enable timer1 overflow interrupt(TOIE1)

	// TCNT0 = TCNT0_RELOAD;		//load count value for 1ms time delay
   	// TCCR0A = 0x00;
   	// TCCR0B |= _BV(CS00);	// no prescale
	// TIMSK0 = (1 << TOIE0);
	// config_PWM();
	sei();        // Enable global interrupts by setting global interrupt enable bit in SREG

	adc_init();

	waitms(500); // Wait for putty to start

	usart_pstr("\x1b[2J\x1b[1;1H\r\n\r\n"); // Clear screen using ANSI escape sequence.


	
	// DDRD |= 0x07;
	// PORTB |= 0x07;
	uint16_t adc;
	uint16_t v1;
	uint16_t v2;

	while(1) 
	{
		mode = motor_power(period, 1);
		if (i != 100000) {
			waitms(200);
			usart_pstr("Period: ");
			PrintNumber(period, 10, 3);
			usart_pstr(" mode: ");
			PrintNumber(mode, 10, 2);
			usart_pstr("\r\n");
			adc=adc_read(0);

			v1=(adc*5000L)/1023L;
			usart_pstr("ADC[0]=0x");
			PrintNumber(adc, 16, 3);
			usart_pstr(", ");
			PrintNumber(v1/1000, 10, 1);
			usart_pstr(".");
			PrintNumber(v1%1000, 10, 3);
			usart_pstr("V ");
			
			adc=adc_read(1);
			v2=(adc*5000L)/1023L;
			usart_pstr("ADC[1]=0x");
			PrintNumber(adc, 16, 3);
			usart_pstr(", ");
			PrintNumber(v2/1000, 10, 1);
			usart_pstr(".");
			PrintNumber(v2%1000, 10, 3);
			usart_pstr("V ");
			i = 0;
		}
		i++;
	}
	return 0;
}