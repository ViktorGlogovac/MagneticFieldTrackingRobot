#include <avr/io.h>
#include <stdio.h>
#include "usart.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>

#define  Trigger_pin	PD2	/* Trigger pin */

int TimerOverflow = 0;

ISR(TIMER1_OVF_vect)
{
	TimerOverflow++;	/* Increment Timer Overflow count */
}

ISR(TIMER0_COMPA_vect) {
	OCR0A = OCR0A + OCR0_RELOAD;
	ISR_cnt++;
	if (ISR_cnt == ISR_pwm1) {
		PORTD &= ~(1 << 7); // PD7=0
	}
	if (ISR_cnt == ISR_pwm2) {
		PORTB &= ~(1 << 0); // PB0=0
	}
	if (ISR_cnt >= 2000) {
		ISR_cnt = 0; // 2000 * 10us=20ms
		PORTD |= (1 << 7); // PD7=1
		PORTB |= (1 << 0); // PB0=1
	}
}


int main(void)
{

	long count;
	float distance;
	
	usart_init();
	
	DDRD = 0x04;		/* Make trigger pin as output */
	PORTD = 0xFF;		/* Turn on Pull-up */

	
	sei();			/* Enable global interrupt */
	TIMSK1 = (1 << TOIE1);	/* Enable Timer1 overflow interrupts */
	TCCR1A = 0;		/* Set all bit to zero Normal operation */
	
	printf("Readings:\n");

	while(1)
	{
		/* Give 10us trigger pulse on trig. pin to HC-SR04 */
		PORTD |= (1 << Trigger_pin);
		_delay_us(10);
		PORTD &= (~(1 << Trigger_pin));
		
		TCNT1 = 0;	/* Clear Timer counter */
		TCCR1B = 0x41;	/* Capture on rising edge, No prescaler*/
		TIFR1 = 1<<ICF1;	/* Clear ICP flag (Input Capture flag) */
		TIFR1 = 1<<TOV1;	/* Clear Timer Overflow flag */

		/*Calculate width of Echo by Input Capture (ICP) */
		
		while ((TIFR1 & (1 << ICF1)) == 0);/* Wait for rising edge */
		TCNT1 = 0;	/* Clear Timer counter */
		TCCR1B = 0x01;	/* Capture on falling edge, No prescaler */
		TIFR1 = 1<<ICF1;	/* Clear ICP flag (Input Capture flag) */
		TIFR1 = 1<<TOV1;	/* Clear Timer Overflow flag */
		TimerOverflow = 0;/* Clear Timer overflow count */

		while ((TIFR1 & (1 << ICF1)) == 0);/* Wait for falling edge */
		count = ICR1 + (65535 * TimerOverflow);	/* Take count */
		/* 8MHz Timer freq, sound speed =343 m/s */
		distance = (float)count / (466.47*2);
		//distance = (int)count / 466;
		// Convert the floating-point distance value into integer and fractional parts
        int integer_part = (int)distance;
        int fractional_part = (int)((distance - integer_part) * 1000); // 3 decimal places

        printf("distance: %d.%03d cm\r", integer_part, fractional_part);
		_delay_ms(200);
	}
}
