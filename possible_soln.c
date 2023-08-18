#include <avr/io.h>
#include <stdio.h>
#include "usart.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>

#define Trigger_pin PD2 /* Trigger pin */
#define Echo_pin    PC0 /* Echo pin (SCL) */

volatile uint16_t pulse_start = 0;
volatile uint16_t pulse_end = 0;
volatile uint8_t pulse_flag = 0;

ISR(PCINT1_vect)
{
    if (PINC & (1 << Echo_pin)) {
        pulse_start = TCNT1;
        pulse_flag = 1;
    }
    else if (pulse_flag == 1) {
        pulse_end = TCNT1;
        pulse_flag = 0;
    }
}

int main(void)
{
    long count;
    float distance;

    usart_init();

    DDRD |= (1 << Trigger_pin);   /* Make trigger pin as output */
    PORTC |= (1 << Echo_pin);     /* Turn on Pull-up for Echo pin */

    sei();                        /* Enable global interrupt */
    PCICR |= (1 << PCIE1);        /* Enable Pin Change Interrupt 1 */
    PCMSK1 |= (1 << PCINT8);      /* Enable Pin Change Interrupt for PC0 */

    TCCR1A = 0;                   /* Set all bit to zero Normal operation */
    TCCR1B = (1 << CS11);         /* Prescaler 8 */

    printf("Readings:\n");

    while (1)
    {
        /* Give 10us trigger pulse on trig. pin to HC-SR04 */
        PORTD |= (1 << Trigger_pin);
        _delay_us(10);
        PORTD &= (~(1 << Trigger_pin));

        _delay_ms(50); /* Wait for the measurements to complete */

        if (pulse_end >= pulse_start) {
            count = pulse_end - pulse_start;
        }
        else {
            count = (65535 - pulse_start) + pulse_end;
        }

        /* 8MHz Timer freq, sound speed =343 m/s, prescaler 8 */
        distance = (float)count / (58.82 * 2);

        int integer_part = (int)distance;
        int fractional_part = (int)((distance - integer_part) * 1000); // 3 decimal places

        printf("distance: %d.%03d cm\r", integer_part, fractional_part);
        _delay_ms(200);
    }
}
