#define F_CPU 22118400UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "usart.h"

#define TRIG_PIN PD3
#define ECHO_PIN PC5

volatile uint32_t duration;
volatile uint8_t echo_flag = 0;

ISR(INT1_vect) {
    if (echo_flag == 0) {
        TCNT2 = 0;
        echo_flag = 1;
    } else {
        duration = TCNT2;
        echo_flag = 0;
    }
}

void trigger_pulse() {
    PORTD |= (1 << TRIG_PIN);
    _delay_us(10);
    PORTD &= ~(1 << TRIG_PIN);
}

int main(void) {
    // Configure pins
    DDRD |= (1 << TRIG_PIN); // Set TRIG_PIN as output
    DDRC &= ~(1 << ECHO_PIN); // Set ECHO_PIN as input

    // Initialize UART
    usart_init();
    sei(); // Enable global interrupts

    // Configure external interrupt
    EICRA |= (1 << ISC10); // Trigger on any logical change
    EIMSK |= (1 << INT1); // Enable INT1

    // Configure Timer2
    TCCR2B |= (1 << CS21); // Set prescaler to 8

    while (1) {
        trigger_pulse();
        _delay_ms(500);

        if (echo_flag == 0) {
            uint16_t distance = ((uint32_t)duration * 0.034 * 8) / 2; // Multiply by prescaler value (8) to adjust for Timer2
            /*char buffer[20];
            sprintf(buffer, "Distance: %u cm\r\n", distance);
            uart_puts(buffer);*/
            int integer_part = (int)distance;
        	int fractional_part = (int)((distance - integer_part) * 1000); // 3 decimal places

        	printf("distance: %d.%03d cm\r", integer_part, fractional_part);
        }
    }
}
