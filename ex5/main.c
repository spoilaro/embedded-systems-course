#define F_CPU 16000000UL // UL stands for Unsigned Long

#include "lcd.h"
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/io.h>
#include <stdbool.h>

volatile bool adc_conv_complete = false;
volatile uint16_t voltage_to_digital_value = 0;

void setup() {
    DDRC &= ~(1 << PC3);
    TCCR0A = 0x02; //See datasheet p. 104
    TCCR0B = 0x05; //See datasheet p. 107
    OCR0A = 0xFF; //See datasheet p. 108
    TIMSK0 |= (1 << 1); //See datasheet p. 109
    TCNT0 = 0; //See datasheet p. 108
    sei();
    ADMUX |= 0b01000000; //See datasheet p. 248. Sets AV_cc as voltage reference.
    ADMUX |= 0b00000011; //See datasheet p. 249. Set A3 as input channel.
    ADCSRA |= (1 << 7); // Enable ADC, see datasheet p. 249
    ADCSRA |= (1 << 5); // Enable ADC auto trigger, see datasheet p. 249
    ADCSRA |= (1 << 3); // Enable ADC interrupt, see datasheet p. 250
    ADCSRA |= 0b00000101; // Set prescaler to 32, see datasheet p. 250
    ADCSRB |= 0b00000011; // Set ADC to trigger with timer/counter0 compare match A, see datasheet p. 251
    sei();
    lcd_init(LCD_DISP_ON);
}

int main(void)
{
    setup();

    while (1) { 
        if (adc_conv_complete) {
            char buffer[10];
            itoa(voltage_to_digital_value, buffer, 10);
            lcd_clrscr();
            lcd_puts(buffer);
            adc_conv_complete = false;
        }   
    }
}

ISR (TIMER0_COMPA_vect) {

}

ISR (ADC_vect) {
  voltage_to_digital_value = ADC;
  adc_conv_complete = true;
}