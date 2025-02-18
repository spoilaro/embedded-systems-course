#define F_CPU 16000000UL // UL stands for Unsigned Long
#define DELAY 100

#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
#include "lcd.h"
#include <stdlib.h>


void setup() {
    DDRD &= ~(1 << PD2) & ~(1 << PD7); // Set the PD2 and PD7 as input
    EICRA |= (1 << ISC01); // Set the interrupt to trigger on falling edge
    EIMSK |= (1 << INT0); // Enable the interrupt

    SMCR |= (1 << SM1); // Set the sleep mode to power down
    sei(); // Enable the global interrupt
    lcd_init(LCD_DISP_ON); // Initialize the LCD
}

float time_from_start = 0; // Create a variable to store the time from start
volatile float counter = 0; // Create a variable to store the counter

int main(void) 
{
    setup(); // Run the code in the setup

    while (1) {
        char count[16]; 
        dtostrf(counter, 3, 0, count); 
        if((PIND & (1 << PD7))!= 0) { // Run when sleep button is pressed
            lcd_clrscr(); 
            lcd_puts("Sleep count: \n"); // Print the sleep count to the screen with newline character included
            lcd_puts(count);  
            SMCR |= (1 << SE); // write 1 in sleep control register bit 0 to enable sleep
            sleep_cpu(); 
            SMCR &= ~(1 << SE); // write 0 in sleep control register bit 0 to disable sleep
        } else {
            lcd_clrscr(); // clear the screen, we don't want to show the counter during timer
            time_from_start++; 
            char time_elapsed[16]; 
            dtostrf(time_from_start/((float)1000/DELAY), 3, 2, time_elapsed); 
            lcd_puts("Time: \n"); // Print the text to the screen with newline character included
            lcd_puts(time_elapsed); 
            _delay_ms(DELAY); 
        }
    }

    return 0;
}

ISR(INT0_vect) { // Increase the counter when sleep button is pressed
	counter++; 
}


