#define F_CPU 16000000UL // UL stands for Unsigned Long
#define BAUD 9600

#include <avr/io.h>
#include "keypad.h"
#include "lcd.h"
#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <stdlib.h>

void setup() {
    // initialize LCD
    lcd_init(LCD_DISP_ON);

    // initialize keypad
    KEYPAD_Init();
}


int main(void)
{
    setup();

    while (1) {
        char buf[0];
        // get keypad signal
        uint8_t ASCII_signal = KEYPAD_GetKey();
        uint8_t NUM_value = ASCII_signal - '0';
        
        // convert signal value to string
        itoa(NUM_value, buf, 10);

        // tää kohta tarvitaan jotta ei-numeromerkit näkyy oikein 
        buf[0] = ASCII_signal;
        buf[1] = '\0';

        // clear LCD screen
        lcd_clrscr();

        // display output on LCD screen
        lcd_puts(buf);
        _delay_ms(1000);
    }
    return 0;
}