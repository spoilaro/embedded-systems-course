#define F_CPU 16000000UL // UL stands for Unsigned Long
#define BAUD 9600

#include "keypad.h"
#include "lcd.h"
#include <util/setbaud.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/io.h>


void setup() {
    // initialize LCD
    lcd_init(LCD_DISP_ON);

    // initialize keypad
    KEYPAD_Init();
}


int main(void)
{
    setup();

    char buf[8 * sizeof(int) + 1];

    while (1) {
        // get keypad signal
        uint8_t key = KEYPAD_GetKey();

        // convert signal value to string
        char* s = itoa(key, buf, 2);

        // clear LCD screen
        lcd_clrscr();

        // display output on LCD screen
        lcd_puts(s);
    }
}
