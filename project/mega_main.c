#define F_CPU 16000000UL
#define FOSC 16000000UL
#define BAUD 9600
#define MYUBRR (FOSC / 16 / BAUD - 1) // see datasheet p. 203 in the example code for C
#define SLAVE_ADDRESS 0b1010111       // 87 as decimal.

// Define states for elevator
#define IDLE 0
#define OPEN 1
#define UP 2
#define DOWN 3
#define FAULT 4
#define EMERGENCY_START 5
#define EMERGENCY 6
#define EMERGENCY_STOP 7

// Error codes for button dialog
#define BUTTON_DIALOG_OK 0 // Code when the button dialog was successful
#define BUTTON_DIALOG_INVALID_INPUT 1 // Code when the input given was invalid

#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "lcd.h"
#include <stdbool.h>
#include "keypad.h"

volatile uint8_t state = IDLE;
bool emergency_flag = false;

ISR(INT2_vect) {
    // Only start emergency if elevator is moving
    if (state != UP && state != DOWN) {
        return;
    }
    emergency_protocol();
} 

/*
    Handles the emergency sequence and sets the emergency flag to true.
*/
void emergency_protocol() {
    set_and_send_state(EMERGENCY_START);
    lcd_string_to_screen("EMERGENCY");
    /*
        Get input to open door
        send EMERGENCY state -> plays melody and opens door
    */
    KEYPAD_GetKey();
    set_and_send_state(EMERGENCY);
    /*
        Wait and stop emergency
        send EMERGENCY_STOP state -> close door
    */
    _delay_ms(3000);
    set_and_send_state(EMERGENCY_STOP);
    set_state(IDLE);
    lcd_clrscr();
    emergency_flag = true;
}

/* 
    Converts ASCII signal to number.
    arguments: signal, uint8_t
    returns: number, uint8_t
*/
uint8_t ascii_signal_to_number(uint8_t signal)
{
    uint8_t result = signal - '0';
    return result;
}

/* 
    Sends fault status and resets the state to IDLE.
    arguments: signal, char*
*/
void send_fault_and_reset(char * message) {
    lcd_string_to_screen(message);
    set_and_send_state(FAULT);
    _delay_ms(1000);
    set_and_send_state(IDLE);
}

/* 
    Simple function for printing strings to the LCD screen.
    arguments: signal, char*
*/
void lcd_string_to_screen(char* message) {
    if (emergency_flag) {
        return;
    }
    lcd_clrscr();
    lcd_puts(message);
}

/* 
    Converts number to ASCII signal and prints it to the LCD screen.
    arguments: number, uint8_t
*/
void lcd_number_to_screen(uint8_t number) {
    char str[3];
    itoa(number, str, 10);
    lcd_string_to_screen(str);
}

/* 
    Reads button states and returns the pushed button.
    arguments: current floor, uint8_t
    returns: status code of the button dialog, int
    0 - success, 1 - invalid input
 */
int floor_button_choice(uint8_t *current_floor_button)
{

    // Compose buffer of numbers until "a" (confirm-button) is pressed
    uint8_t signal_buffer[2];
    uint8_t count = 0;
    uint8_t ASCII_signal;
    
    while (true) // While confirm button not pressed
    {
        if (emergency_flag) {
            return BUTTON_DIALOG_INVALID_INPUT;
        }
        // Read signal from keypad. Waits for input.
        ASCII_signal = KEYPAD_GetKey();
        
        /* 
            User didn't input any floor numbers and pressed "A" (confirm button)
            Sends fault status
        */
        if(ASCII_signal == 'A' && count == 0) {
            send_fault_and_reset("Max floor is 99");
            return BUTTON_DIALOG_INVALID_INPUT;
        
        /* 
            User pressed "A" (confirm button) and inputted 1 or 2 floor numbers
        */
        } else if (ASCII_signal == 'A' && count != 0) {
            break;
        
        /* 
            User tries to input more than 2 floor numbers.
            returns invalid input status code
        */
        } else if (count >= 2 && ASCII_signal != 'A') {
            send_fault_and_reset("Invalid floor");
            return BUTTON_DIALOG_INVALID_INPUT;
        
        /* 
            Add to count and add input to buffer
            Default case for correct floor input
            Check if ASCII signal is between 0 and 9
        */
        } else if (ascii_signal_to_number(ASCII_signal) >= 0 && ascii_signal_to_number(ASCII_signal) < 11) {
            signal_buffer[count] = ASCII_signal;
            count++;
        }
    }

    // Converts individual ASCII signals to numbers
    uint8_t n1 = ascii_signal_to_number(signal_buffer[0]);
    uint8_t n2 = ascii_signal_to_number(signal_buffer[1]);
    if(count < 2)
    {
        *current_floor_button = n1;
    } else {
        // Composites two numbers to one string + null terminator
        // E.g 2 and 3 becomes 23
        char c[3];
        sprintf(c, "%d%d", n1, n2);
        *current_floor_button = atoi(c);
    }
    lcd_number_to_screen(*current_floor_button);
    _delay_ms(1000);
    return BUTTON_DIALOG_OK;
}

/*
    Prints the current floor number to the LCD screen. Returns if emergency flag is true.
    arguments: floor number, uint8_t
*/
void lcd_write_cur_floor(uint8_t floor_current) {
    if (emergency_flag) {
        return;
    }
    char floor_str[3];
    itoa(floor_current, floor_str, 10);
    lcd_clrscr();
    lcd_puts(floor_str);
}

/*
    Transitions to a new state. Returns if emergency flag is true.
    arguments: new state, uint8_t
*/
void set_state(uint8_t new_state) {
    if (emergency_flag) {
        return;
    }
    state = new_state;
}

/*
    Sends current state data to slave. Returns if emergency flag is true.
*/
void send_state() {
    if (emergency_flag) {
        return;
    }
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // Enable 2-wire serial interface, start condition
    while (!(TWCR & (1 << TWINT)))                    // Wait until TWINT flag is set
    {
        ;
    }

    TWDR = 0b10101110;                 // Load slave address into TWDR register
    TWCR = (1 << TWINT) | (1 << TWEN); // Clear TWINT bit to start transmission of address
    while (!(TWCR & (1 << TWINT)))     // Wait until TWINT flag is set
    {
        ;
    }

    TWDR = state;                      // Load data into TWDR register
    TWCR = (1 << TWINT) | (1 << TWEN); // Clear TWINT bit to start transmission of data
    while (!(TWCR & (1 << TWINT)))     // Wait until TWINT flag is set
    {
        ;
    }

    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO); // Enable TWI, stop condition
}

/*
    Transitions to a new state and sends the state data to the slave.
    arguments: new state, uint8_t
*/
void set_and_send_state(uint8_t new_state)
{
    set_state(new_state);
    send_state();
}

/*
    Sets up and enables interrupts.
*/
void setup_interrupt() {
    cli();
    DDRD &= ~(1 << PD2) & ~(1 << PD7); // Set the PD2 and PD7 as input
    EICRA |= (1 << ISC21) | (1 << ISC20); // Set the interrupt to trigger on falling edge
    EIMSK |= (1 << INT2); // Enable the interrupt
    sei();

}

int main(void)
{
    lcd_init(LCD_DISP_ON);
    KEYPAD_Init();

    TWBR = 0x03;        // TWI bit rate register, SCL frequency set to 400kHz
    TWSR = 0x00;        // TWI status register, prescaler set to 1
    TWCR = (2 << TWEN); // Enable TWI
    setup_interrupt();
    
    static uint8_t requested_floor = 0;
    static uint8_t floor_current = 1;
    static bool b_doors_opened = false;

    char test_char_array[16];
    uint8_t twi_status = 0;

    while (1)
    {
        emergency_flag = false;
        
        switch (state)
        {
        case IDLE:
            lcd_string_to_screen("Choose the floor");

            int err_code = floor_button_choice(&requested_floor);
            if (err_code == BUTTON_DIALOG_INVALID_INPUT) {
                continue;
            }

            // Check if the requested floor is the same as the current floor
            if (requested_floor == floor_current)
            {
                send_fault_and_reset("Already on\nthis floor");
            }
            else if (requested_floor < floor_current)
            {
                set_and_send_state(DOWN);
            }
            else if (requested_floor > floor_current)
            {
                set_and_send_state(UP);
            }
            break;

        case UP:
            lcd_write_cur_floor(floor_current);
            if (requested_floor > floor_current)
            {
                // GOING UP TO THE NEXT FLOOR
                set_state(UP);
                if (!emergency_flag) {
                    floor_current++;
                }
                _delay_ms(100);
            }
            else if (requested_floor == floor_current)
            {
                // FLOOR REACHED
                set_state(OPEN);
            }
            break;

        case DOWN:
            lcd_write_cur_floor(floor_current);
            if (requested_floor < floor_current)
            {
                // GOING DOWN TO THE NEXT FLOOR
                set_state(DOWN);
                if (!emergency_flag) {
                    floor_current--;
                }
                _delay_ms(100);
            }
            else if (requested_floor == floor_current)
            {
                // FLOOR REACHED
                set_state(OPEN);
            }
            break;

        case OPEN:  
            if (!b_doors_opened)
            {
                lcd_string_to_screen("Door open");
                b_doors_opened = true;
                set_and_send_state(OPEN);
                _delay_ms(5000);
            }
            else
            {
                // DOORS CLOSED
                lcd_string_to_screen("Door closed");
                _delay_ms(500);
                set_and_send_state(IDLE);
                b_doors_opened = false;
            }
            break;

        default:
            // This state should not be reached.
            set_state(IDLE);
            break;
        }
        _delay_ms(100);
    }
    return 0;
}

