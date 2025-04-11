#define F_CPU 16000000UL
#define FOSC 16000000UL
#define BAUD 9600
#define MYUBRR (FOSC / 16 / BAUD - 1) // see datasheet p. 203 in the example code for C
#define SLAVE_ADDRESS 0b1010111       // 87 as decimal.
#define DELAY 100

// Pins of components

// Define states for elevator
#define IDLE 0
#define OPEN 1
#define UP 2
#define DOWN 3
#define FAULT 4
#define EMERGENCY_START 5
#define EMERGENCY 6
#define EMERGENCY_STOP 7

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

// Function prototypes
int ascii_signal_to_number(uint8_t signal);
void send_fault_and_reset();

int ascii_signal_to_number(uint8_t signal)
{
    uint8_t result = signal - '0';
    return result;
}

void send_fault_and_reset() {
    state = FAULT;
    send_state();
    state = IDLE;
    send_state();
}

void lcd_string_to_screen(char* signal) {
    lcd_clrscr();
    lcd_puts(signal);
}

void lcd_number_to_screen(uint8_t number) {
    char str[3];
    itoa(ascii_signal_to_number(number), str, 10);
    lcd_string_to_screen(str);
}

// reads button states and returns the pushed button.
// If several buttons are pressed, the lowest one is returned.
// arguments: current floor, int8_t
// returns: 
void floor_button_choice(int8_t *current_floor_button)
{

    // Compose buffer of numbers until "a" (confirm-button) is pressed
    uint8_t signal_buffer[2];
    uint8_t count = 0;
    uint8_t ASCII_signal;
    
    while (true) // While confirm button not pressed
    {
        // Read signal from keypad
        ASCII_signal = KEYPAD_GetKey();
        uint8_t NUM_value = ASCII_signal - '0';
        // User didn't input any floor numbers.
        // Sends fault status
        if(ASCII_signal == 'A' && count == 0) {
            send_fault_and_reset();
            continue;
        // User confirms inputted floor numbers.
        // Loop breaks and logic continues forward.
        } else if (ASCII_signal == 'A' && count != 0) {
            break;
        // Default case
        // Keypad returns "empty" signal 
        } else if (ASCII_signal == 'z') {
            continue;
        // User tries to input more than 2 floor numbers.
        // Sends fault status, reset count, buffer values overwritten later
        } else if (count >= 2 && ASCII_signal != 'A') {
            send_fault_and_reset();
            count = 0;
            continue;
        // Add to count and add input to buffer
        // Default case for correct floor input
        // Check if ASCII signal is between 0 and 9
        } else if (ascii_signal_to_number(ASCII_signal) >= 0 && ascii_signal_to_number(ASCII_signal) < 11) {
            signal_buffer[count] = ASCII_signal;
            count++;
            
        }
    }

    // Converts individual ASCII signals to numbers
    uint8_t n1 = ascii_signal_to_number(signal_buffer[0]);
    uint8_t n2 = ascii_signal_to_number(signal_buffer[1]);
    
    // Composites two numbers to one string + null terminator
    // E.g 2 and 3 becomes 23
    char c[3];
    sprintf(c, "%d%d", n1, n2);

    *current_floor_button = atoi(c);
}

static void USART_init(uint16_t ubrr)
{
    /* Set baud rate */
    UBRR0H = (unsigned char)(ubrr >> 8); // datasheet p.206
    UBRR0L = (unsigned char)ubrr;        // datasheet p.206
    /* Enable receiver and transmitter */
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0);  // datasheet p.206
    UCSR0C |= (1 << USBS0) | (3 << UCSZ00); // datasheet p.221 and p.222
}

static void USART_Transmit(unsigned char data, FILE *stream)
{ // datasheet p.207
    while (!(UCSR0A & (1 << UDRE0)))
    { // Waits until the transmit buffer is empty
        ;
    }
    UDR0 = data; // When the wait is done, writes the data to transmit register
}

static char USART_Receive(FILE *stream)
{
    while (!(UCSR0A & (1 << RXC0)))
    { // Waits until the receive buffer is filled
        ;
    }
    // When the wait is done, return UDR0;
    return UDR0;
}

void lcd_write_cur_floor(uint8_t floor_current) {
    char floor_str[3];
    itoa(floor_current, floor_str, 10);
    lcd_clrscr();
    lcd_puts(floor_str);
}

void send_state()
{
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

// Setup buffers for input and output
FILE uart_output = FDEV_SETUP_STREAM(USART_Transmit, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, USART_Receive, _FDEV_SETUP_READ);

int main(void)
{
    lcd_init(LCD_DISP_ON);
    KEYPAD_Init();
    USART_init(MYUBRR);
    // Redirect STDIN and STDOUT to UART
    stdout = &uart_output;
    stdin = &uart_input;

    TWBR = 0x03;        // TWI bit rate register, SCL frequency set to 400kHz
    TWSR = 0x00;        // TWI status register, prescaler set to 1
    TWCR = (1 << TWEN); // Enable TWI

    static int8_t requested_floor = -1;
    static int8_t floor_current = 1;
    static bool b_doors_opened = false;

    char test_char_array[16];
    uint8_t twi_status = 0;
    
    uint8_t ASCII_signal = KEYPAD_GetKey();
    lcd_number_to_screen(ASCII_signal);
    _delay_ms(100000);
    

    // TODO: add emergency states
    while (1)
    {
        // TODO: add emergency states / Check when emergency button is pressed
        // TODO: emergency states order EMERGENCY_START ->  EMERGENCY -> EMERGENCY_STOP
        // TODO: blink movement led -> play song indefinitely -> read keypress to stop emergency
        
        switch (state)
        {
        case IDLE:
            lcd_clrscr();
            lcd_puts("Choose the floor");
            floor_button_choice(&requested_floor);
            ASCII_signal = KEYPAD_GetKey();

            // Check if the requested floor is the same as the current floor
            if (requested_floor == floor_current)
            {
                state = FAULT;
                send_state();
                // requested_floor = -1;
                state = IDLE;
            }
            // Case of default state of the requested floor
            else if (requested_floor == -1)
            {
                state = IDLE;
            }
            else if (requested_floor < floor_current)
            {
                state = DOWN;
                send_state();
            }
            else if (requested_floor > floor_current)
            {
                state = UP;
                send_state();
            }
            break;

        case UP:
            lcd_write_cur_floor(floor_current);
            if (requested_floor > floor_current)
            {
                // GOING UP TO THE NEXT FLOOR
                state = UP;
                floor_current++;
                _delay_ms(1000);
            }
            else if (requested_floor == floor_current)
            {
                // FLOOR REACHED
                state = OPEN;
                send_state();
            }
            break;

        case DOWN:
            lcd_write_cur_floor(floor_current);
            if (requested_floor < floor_current)
            {
                // GOING DOWN TO THE NEXT FLOOR
                state = DOWN;

                _delay_ms(1000);
                if (1 == floor_current)
                {
                    ;
                }
                floor_current--;
            }
            else if (requested_floor == floor_current)
            {
                // FLOOR REACHED
                state = OPEN;
            }
            break;

        case OPEN:  
            if (!b_doors_opened)
            {
                lcd_clrscr();
                lcd_puts("Door open");
                b_doors_opened = true;
                state = OPEN;
                send_state();
                _delay_ms(5000);
            }
            else
            {
                // DOORS CLOSED
                lcd_clrscr();
                lcd_puts("Door closed");
                state = IDLE;
                b_doors_opened = false;
                send_state();
                _delay_ms(2000);
            }
            break;

        default:
            // This state should not be reached.
            state = IDLE;
            break;
        }

        send_state();
        _delay_ms(1000);
    }
    return 0;
}

// void
// lcd_puts_line(int line, char* content) {
//     lcd_gotoxy(0, line);
//     lcd_puts("                "); // empty the buffer
//     lcd_gotoxy(0, line);
//     lcd_puts(content); // replace content
// }