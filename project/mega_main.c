#define F_CPU 16000000UL
#define FOSC 16000000UL
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD-1)  //see datasheet p. 203 in the example code for C 
#define SLAVE_ADDRESS 0b1010111 // 87 as decimal.

//define states for elevator

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
#include <stdlib.h>
#include <lcd.h>

volatile uint8_t state = IDLE;


// reads buttonstates and returns the pushed button.
// If several buttons are pressed, the lowest one is returned.
// arguments: current floor, int8_t
// returns: none
int
floor_button_choice(int8_t *current_floor_button)
{
    // get keypad signal
    uint8_t ASCII_signal = KEYPAD_GetKey();
    uint8_t NUM_value = ASCII_signal - '0';

    *current_floor_button = NUM_value;
	
}

static void USART_init(uint16_t ubrr) {
    /* Set baud rate */ 
    UBRR0H = (unsigned char) (ubrr >> 8); //datasheet p.206
    UBRR0L = (unsigned char) ubrr;        //datasheet p.206 
    /* Enable receiver and transmitter */
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0); //datasheet p.206
    UCSR0C |= (1 << USBS0) | (3 << UCSZ00); //datasheet p.221 and p.222
}

static void USART_Transmit(unsigned char data, FILE *stream) { //datasheet p.207 
    while(!(UCSR0A & (1 << UDRE0))) { // Waits until the transmit buffer is empty
        ;
    }
    UDR0 = data; // When the wait is done, writes the data to transmit register
}

static char USART_Receive(FILE *stream) {
    while(!(UCSR0A & (1 << RXC0))) { // Waits until the receive buffer is filled
        ;
    }
    // When the wait is done, return UDR0;    
    return UDR0;
}

void send_state() {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // Enable 2-wire serial interface, start condition
    while(!(TWCR & (1 << TWINT))) // Wait until TWINT flag is set
        {;}

    TWDR = 0b10101110; // Load slave address into TWDR register
    TWCR = (1 << TWINT) | (1 << TWEN); // Clear TWINT bit to start transmission of address
    while(!(TWCR & (1 << TWINT))) // Wait until TWINT flag is set
    {;}

    TWDR = state; // Load data into TWDR register
    TWCR = (1 << TWINT) | (1 << TWEN); // Clear TWINT bit to start transmission of data
    while(!(TWCR & (1 << TWINT))) // Wait until TWINT flag is set
        {;}
    
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO); // Enable TWI, stop condition
}

// Setup buffers for input and output
FILE uart_output = FDEV_SETUP_STREAM(USART_Transmit, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, USART_Receive, _FDEV_SETUP_READ);

int main(void) {
    USART_init(MYUBRR);
    // Redirect STDIN and STDOUT to UART
    stdout = &uart_output;
    stdin = &uart_input;

    TWBR = 0x03; // TWI bit rate register, SCL frequency set to 400kHz
    TWSR = 0x00; // TWI status register, prescaler set to 1
    TWCR = (1 << TWEN); // Enable TWI

    // Enable input for port A
    DDRA &= ~(1 << PA0) & ~(1 << PA1) & ~(1 << PA2) & ~(1 << PA3) & ~(1 << PA4); // Floor buttons

    static int8_t requested_floor = -1;
	static int8_t floor_current = 1;
	static bool  b_doors_opened = false;

    char test_char_array[16];
    uint8_t twi_status = 0;

    while(1) {

        switch(state) {
			case IDLE:
                floor_button_choice(&requested_floor);

                // Check if the requested floor is the same as the current floor
                if (requested_floor == floor_current)
                {
                    state = FAULT;
                    send_state();
                    // requested_floor = -1;
                    state = IDLE;
                }
                // Case of default state of the requested floor
                else if (requested_floor == -1) {
                    state = IDLE;
                }
                else if (requested_floor < floor_current)
                {
                    state = DOWN;
                }
                else if (requested_floor > floor_current)
                {
                    state = UP;
                }
                break;
			
			case UP:
                if (requested_floor > floor_current)
                {
                    //GOING UP TO THE NEXT FLOOR
                    state = UP;
                    floor_current++;
                    _delay_ms(1000);
                    

                }
                else if (requested_floor == floor_current)
                {
                    //FLOOR REACHED
                    state = OPEN;
                }
                break;
			
			case DOWN:
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
                    
                    b_doors_opened = true;
                    state = OPEN;
                }
                else
                {
                    // DOORS CLOSED
                    
                    state = IDLE;
                    b_doors_opened = false;
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
