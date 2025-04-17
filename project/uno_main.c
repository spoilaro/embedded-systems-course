#define F_CPU 16000000UL
#define FOSC 16000000UL
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD-1)  //see datasheet p. 203 in the example code for C 
#define SLAVE_ADDRESS 0b1010111 // 87 as decimal.

#define IDLE 0
#define OPEN 1
#define UP 2
#define DOWN 3
#define FAULT 4
#define EMERGENCY_START 5
#define EMERGENCY 6
#define EMERGENCY_STOP 7

#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <avr/io.h>


ISR (TIMER1_COMPA_vect) {

}

void setup_buzzer() {
    DDRB |= (1 << PB1);

    TCNT1 = 0;
    // Reset the TCCR registers
    TCCR1B = 0;
    TCCR1A = 0;
    // vvv Not sure if correct vvv
    TCCR1A |= (1 << COM1A0); // Set toggle mode for OC1A (PB1)
    TCCR1A |= (1 << WGM10); // Set phase and frequency correct PWM mode, with OCR1A as the TOP
    TCCR1B |= (1 << WGM13);
    TIMSK1 |= (1 << OCIE1A);
    TCCR1B |= (1 << CS10); // no prescaler

    /* Prescalers
    TCCR1B |= (1 << CS10) // no prescaler
    TCCR1B |= (1 << CS11) // 8 prescaler
    TCCR1B |= (1 << CS10) | (1 << CS11) // 64 prescaler
    */
    sei();
}

void play_melody() {

    /*
    megalovania top values
    27239 x2
    13622
    18182

    */

    OCR1A = 27239;
    _delay_ms(100);
    OCR1A = 0;
    _delay_ms(8);
    OCR1A = 27239;
    _delay_ms(100);
    OCR1A = 0;
    _delay_ms(8);
    OCR1A = 13622;
    _delay_ms(100);
    OCR1A = 0;
    _delay_ms(15);
    OCR1A = 18182;
    _delay_ms(100);
    OCR1A = 0;
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

void blink_movement() {
    for (int blinks = 0; blinks < 3; blinks++) {
        PORTB |= (1 << PB0); // Turn on the LED
        _delay_ms(100); // Wait for 100ms
        PORTB &= ~(1 << PB0); // Turn off the LED
        _delay_ms(100); // Wait for 100ms
    }
}

void setup_twi() {
    TWCR |= (1 << TWEA) | (1 << TWEN);
    TWCR &= ~(1 << TWSTA) & ~(1 << TWSTO);

    TWAR = 0b10101110; // 7-bit slave address and 1 write bit (LSB)
}

// Setup buffers for input and output
FILE uart_output = FDEV_SETUP_STREAM(USART_Transmit, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, USART_Receive, _FDEV_SETUP_READ);

int main(void) {
    USART_init(MYUBRR);
    // Redirect STDIN and STDOUT to UART
    stdout = &uart_output;
    stdin = &uart_input;

    setup_twi();
    setup_buzzer();

    play_melody();

    uint8_t twi_receive_data; // Use TWI instead of SPI
    char test_char_array[16]; // 16-bit array, assumes that the int given is 16-bits 
    uint8_t twi_status = 0; 

    /* 
        PB0, movement LED
        PB2, door open LED
    */
    DDRB |= (1<<PB0); // Set the pin as output
    DDRB |= (1<<PB2); // Set the pin as output

    while(1) {
        while(!(TWCR & (1 << TWINT))) // Wait for the interrupt flag to be set
        {;}

        twi_status = (TWSR & 0xF8); // Mask the status register

        TWCR |= (1 << TWINT) | (1 << TWEA) | (1 << TWEN); // Clear the interrupt flag

        while(!(TWCR & (1 << TWINT))) // Wait for the interrupt flag to be set
        {;}

        twi_status = (TWSR & 0xF8);

        if((twi_status == 0x80) || (twi_status == 0x90)) // Data received, ACK returned
        {
            twi_receive_data = TWDR; // Store the data in the buffer
        } else if((twi_status == 0x88) || (twi_status == 0x98)) // Data received, No ACK returned
        {
            twi_receive_data = TWDR; // Store the data in the buffer
        } else if(twi_status == 0xA0) // Check if the stop condition was received
        {
            TWCR |= (1 << TWINT); // Clear the interrupt flag
        }
        
        switch(twi_receive_data) {
            case OPEN:
                // Open door LED on and LCD displays "Door is open" text for 5 seconds
                // "Door is closed" text on LCD for 1 seconds
                PORTB &= ~(1 << PB0); // Turn off movement the LED
                PORTB |= (1 << PB2); // Turn on door the LED
                break;

            case UP:
                PORTB |= (1 << PB0); // Turn on the movement LED
                PORTB &= ~(1 << PB2); // Turn off the door open LED
                break;

            case DOWN:
                PORTB |= (1 << PB0); // Turn on the movement LED
                PORTB &= ~(1 << PB2); // Turn off the door LED
                break;

            case FAULT: // Triggered when the same floor as the current floor is selected
                blink_movement(); // Call the blink function
                break;

            case EMERGENCY_START: // Triggered on emergency button press
                // Movement LED blinks 3 times
                blink_movement(); // Call the blink function
                break;
            
            case EMERGENCY: // Triggered on random keypad button press
                // Open door and play the melody infinitely
                // When another random button is pressed
                play_melody(); // Call the play melody function
                PORTB |= (1 << PB2); // Turn on the door LED
                break;

            case EMERGENCY_STOP: // Triggered on random keypad button press
                // Stop the melody and close the door
                PORTB &= ~(1 << PB2); // Turn off the door LED
                break;

            default: // case IDLE
                PORTB&= ~(1 << PB2); // Turn off the door LED
                break;

        }
    }
    return 0;
}
