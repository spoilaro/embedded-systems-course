#define F_CPU 16000000UL
#define FOSC 16000000UL
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD-1)  //see datasheet p. 203 in the example code for C 

#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <avr/io.h>
#include <stdbool.h>
#include <string.h>


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

// Setup buffers for input and output
FILE uart_output = FDEV_SETUP_STREAM(USART_Transmit, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, USART_Receive, _FDEV_SETUP_READ);

int main(void) {
    USART_init(MYUBRR);
    // Redirect STDIN and STDOUT to USART
    stdout = &uart_output;
    stdin = &uart_input;

    DDRB &= ~(1 << PB0); // pin 8 for read 
    DDRD &= ~(1 << PD5); // pin 5 for write

    bool button_read = 0;
    bool button_write = 0;
    uint16_t max_address = 32;
    char input_data[32]; // used when reading the data from EEPROM
    char output_data[32] = "This is the initial sentence\n"; // data to be written to EEPROM. 

    char output_data_1[32] = "Hey, I am sentence 1\n"; // Sentence 1
    char output_data_2[32] = "I have changed to sentence 2\n"; // Sentence 2
    bool b_switch_text = 0;

    while(1) {
        button_read = (PINB & (1 << PB0)); // Status from read button
        button_write = (PIND & (1 << PD5)); // Status from write button
        if (1 == button_read) {
            // When the read button is pressed, read and print the data from EEPROM to USART
            for(uint16_t address_index = 0; address_index < max_address; address_index++) {
                EEAR = address_index; // Set up address register
                EECR |= (1 << EERE); // Start EEPROM read 
                input_data[address_index] = EEDR; // Read value from data register
            }
            printf(input_data);
            _delay_ms(500);
        } else if (1 == button_write) {
            // handle setup for writing
            for(uint16_t address_index = 0; address_index < sizeof(output_data); address_index++) {
                while(EECR & (1 << EEPE)) {} // wait for an ongoing write operation to end
                EEAR = address_index; // Set up address register
                EEDR = output_data[address_index]; // Set up data register
                EECR |= (1 << EEMPE); // Enable master write
                EECR |= (1 << EEPE); // Enable EEPROM write
            }
            printf(output_data);
            
            // Switch between the two different outputs
            if (0 == b_switch_text) {
                strcpy(output_data, output_data_2); // Copies the data to correct buffer
                b_switch_text = 1;
            } else if (1 == b_switch_text) {
                strcpy(output_data, output_data_1); // Copies the data to correct buffer
                b_switch_text = 0;
            }
            _delay_ms(500);
        }
    }
    return 0;
}


