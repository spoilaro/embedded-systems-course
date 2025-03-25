#define F_CPU 16000000UL
#define FOSC 16000000UL
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD-1)  //see datasheet p. 203 in the example code for C 
#define SLAVE_ADDRESS 0b1010111 // 87 as decimal.

#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <avr/io.h>


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
    // Redirect STDIN and STDOUT to UART
    stdout = &uart_output;
    stdin = &uart_input;

    TWCR |= (1 << TWEA) | (1 << TWEN);
    TWCR &= ~(1 << TWSTA) & ~(1 << TWSTO);

    TWAR = 0b10101110; // 7-bit slave address and 1 write bit (LSB)

    char twi_receive_data[20]; // Use TWI instead of SPI
    char test_char_array[16]; // 16-bit array, assumes that the int given is 16-bits
    uint8_t twi_index = 0; 
    uint8_t twi_status = 0; 

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
            twi_receive_data[twi_index] = TWDR; // Store the data in the buffer
            twi_index++;
        } else if((twi_status == 0x88) || (twi_status == 0x98)) // Data received, No ACK returned
        {
            twi_receive_data[twi_index] = TWDR; // Store the data in the buffer
            twi_index++;
        } else if(twi_status == 0xA0) // Check if the stop condition was received
        {
            TWCR |= (1 << TWINT); // Clear the interrupt flag
        }
        // If buffer is full, print received data and reset index
        if (20 <= twi_index)
        {
            printf(twi_receive_data);
            twi_index = 0;
        }
    }
    return 0;
}
