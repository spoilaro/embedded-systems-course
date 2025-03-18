#define F_CPU 16000000UL
#define FOSC 16000000UL
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD-1)  //see datasheet p. 203 in the example code for C 

#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <avr/io.h>
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
    // Redirect STDIN and STDOUT to UART
    stdout = &uart_output;
    stdin = &uart_input;

    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2); // Set MOSI, SCK and SS as output
    SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR0); // Enable SPI, Master mode, clock rate fck/16

    unsigned char spi_send_data[20] = "Hi from master \n\r"; 
    unsigned char spi_receive_data[20];

    while(1) {
        
        PORTB &= ~(1 << PB0); // Set SS low to enable slave
        for(int8_t spi_data_index = 0; spi_data_index < sizeof(spi_send_data); spi_data_index++) {
            SPDR = spi_send_data[spi_data_index]; // Start transmission
            _delay_us(5);
            // Wait for transmission to be complete by checking the interrupt flag in the SPI status register
            while(!(SPSR & (1 << SPIF))) { 
                ;
            }
            _delay_us(5);
            spi_receive_data[spi_data_index - 1] = SPDR; // Read data
        }
        PORTB |= (1 << PB0); // Set SS high to disable slave
        printf(spi_receive_data);
        _delay_ms(1000);
    }   
    return 0;
}


