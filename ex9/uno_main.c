#define F_CPU 16000000UL
#define FOSC 16000000UL
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD-1)  //see datasheet p. 203 in the example code for C 

#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>


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

unsigned char spi_send_data[20]= "Hi from slave \n\r"; // Data to send to master
unsigned char spi_receive_data[20]; // Buffer for received data
volatile bool transfer_complete = false;
volatile int8_t spi_index = false; // Index for sending data
volatile int8_t spi_receive_index = false; // Index for receiving data

// Interrupt function
ISR (SPI_STC_vect) {
    unsigned char spi_interrupt_byte = SPDR;
    unsigned char transfer_end_check = '\r';
    SPDR = spi_send_data[spi_index]; // Send byte of data with SPI data register
    spi_index++; // Increment send data index
    // Store interrupt byte in receive data
    if (spi_receive_index < sizeof(spi_receive_data)){
        spi_receive_data[spi_receive_index++] = spi_interrupt_byte;

        // check if the received byte is '\r'
        if(spi_interrupt_byte == transfer_end_check){
            transfer_complete = true;
        }
    }
}

int main(void) {
    USART_init(MYUBRR);
    // Redirect STDIN and STDOUT to UART
    stdout = &uart_output;
    stdin = &uart_input;

    // Set pin and port of MISO as output
    DDRB |= (1 << PB4);

    // Enable SPI and SPI Interrupt Enable
    SPCR |=  (1 << SPE) | (1 << SPIE);
    SPDR = 0;
    
    sei();

    while(1) {
        if (transfer_complete == true){
            printf(spi_receive_data); // Print receive data
            // Reset transfer complete flag and indices
            transfer_complete = false; 
            spi_index = false;
            spi_receive_index = false;
        }
    }
    return 0;
}


