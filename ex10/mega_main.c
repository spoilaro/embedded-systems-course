#define F_CPU 16000000UL
#define FOSC 16000000UL
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD-1)  //see datasheet p. 203 in the example code for C 
#define SLAVE_ADDRESS 0b1010111 // 87 as decimal.

#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <avr/io.h>
#include <stdlib.h>


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

    TWBR = 0x03; // TWI bit rate register, SCL frequency set to 400kHz
    TWSR = 0x00; // TWI status register, prescaler set to 1
    TWCR = (1 << TWEN); // Enable TWI

    unsigned char twi_send_data[20] = "master to slave\n";
    char test_char_array[16];
    uint8_t twi_status = 0;

    while(1) {
        TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // Enable 2-wire serial interface, start condition
        while(!(TWCR & (1 << TWINT))) // Wait until TWINT flag is set
            {;}

        twi_status = (TWSR & 0xF8); // Read status from TWI
        itoa(twi_status, test_char_array, 16); // Convert status to string
        printf(test_char_array); // Print status
        printf(" ");

        TWDR = 0b10101110; // Load slave address into TWDR register
        TWCR = (1 << TWINT) | (1 << TWEN); // Clear TWINT bit to start transmission of address
        while(!(TWCR & (1 << TWINT))) // Wait until TWINT flag is set
        {;}

        twi_status = (TWSR & 0xF8); // See line 61 - 64
        itoa(twi_status, test_char_array, 16);
        printf(test_char_array);
        printf(" ");

        for(int8_t twi_data_index = 0; twi_data_index < sizeof(twi_send_data); twi_data_index++)  { // Send data
            TWDR = twi_send_data[twi_data_index]; // Load data into TWDR register
            TWCR = (1 << TWINT) | (1 << TWEN); // Clear TWINT bit to start transmission of data
            while(!(TWCR & (1 << TWINT))) // Wait until TWINT flag is set
                {;}

            twi_status = (TWSR & 0xF8); // See line 61 - 64
            itoa(twi_status, test_char_array, 16);
            printf(test_char_array);
            printf(" ");
        }
        TWCR = (1 << TWINT) | (1 << TWEN) |(1 << TWSTO); // Enable TWI, stop condition
        printf("\n");
        _delay_ms(1000);
    }
    return 0;
}
