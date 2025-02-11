#define F_CPU 16000000UL // UL stands for Unsigned Long

#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>


void setup() {
    DDRB |= (1 << PB5);
    TCNT1 = 0;  // Reset the counter
    TCCR1B = 0; // Reset TCCRB1
    TCCR1A |= (1 << 6); //See datasheet p. 154 and 155
    TCCR1A |= (1 << 0); //See datasheet p. 154
    TCCR1B |= (1 << 4); //See datasheet p. 156
    TIMSK1 |= (1 << 1); // Enable timer/counter1 interrupt in TIMSK1 (See datasheet p. 161)
    OCR1A = 30534; // C4 262 Hz, no prescaler
    sei();
}

int main(void)
{
    setup();

    while (1) {
        // C4 262 Hz, no prescaler
        OCR1A = 30534;
        TCCR1B |= (1 << 0); // for no prescaler
        _delay_ms(1000);

        // E3 165 Hz, no prescaler
        OCR1A = 48485;
        _delay_ms(1000);
        
        // F2 87 Hz, 8 prescaler
        OCR1A = 11494; 
        TCCR1B &= ~(1 << 0); // reset prescaler
        TCCR1B |= (1 << 1); // for 8 prescaler
        _delay_ms(1000);
        
        // D0 18 Hz, 64 prescaler
        OCR1A = 6944;
        TCCR1B |= (1 << 0); // for 64 prescaler
        _delay_ms(1000);
        TCCR1B &= ~(1 << 0); // reset prescaler 
        TCCR1B &= ~(1 << 1); // reset prescaler    
    }

    return 0;
}

ISR (TIMER1_COMPA_vect) {

}

