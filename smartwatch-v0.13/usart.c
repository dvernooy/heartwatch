/********************************************************************************
                                Includes
********************************************************************************/
#include <avr/io.h>
#include <stdio.h>
#include <stdbool.h>
#include "usart.h"
/********************************************************************************
                                Function Prototypes
********************************************************************************/
void usart_putchar(char data); 
void usart_pstr(char *s);

/********************************************************************************
                                usart Related
********************************************************************************/

void usart_init( uint16_t ubrr) {
	// Set baud rate
	UBRR0H = (uint8_t)(ubrr>>8);
	UBRR0L = (uint8_t)ubrr;
	// Enable receiver and transmitter
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	// Set frame format: 8data, 1stop bit, no parity
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}


void usart_putchar(char data) { 
	// Wait for empty transmit buffer
	while ( !(UCSR0A & (_BV(UDRE0))) );
	// Start transmission
	UDR0 = data; 
}

char usart_getchar(void) { 
	// Wait for incomming data
	while ( !(UCSR0A & (_BV(RXC0))) );
	// Return the data
	return UDR0;
} 

void usart_pstr(char *s) {
    // loop through entire string
	while (*s) {  
        usart_putchar(*s);
        s++;
    }
}

unsigned char usart_kbhit(void) {
	//return nonzero if char waiting  polled version
	unsigned char b;
	b=0;
	if(UCSR0A & (1<<RXC0)) b=1;
	return b;
}

// this function is called by printf as a stream handler
int usart_putchar_printf(char var, FILE *stream) {
	usart_putchar(var);
	return 0;
}





