#include "spi_native.h"

/*
1284:
CS:PB4
PB6:MISO (not used for ST7735)
PB5:MOSI (not used)
PB7:SCK
*/

void spi_init(void) {
	// Set MOSI and SCK, SS/CS output, all others input
	DDRB |= (1<<PB7) | (1<<PB5) | (1<<PB4);
	// Enable SPI, Master, set clock rate fck/4, mode 0
	SPCR = (1<<SPE) | (1<<MSTR);

	// Set SS/CS
	PORTB |= (1 << PB4);
}
