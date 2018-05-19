

#ifndef __USART_H
#define __USART_H
/********************************************************************************
                                Macros and Defines
********************************************************************************/

#define BAUD 38400
//#define F_CPU 16000000
#define MYUBRR F_CPU/16/BAUD-1


//#include "datatypes.h"

/********************************************************************************
                                Function Prototypes
********************************************************************************/
void usart_init(uint16_t ubrr);
char usart_getchar(void);
unsigned char usart_kbhit(void);
int usart_putchar_printf(char var, FILE *stream);

#endif