
#ifndef _ST7735_FONT_H_
#define _ST7735_FONT_H_

 #include <stdint.h>
// #include <avr/pgmspace.h>
// #include <stdio.h>

void st7735_print(const char str[]);
void st7735_setCursor(int16_t x, int16_t y);
void st7735_setTextColor(uint16_t c, uint16_t bg);
void st7735_setTextSize(uint8_t s);
void st7735_setTextWrap(uint8_t w);

int st7735_printf(char var, FILE *stream);


//int16_t height(void);
//int16_t width(void);

int16_t  WIDTH, HEIGHT;   // this is the 'raw' display w/h - never changes
int16_t  cursor_x, cursor_y;
uint16_t textcolor, textbgcolor;
uint8_t  textsize;
uint8_t  rotation;
uint8_t  wrap; // If set, 'wrap' text at right edge of display


#endif
