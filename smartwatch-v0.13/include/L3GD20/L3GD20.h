#ifndef L3GD20_h
#define L3GD20_h

#include <stdint.h>
#include <stdio.h>
#include "vector.h"
    
vector_t g;
void L3GD20_enableDefault(void);
void L3GD20_write8(uint8_t reg, uint8_t value);
uint8_t L3GD20_read8(uint8_t reg);
void L3GD20_read(void);
	
#endif



