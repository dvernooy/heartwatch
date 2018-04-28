#ifndef LSM303_h
#define LSM303_h
#include "vector.h"

vector_t a; // accelerometer readings
vector_t m; // magnetometer readings
vector_t m_max; // maximum magnetometer values, used for calibration
vector_t m_min; // minimum magnetometer values, used for calibration

void LSM303_enableDefault(void);
	
void LSM303_write8(uint8_t reg, uint8_t value);
uint8_t LSM303_read8(uint8_t reg);

void LSM303_readAcc(void);
void LSM303_readMag(void);
void LSM303_read(void);

void LSM303_setTimeout(unsigned int timeout);
unsigned int LSM303_getTimeout(void);
uint8_t LSM303_timeoutOccurred(void);

float LSM303_heading(void);
float LSM303_heading_calc(vector_t from);

#endif
