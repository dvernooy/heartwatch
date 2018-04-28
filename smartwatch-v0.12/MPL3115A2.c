/*!
 * @file MPL3115A2.c
 *
 * @mainpage Adafruit MPL3115A2 alitmeter
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's MPL3115A2 driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit MPL3115A2 breakout: https://www.adafruit.com/products/1893
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * @section author Author
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */


#include "i2cmaster.h"
#include "MPL3115A2.h"
#include "nil.h"
//#include <util/delay.h>

/**************************************************************************/
/*!
    @brief  Instantiates a new MPL3115A2 class
*/
/**************************************************************************/


/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
    @param twoWire Optional TwoWire I2C object
    @return true on successful startup, false otherwise
*/
/**************************************************************************/
uint8_t MPL3115A2_init(void) {
  
  uint8_t whoami =  MPL3115A2_read8(MPL3115A2_WHOAMI);
  if (whoami != 0xC4) {
    return (0);
  }

  MPL3115A2_write8(MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_RST);
  //_delay_ms(10);
  chThdSleepMilliseconds(10);

  while(MPL3115A2_read8(MPL3115A2_CTRL_REG1) & MPL3115A2_CTRL_REG1_RST) {
  //_delay_ms(10);
  chThdSleepMilliseconds(10);
  }

  _ctrl_reg1.reg = MPL3115A2_CTRL_REG1_OS8 | MPL3115A2_CTRL_REG1_ALT;

  MPL3115A2_write8(MPL3115A2_CTRL_REG1, _ctrl_reg1.reg);

  MPL3115A2_write8(MPL3115A2_PT_DATA_CFG, 
	 MPL3115A2_PT_DATA_CFG_TDEFE |
	 MPL3115A2_PT_DATA_CFG_PDEFE |
	 MPL3115A2_PT_DATA_CFG_DREM);
  return (1);
}

/**************************************************************************/
/*!
    @brief  Gets the floating-point pressure level in kPa
    @return altitude reading as a floating point value
*/
/**************************************************************************/
float MPL3115A2_getPressure() {
  uint32_t pressure;

  while(MPL3115A2_read8(MPL3115A2_CTRL_REG1) & MPL3115A2_CTRL_REG1_OST) {
  //_delay_ms(10);
  chThdSleepMilliseconds(10);
  }

  _ctrl_reg1.bit.ALT = 0;
  MPL3115A2_write8(MPL3115A2_CTRL_REG1, _ctrl_reg1.reg);

  _ctrl_reg1.bit.OST = 1;
  MPL3115A2_write8(MPL3115A2_CTRL_REG1, _ctrl_reg1.reg);

  uint8_t sta = 0;
  while (! (sta & MPL3115A2_REGISTER_STATUS_PDR)) {
    sta = MPL3115A2_read8(MPL3115A2_REGISTER_STATUS);
    //_delay_ms(10);
	chThdSleepMilliseconds(10);
  }
  pressure = MPL3115A2_read8(MPL3115A2_REGISTER_PRESSURE_MSB); // receive DATA
  pressure <<= 8;
  pressure |= MPL3115A2_read8(MPL3115A2_REGISTER_PRESSURE_CSB); // receive DATA
  pressure <<= 8;
  pressure |= MPL3115A2_read8(MPL3115A2_REGISTER_PRESSURE_LSB); // receive DATA
  pressure >>= 4;

  float baro = pressure;
  baro /= 4.0;
  return baro;
}

/**************************************************************************/
/*!
    @brief  Gets the floating-point altitude value
    @return altitude reading as a floating-point value
*/
/**************************************************************************/
float MPL3115A2_getAltitude() {
  int32_t alt;

  while(MPL3115A2_read8(MPL3115A2_CTRL_REG1) & MPL3115A2_CTRL_REG1_OST) {
  //_delay_ms(10);
  chThdSleepMilliseconds(10);
  }

  _ctrl_reg1.bit.ALT = 1;
  MPL3115A2_write8(MPL3115A2_CTRL_REG1, _ctrl_reg1.reg);

  _ctrl_reg1.bit.OST = 1;
  MPL3115A2_write8(MPL3115A2_CTRL_REG1, _ctrl_reg1.reg);

  uint8_t sta = 0;
  while (! (sta & MPL3115A2_REGISTER_STATUS_PDR)) {
    sta = MPL3115A2_read8(MPL3115A2_REGISTER_STATUS);
    //_delay_ms(10);
	chThdSleepMilliseconds(10);
  }
  
  alt  = ((uint32_t)MPL3115A2_read8(MPL3115A2_REGISTER_PRESSURE_MSB)) << 24; // receive DATA
  alt |= ((uint32_t)MPL3115A2_read8(MPL3115A2_REGISTER_PRESSURE_CSB)) << 16; // receive DATA
  alt |= ((uint32_t)MPL3115A2_read8(MPL3115A2_REGISTER_PRESSURE_LSB)) << 8; // receive DATA

  float altitude = alt;
  altitude /= 65536.0;
  return altitude;
}

/**************************************************************************/
/*!
    @brief  Set the local sea level barometric pressure
    @param pascal the pressure to use as the baseline
*/
/**************************************************************************/
void MPL3115A2_setSeaPressure(float pascal) {
  uint16_t bar = pascal/2;
  i2c_start(_MPL3115A2_W_ADDRESS);  
  i2c_write((uint8_t)MPL3115A2_BAR_IN_MSB);              
  i2c_write((uint8_t)(bar>>8));
  i2c_write((uint8_t)bar);               
  i2c_stop();                    
}

/**************************************************************************/
/*!
    @brief  Gets the floating-point temperature in Centigrade
    @return temperature reading in Centigrade as a floating-point value
*/
/**************************************************************************/
float MPL3115A2_getTemperature() {
  int16_t t;

  uint8_t sta = 0;
  while (! (sta & MPL3115A2_REGISTER_STATUS_TDR)) {
    sta = MPL3115A2_read8(MPL3115A2_REGISTER_STATUS);
    //_delay_ms(10);
	chThdSleepMilliseconds(10);
  }
  
  t = MPL3115A2_read8(MPL3115A2_REGISTER_TEMP_MSB); // receive DATA
  t <<= 8;
  t |= MPL3115A2_read8(MPL3115A2_REGISTER_TEMP_LSB); // receive DATA
  t >>= 4;
  
  if (t & 0x800) {
    t |= 0xF000;
  }

  float temp = t;
  temp /= 16.0;
  return temp;
}




/**************************************************************************/
/*!
    @brief  read 1 byte of data at the specified address
    @param a the address to read
    @return the read data byte
*/
/**************************************************************************/

// Read from MPL3115A2 sensor
uint8_t MPL3115A2_read8(uint8_t address) {
  uint8_t tmp = 0;

  i2c_start(_MPL3115A2_W_ADDRESS);   // send byte via I2C (device address + W)
  i2c_write(address);               // send byte (data address)
 
  i2c_rep_start(_MPL3115A2_R_ADDRESS);          // issue I2C signal repeated start
  tmp = i2c_readNak();       // Read the data (NO acknowledge)
  i2c_stop();                    // issue I2C stop signal

  return tmp;
}

/**************************************************************************/
/*!
    @brief  write a byte of data to the specified address
    @param a the address to write to
    @param d the byte to write
*/
/**************************************************************************/

// Write to MPL3115A2 sensor
void MPL3115A2_write8(uint8_t address, uint8_t data_) {
  i2c_start(_MPL3115A2_W_ADDRESS);  // send byte via I2C  (device address + W)
  i2c_write(address);               // send byte (address of the location)
  i2c_write(data_);                 // send data (data to be written)
  i2c_stop();                    // issue I2C stop signal
}
