#include <L3GD20.h>
#include <stdint.h>
#include "i2cmaster.h"
#include "usart.h"
#include <avr/pgmspace.h>
// NOTE: device is a D20, not a D20H

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define D20_SA0_HIGH_ADDRESS      0b1101011 // also applies to D20H 0x6B = 107
#define D20_SA0_LOW_ADDRESS       0b1101010 // also applies to D20H 0x6A
#define L3G4200D_SA0_HIGH_ADDRESS 0b1101001 //0x69
#define L3G4200D_SA0_LOW_ADDRESS  0b1101000 //0x68

#define TEST_REG_ERROR -1

#define D20H_WHO_ID     0xD7
#define D20_WHO_ID      0xD4
#define L3G4200D_WHO_ID 0xD3


#define L3GD20_ADDRESS                    	(0x6B)   
#define _L3GD20_W_ADDRESS                   ((L3GD20_ADDRESS <<1) & 0xFE)    
#define _L3GD20_R_ADDRESS                   ((L3GD20_ADDRESS <<1) | 0x01) 


// register addresses
enum 
{
       WHO_AM_I       = 0x0F,
       CTRL1          = 0x20, // D20H
       CTRL_REG1      = 0x20, // D20, 4200D
       CTRL2          = 0x21, // D20H
       CTRL_REG2      = 0x21, // D20, 4200D
       CTRL3          = 0x22, // D20H
       CTRL_REG3      = 0x22, // D20, 4200D
       CTRL4          = 0x23, // D20H
       CTRL_REG4      = 0x23, // D20, 4200D
       CTRL5          = 0x24, // D20H
       CTRL_REG5      = 0x24, // D20, 4200D
       REFERENCE      = 0x25,
       OUT_TEMP       = 0x26,
       STATUS         = 0x27, // D20H
       STATUS_REG     = 0x27, // D20, 4200D
       OUT_X_L        = 0x28,
       OUT_X_H        = 0x29,
       OUT_Y_L        = 0x2A,
       OUT_Y_H        = 0x2B,
       OUT_Z_L        = 0x2C,
       OUT_Z_H        = 0x2D,
       FIFO_CTRL      = 0x2E, // D20H
       FIFO_CTRL_REG  = 0x2E, // D20, 4200D
       FIFO_SRC       = 0x2F, // D20H
       FIFO_SRC_REG   = 0x2F, // D20, 4200D
       IG_CFG         = 0x30, // D20H
       INT1_CFG       = 0x30, // D20, 4200D
       IG_SRC         = 0x31, // D20H
       INT1_SRC       = 0x31, // D20, 4200D
       IG_THS_XH      = 0x32, // D20H
       INT1_THS_XH    = 0x32, // D20, 4200D
       IG_THS_XL      = 0x33, // D20H
       INT1_THS_XL    = 0x33, // D20, 4200D
       IG_THS_YH      = 0x34, // D20H
       INT1_THS_YH    = 0x34, // D20, 4200D
       IG_THS_YL      = 0x35, // D20H
       INT1_THS_YL    = 0x35, // D20, 4200D
       IG_THS_ZH      = 0x36, // D20H
       INT1_THS_ZH    = 0x36, // D20, 4200D
       IG_THS_ZL      = 0x37, // D20H
       INT1_THS_ZL    = 0x37, // D20, 4200D
       IG_DURATION    = 0x38, // D20H
       INT1_DURATION  = 0x38, // D20, 4200D
       LOW_ODR        = 0x39  // D20H
    };


static FILE usart_out = FDEV_SETUP_STREAM(usart_putchar_printf, NULL, _FDEV_SETUP_WRITE);

/*
Enables the L3G's gyro. Also:
- Sets gyro full scale (gain) to default power-on value of +/- 250 dps
  (specified as +/- 245 dps for L3GD20H).
- Selects 200 Hz ODR (output data rate). (Exact rate is specified as 189.4 Hz
  for L3GD20H and 190 Hz for L3GD20.)
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void L3GD20_enableDefault(void)
{
  
  // 0x00 = 0b00000000
  // FS = 00 (+/- 250 dps full scale)
  L3GD20_write8(CTRL_REG4, 0x00);
  
  // 0x6F = 0b01101111
  // DR = 01 (200 Hz ODR); BW = 10 (50 Hz bandwidth); PD = 1 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
  L3GD20_write8(CTRL_REG1, 0x6F);
}

// Writes an L3GD20 register
void L3GD20_write8(uint8_t reg, uint8_t data_) {
  i2c_start(_L3GD20_W_ADDRESS);  // send byte via I2C  (device address + W)
  i2c_write(reg);               // send byte (address of the location)
  i2c_write(data_);                 // send data (data to be written)
  i2c_stop();                    // issue I2C stop signal
}


// Reads an L3GD20 register
uint8_t L3GD20_read8(uint8_t reg) {
  uint8_t tmp = 0;
  i2c_start(_L3GD20_W_ADDRESS);   // send byte via I2C (device address + W)
  i2c_write(reg);               // send byte (data address)
  i2c_rep_start(_L3GD20_R_ADDRESS);          // issue I2C signal repeated start
  tmp = i2c_readNak();       // Read the data (NO acknowledge)
  i2c_stop();                    // issue I2C stop signal

  return tmp;
}


// Reads the 3 gyro channels and stores them in vector g
void L3GD20_read(void)
{

  uint8_t xlg = L3GD20_read8(OUT_X_L);
  uint8_t xhg = L3GD20_read8(OUT_X_H);
  uint8_t ylg = L3GD20_read8(OUT_Y_L);
  uint8_t yhg = L3GD20_read8(OUT_Y_H);
  uint8_t zlg = L3GD20_read8(OUT_Z_L);
  uint8_t zhg = L3GD20_read8(OUT_Z_H);

  // combine high and low bytes
  g.x = (float)(xhg << 8 | xlg);
  g.y = (float)(yhg << 8 | ylg);
  g.z = (float)(zhg << 8 | zlg);
}

