// NOTE: device is a D, not DLH or DLM, DLHC
#include <LSM303.h>
#include <stdint.h>
#include "i2cmaster.h"
#include "usart.h"
#include <avr/pgmspace.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define D_SA0_HIGH_ADDRESS                0b0011101
#define D_SA0_LOW_ADDRESS                 0b0011110

#define D_WHO_ID    0x49

#define LSM303_ADDRESS                    	(0x1D)   
#define _LSM303_W_ADDRESS                   ((LSM303_ADDRESS <<1) & 0xFE)    
#define _LSM303_R_ADDRESS                   ((LSM303_ADDRESS <<1) | 0x01) 

#define PI 3.1415926
#define ACC_CAL 0.000061
#define MAG_CAL 0.160

static FILE usart_out = FDEV_SETUP_STREAM(usart_putchar_printf, NULL, _FDEV_SETUP_WRITE);

    // register addresses
    enum 
    {
      TEMP_OUT_L        = 0x05, // D
      TEMP_OUT_H        = 0x06, // D

      STATUS_M          = 0x07, // D
	  
	  OUT_X_L_M			= 0x08,
      OUT_X_H_M			= 0x09,
      OUT_Y_L_M			= 0x0A,
      OUT_Y_H_M			= 0x0B,
      OUT_Z_L_M			= 0x0C,
      OUT_Z_H_M			= 0x0D,

      INT_CTRL_M        = 0x12, // D
      INT_SRC_M         = 0x13, // D
      INT_THS_L_M       = 0x14, // D
      INT_THS_H_M       = 0x15, // D

      OFFSET_X_L_M      = 0x16, // D
      OFFSET_X_H_M      = 0x17, // D
      OFFSET_Y_L_M      = 0x18, // D
      OFFSET_Y_H_M      = 0x19, // D
      OFFSET_Z_L_M      = 0x1A, // D
      OFFSET_Z_H_M      = 0x1B, // D

      REFERENCE_X       = 0x1C, // D
      REFERENCE_Y       = 0x1D, // D
      REFERENCE_Z       = 0x1E, // D

      CTRL0             = 0x1F, // D
      CTRL1             = 0x20, // D
      CTRL2             = 0x21, // D
      CTRL3             = 0x22, // D
      CTRL4             = 0x23, // D
      CTRL5             = 0x24, // D
      CTRL6             = 0x25, // D
      CTRL7             = 0x26, // D
      STATUS_A          = 0x27, // D

      OUT_X_L_A         = 0x28,
      OUT_X_H_A         = 0x29,
      OUT_Y_L_A         = 0x2A,
      OUT_Y_H_A         = 0x2B,
      OUT_Z_L_A         = 0x2C,
      OUT_Z_H_A         = 0x2D,

      FIFO_CTRL         = 0x2E, // D
      FIFO_SRC          = 0x2F, // D

      IG_CFG1           = 0x30, // D
      IG_SRC1           = 0x31, // D
      IG_THS1           = 0x32, // D
      IG_DUR1           = 0x33, // D
      IG_CFG2           = 0x34, // D
      IG_SRC2           = 0x35, // D
      IG_THS2           = 0x36, // D
      IG_DUR2           = 0x37, // D

      CLICK_CFG         = 0x38, // D
      CLICK_SRC         = 0x39, // D
      CLICK_THS         = 0x3A, // D
      TIME_LIMIT        = 0x3B, // D
      TIME_LATENCY      = 0x3C, // D
      TIME_WINDOW       = 0x3D, // D

      Act_THS           = 0x3E, // D
      Act_DUR           = 0x3F, // D

      WHO_AM_I          = 0x0F, // D
	  
    };
   

/*
Enables the LSM303's accelerometer and magnetometer. Also:
- Sets sensor full scales (gain) to default power-on values, which are
  +/- 2 g for accelerometer and +/- 1.3 gauss for magnetometer
  (+/- 4 gauss on LSM303D).
- Selects 50 Hz ODR (output data rate) for accelerometer and 7.5 Hz
  ODR for magnetometer (6.25 Hz on LSM303D). (These are the ODR
  settings for which the electrical characteristics are specified in
  the datasheets.)
- Enables high resolution modes (if available).
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void LSM303_enableDefault(void)
{
	// Accelerometer

    // 0x00 = 0b00000000
    // AFS = 0 (+/- 2 g full scale)
    LSM303_write8(CTRL2, 0x00);

    // 0x57 = 0b01010111
    // AODR = 0001 (3 Hz ODR); AZEN = AYEN = AXEN = 1 (all axes enabled)
    LSM303_write8(CTRL1, 0x17);

    // Magnetometer
	
    /*
    These values lead to an assumed magnetometer bias of 0.
    Use the Calibrate example program to determine appropriate values
    for your particular unit. The Heading example demonstrates how to
    adjust these values in your own sketch.
    */
	m_min.x = m_min.y = m_min.z = -1* MAG_CAL* 32767;
	m_max.x = m_max.y = m_max.z = MAG_CAL* 32767;

    // 0x64 = 0b01100100
    // M_RES = 11 (high resolution mode); M_ODR = 001 (6.25 Hz ODR)
    LSM303_write8(CTRL5, 0x64);

    // 0x20 = 0b00100000
    // MFS = 01 (+/- 4 gauss full scale)
    LSM303_write8(CTRL6, 0x20);

    // 0x00 = 0b00000000
    // MLP = 0 (low power mode off); MD = 00 (continuous-conversion mode)
    LSM303_write8(CTRL7, 0x00);
}

// Writes an accelerometer register
void LSM303_write8(uint8_t reg, uint8_t value)
{
  i2c_start(_LSM303_W_ADDRESS);
  i2c_write(reg);
  i2c_write(value);
  i2c_stop();
}

// Reads an accelerometer register
uint8_t LSM303_read8(uint8_t reg)
{
  uint8_t value;
  
  i2c_start(_LSM303_W_ADDRESS);
  i2c_write(reg);
  i2c_rep_start(_LSM303_R_ADDRESS);
  value = i2c_readNak(); 
  i2c_stop();

  return value;
}

/*
The sensor outputs provided by the library are the raw 16-bit values
obtained by concatenating the 8-bit high and low accelerometer and
magnetometer data registers. They can be converted to units of g and
gauss using the conversion factors specified in the datasheet for your
particular device and full scale setting (gain).

Example: An LSM303D gives a magnetometer X axis reading of 1982 with
its default full scale setting of +/- 4 gauss. The M_GN specification
in the LSM303D datasheet (page 10) states a conversion factor of 0.160
mgauss/LSB (least significant bit) at this FS setting, so the raw
reading of -1982 corresponds to 1982 * 0.160 = 317.1 mgauss =
0.3171 gauss.

In the LSM303DLHC, LSM303DLM, and LSM303DLH, the acceleration data
registers actually contain a left-aligned 12-bit number, so the lowest
4 bits are always 0, and the values should be shifted right by 4 bits
(divided by 16) to be consistent with the conversion factors specified
in the datasheets.

Example: An LSM303DLH gives an accelerometer Z axis reading of -16144
with its default full scale setting of +/- 2 g. Dropping the lowest 4
bits gives a 12-bit raw value of -1009. The LA_So specification in the
LSM303DLH datasheet (page 11) states a conversion factor of 1 mg/digit
at this FS setting, so the value of -1009 corresponds to -1009 * 1 =
1009 mg = 1.009 g.
*/

// Reads the 3 accelerometer channels and stores them in vector a
void LSM303_readAcc(void)
{
 
  uint8_t xla = LSM303_read8(OUT_X_L_A);
  uint8_t xha = LSM303_read8(OUT_X_H_A);
  uint8_t yla = LSM303_read8(OUT_Y_L_A);
  uint8_t yha = LSM303_read8(OUT_Y_H_A);
  uint8_t zla = LSM303_read8(OUT_Z_L_A);
  uint8_t zha = LSM303_read8(OUT_Z_H_A);

  // combine high and low bytes
  // This no longer drops the lowest 4 bits of the readings from the DLH/DLM/DLHC, which are always 0
  // (12-bit resolution, left-aligned). The D has 16-bit resolution
  a.x = ACC_CAL* (double)((int16_t)(xha << 8 | xla));
  a.y = ACC_CAL* (double)((int16_t)(yha << 8 | yla));
  a.z = ACC_CAL* (double)((int16_t)(zha << 8 | zla));
}

// Reads the 3 magnetometer channels and stores them in vector m
void LSM303_readMag(void)
{

  uint8_t xlm, xhm, ylm, yhm, zlm, zhm;

    xlm = LSM303_read8(OUT_X_L_M);
    xhm = LSM303_read8(OUT_X_H_M);
    ylm = LSM303_read8(OUT_Y_L_M);
    yhm = LSM303_read8(OUT_Y_H_M);
    zlm = LSM303_read8(OUT_Z_L_M);
    zhm = LSM303_read8(OUT_Z_H_M);

  // combine high and low bytes
  m.x = MAG_CAL* (double)((int16_t)(xhm << 8 | xlm));
  m.y = MAG_CAL* (double)((int16_t)(yhm << 8 | ylm));
  m.z = MAG_CAL* (double)((int16_t)(zhm << 8 | zlm));
}

// Reads all 6 channels of the LSM303 and stores them in the object variables
void LSM303_read(void)
{
  LSM303_readAcc();
  LSM303_readMag();
}

/*
Returns the angular difference in the horizontal plane between the
"from" vector and north, in degrees.

Description of heading algorithm:
Shift and scale the magnetic reading based on calibration data to find
the North vector. Use the acceleration readings to determine the Up
vector (gravity is measured as an upward acceleration). The cross
product of North and Up vectors is East. The vectors East and North
form a basis for the horizontal plane. The From vector is projected
into the horizontal plane and the angle between the projected vector
and horizontal north is returned.
*/
float LSM303_heading_calc(vector_t from)
{
    vector_t temp_m = {m.x, m.y, m.z};

    // subtract offset (average of min and max) from magnetometer readings
    temp_m.x -= (m_min.x + m_max.x) / 2;
    temp_m.y -= (m_min.y + m_max.y) / 2;
    temp_m.z -= (m_min.z + m_max.z) / 2;

    // compute E and N
    vector_t E;
    vector_t N;
    vector_cross(&temp_m, &a, &E);
    vector_normalize(&E);
    vector_cross(&a, &E, &N);
    vector_normalize(&N);

    // compute heading
    float heading = atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / PI;
    if (heading < 0) heading += 360;
    return heading;
}


/*
Returns the angular difference in the horizontal plane between a
default vector and north, in degrees.

The default vector here is chosen to point along the surface of the
PCB, in the direction of the top of the text on the silkscreen.
This is the +X axis on the Pololu LSM303D carrier and the -Y axis on
the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH carriers.
*/
float LSM303_heading(void)
{
    return LSM303_heading_calc((vector_t){1, 0, 0});
}










