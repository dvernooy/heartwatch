/*************************************************** 
 This is a library written for the Maxim MAX30105 Optical Smoke Detector
 It should also work with the MAX30102. However, the MAX30102 does not have a Green LED.

 These sensors use I2C to communicate, as well as a single (optional)
 interrupt line that is not currently supported in this driver.
 
 Written by Peter Jansen and Nathan Seidle (SparkFun)
 BSD license, all text above must be included in any redistribution.
 *****************************************************/
#include <stdint.h>

#define MAX30105_ADDRESS          0x57 //7-bit I2C Address
#define _MAX30105_W_ADDRESS      ((MAX30105_ADDRESS <<1) & 0xFE)    
#define _MAX30105_R_ADDRESS      ((MAX30105_ADDRESS <<1) | 0x01)    
//Note that MAX30102 has the same I2C address and Part ID

//I2C_BUFFER_LENGTH is defined in Wire.H
//#define I2C_BUFFER_LENGTH BUFFER_LENGTH

//The catch-all default is 32
#define I2C_BUFFER_LENGTH 32

  uint8_t MAX30105_init(void);

  uint32_t MAX30105_getRed(void); //Returns immediate red value
  uint32_t MAX30105_getIR(void); //Returns immediate IR value
  uint32_t MAX30105_getRedIR(int32_t *data); //Returns immediate red and IR values

//  uint32_t MAX30105_getGreen(void); //Returns immediate green value
  uint8_t MAX30105_safeCheck(uint8_t maxTimeToCheck); //Given a max amount of time, check for new data

  // Configuration
  void MAX30105_softReset(void);
  void MAX30105_shutDown(void); 
  void MAX30105_wakeUp(void); 

  void MAX30105_setLEDMode(uint8_t mode);

  void MAX30105_setADCRange(uint8_t adcRange);
  void MAX30105_setSampleRate(uint8_t sampleRate);
  void MAX30105_setPulseWidth(uint8_t pulseWidth);

  void MAX30105_setPulseAmplitudeRed(uint8_t value);
  void MAX30105_setPulseAmplitudeIR(uint8_t value);
// void MAX30105_setPulseAmplitudeGreen(uint8_t value);
  void MAX30105_setPulseAmplitudeProximity(uint8_t value);

  void MAX30105_setProximityThreshold(uint8_t threshMSB);

  //Multi-led configuration mode (page 22)
  void MAX30105_enableSlot(uint8_t slotNumber, uint8_t device); //Given slot number, assign a device to slot
  void MAX30105_disableSlots(void);
  
  // Data Collection

  //Interrupts (page 13, 14)
  uint8_t MAX30105_getINT1(void); //Returns the main interrupt group
  uint8_t MAX30105_getINT2(void); //Returns the temp ready interrupt
  void MAX30105_enableAFULL(void); //Enable/disable individual interrupts
  void MAX30105_disableAFULL(void);
  void MAX30105_enableDATARDY(void);
  void MAX30105_disableDATARDY(void);
  void MAX30105_enableALCOVF(void);
  void MAX30105_disableALCOVF(void);
  void MAX30105_enablePROXINT(void);
  void MAX30105_disablePROXINT(void);
  void MAX30105_enableDIETEMPRDY(void);
  void MAX30105_disableDIETEMPRDY(void);

  //FIFO Configuration (page 18)
  void MAX30105_setFIFOAverage(uint8_t samples);
  void MAX30105_enableFIFORollover(void);
  void MAX30105_disableFIFORollover(void);
  void MAX30105_setFIFOAlmostFull(uint8_t samples);
  
  //FIFO Reading
  uint16_t MAX30105_check(void); //Checks for new data and fills FIFO
  uint8_t MAX30105_available(void); //Tells caller how many new samples are available (head - tail)
  void MAX30105_nextSample(void); //Advances the tail of the sense array
  uint32_t MAX30105_getFIFORed(void); //Returns the FIFO sample pointed to by tail
  uint32_t MAX30105_getFIFOIR(void); //Returns the FIFO sample pointed to by tail
  //uint32_t MAX30105_getFIFOGreen(void); //Returns the FIFO sample pointed to by tail

  uint8_t MAX30105_getWritePointer(void);
  uint8_t MAX30105_getReadPointer(void);
  void MAX30105_clearFIFO(void); //Sets the read/write pointers to zero

  //Proximity Mode Interrupt Threshold
  void MAX30105_setPROXINTTHRESH(uint8_t val);

  // Die Temperature
  float MAX30105_readTemperature(void);
  float MAX30105_readTemperatureF(void);

  // Detecting ID/Revision
  uint8_t MAX30105_getRevisionID(void);
  uint8_t MAX30105_readPartID(void);  

  // Setup the IC with user selectable settings
  void MAX30105_setup(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange);

  // Low-level I2C communication
  uint8_t MAX30105_read8(uint8_t reg);
  void MAX30105_write8(uint8_t reg, uint8_t value);

  uint8_t MAX30105_i2caddr;

  //activeLEDs is the number of channels turned on, and can be 1 to 3. 2 is common for Red+IR.
  uint8_t MAX30105_activeLEDs; //Gets set during setup. Allows check() to calculate how many bytes to read from FIFO
  
  uint8_t MAX30105_revisionID; 

  void MAX30105_readRevisionID(void);

  void MAX30105_bitMask(uint8_t reg, uint8_t mask, uint8_t thing);

