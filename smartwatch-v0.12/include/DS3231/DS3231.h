 #define DS3231_Address              0x68            
 #define _DS3231_Read_addr            ((DS3231_Address << 1) | 0x01) 
 #define _DS3231_Write_addr           ((DS3231_Address << 1) & 0xFE) 
 
 #define secondREG                   0x00 
 #define minuteREG                   0x01 
 #define hourREG                     0x02 
 #define dayREG                      0x03 
 #define dateREG                     0x04                              
 #define monthREG                    0x05                            
 #define yearREG                     0x06                    
 #define alarm1secREG                0x07        
 #define alarm1minREG                0x08 
 #define alarm1hrREG                 0x09            
 #define alarm1dateREG               0x0A  
 #define alarm2minREG                0x0B    
 #define alarm2hrREG                 0x0C 
 #define alarm2dateREG               0x0D 
 #define controlREG                  0x0E 
 #define statusREG                   0x0F 
 #define ageoffsetREG                0x10 
 #define tempMSBREG                  0x11 
 #define tempLSBREG                  0x12  
 #define _24_hour_format             0 
 #define _12_hour_format             1 
 #define am                          0 
 #define pm                          1 

 unsigned char bcd_to_decimal(unsigned char d); 
 unsigned char decimal_to_bcd(unsigned char d);                      
 unsigned char DS3231_Read(unsigned char address); 
 void DS3231_Write(unsigned char address, unsigned char value); 
 void DS3231_init(void);  
 void DS3231_getTime(unsigned char *p3, unsigned char *p2, unsigned char *p1, short *p0, short hour_format);  
 void DS3231_getDate(unsigned char *p4, unsigned char *p3, unsigned char *p2, unsigned char *p1);    
 void DS3231_setTime(unsigned char hSet, unsigned char mSet, unsigned char sSet, short am_pm_state, short hour_format); 
 void DS3231_setDate(unsigned char daySet, unsigned char dateSet, unsigned char monthSet, unsigned char yearSet);    
 float DS3231_getTemp(void);
 unsigned char DS3231_getStatus(void);