#include "DS3231.h"
#include "i2cmaster.h"                                                     
 
 unsigned char bcd_to_decimal(unsigned char d)                
 {                                                                                          
    return ((d & 0x0F) + (((d & 0xF0) >> 4) * 10)); 
 }                                

 unsigned char decimal_to_bcd(unsigned char d) 
 { 
   return (((d / 10) << 4) & 0xF0) | ((d % 10) & 0x0F); 
 }                                                    

 unsigned char DS3231_read8(unsigned char address) 
 {                                      
    unsigned char value = 0;                                                       
    i2c_start(_DS3231_Write_addr);      
    i2c_write(address);                             
    i2c_rep_start(_DS3231_Read_addr);    
    value = i2c_readNak();                      
    i2c_stop();                  
    return value; 
 }                      




 void DS3231_write8(unsigned char address, unsigned char value)    
 {  
    i2c_start(_DS3231_Write_addr); 
    i2c_write(address); 
    i2c_write(value);    
    i2c_stop(); 
 }  


 void DS3231_init() 
 {                                    
    DS3231_write8(controlREG, 0x00); 
	//DS3231_write8(statusREG, 0x08);  
    DS3231_write8(statusREG, (DS3231_read8(statusREG) & 0xF7)); //disable 32khz output
 }                        


 void DS3231_getTime(unsigned char *p3, unsigned char *p2, unsigned char *p1, short *p0, short hour_format) 
 {                        
          unsigned char tmp = 0; 
          *p1 = DS3231_read8(secondREG); 
          *p1 = bcd_to_decimal(*p1); 
          *p2 = DS3231_read8(minuteREG); 
          *p2 = bcd_to_decimal(*p2); 
          switch(hour_format) 
          {                                              
                   case 1:          
                   {          
                            tmp = DS3231_read8(hourREG); 
                            tmp &= 0x20; 
                            *p0 = (short)(tmp >> 5);              
                            *p3 = (0x1F & DS3231_read8(hourREG)); 
                            *p3 = bcd_to_decimal(*p3);                            
                            break;      
                   }    
                   default: 
                   { 
                            *p3 = (0x3F & DS3231_read8(hourREG));            
                            *p3 = bcd_to_decimal(*p3);    
                            break;      
                   } 
          }  
 }                                  
   
 void DS3231_getDate(unsigned char *p4, unsigned char *p3, unsigned char *p2, unsigned char *p1) 
 { 
    *p1 = DS3231_read8(yearREG); 
    *p1 = bcd_to_decimal(*p1);                  
    *p2 = (0x1F & DS3231_read8(monthREG)); 
    *p2 = bcd_to_decimal(*p2);                                
    *p3 = (0x3F & DS3231_read8(dateREG)); 
    *p3 = bcd_to_decimal(*p3);    
    *p4 = (0x07 & DS3231_read8(dayREG));    
    *p4 = bcd_to_decimal(*p4);                    
 } 

                                                     
 void DS3231_setTime(unsigned char hSet, unsigned char mSet, unsigned char sSet, short am_pm_state, short hour_format)  
 {                                                                                                              
          unsigned char tmp = 0; 
          DS3231_write8(secondREG, (decimal_to_bcd(sSet))); 
          DS3231_write8(minuteREG, (decimal_to_bcd(mSet)));        
          switch(hour_format) 
          { 
                   case 1: 
                   {        
                            switch(am_pm_state) 
                            { 
                                     case 1: 
                                     {            
                                              tmp = 0x60; 
                                              break; 
                                     } 
                                     default: 
                                     {    
                                              tmp = 0x40; 
                                              break; 
                                     } 
                            }                            
                            DS3231_write8(hourREG, ((tmp | (0x1F & (decimal_to_bcd(hSet))))));                    
                            break; 
                   }                                              
                   
                   default: 
                   { 
                    DS3231_write8(hourREG, (0x3F & (decimal_to_bcd(hSet)))); 
                    break; 
                   }  
          }    
 }                                                  

 void DS3231_setDate(unsigned char daySet, unsigned char dateSet, unsigned char monthSet, unsigned char yearSet) 
 {          
    DS3231_write8(dayREG, (decimal_to_bcd(daySet)));            
    DS3231_write8(dateREG, (decimal_to_bcd(dateSet)));  
    DS3231_write8(monthREG, (decimal_to_bcd(monthSet))); 
    DS3231_write8(yearREG, (decimal_to_bcd(yearSet)));    
 } 

unsigned char DS3231_getStatus(void) 
 {          
    return (DS3231_read8(statusREG));    
 } 
 
 float DS3231_getTemp()                                                  
 { 
    register float t = 0.0; 
    unsigned char lowByte = 0; 
    signed char highByte = 0; 
    lowByte = DS3231_read8(tempLSBREG); 
    highByte = DS3231_read8(tempMSBREG);  
    lowByte >>= 6;                  
    lowByte &= 0x03;      
    t = ((float)lowByte);  
    t *= 0.25;      
    t += highByte;          
    return t; 
 }