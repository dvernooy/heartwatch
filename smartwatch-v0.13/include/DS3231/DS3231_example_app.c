include <16F877A.h> 


 #device *= 16 

                                                                       
 #fuses NOWDT, HS, PROTECT, CPD, NOWRT, BROWNOUT, NODEBUG, NOLVP, PUT 
                     
                         
 #use delay (clock = 10MHz)            
 #use I2C(Master, SDA = pin_C4, SCL = pin_C3) 
                                   
                                 
 #define rowA      pin_B6                  
 #define rowB      pin_B5 
 #define rowC      pin_B4 
 #define rowD      pin_B3 

 #define col1      !input(pin_B2)    
 #define col2      !input(pin_B1)    
 #define col3      !input(pin_B0) 

 #define dly_1     2000 
 #define dly_2      300 
                             

 #include "lcd.c"              
 #include "DS3231.c"    
                                 
                             
 unsigned char s = 10;                    
 unsigned char min = 10;              
 unsigned char hr = 10;    
 unsigned char dy = 1;    
 unsigned char dt = 31;              
 unsigned char mt = 12;                    
 unsigned char yr = 99; 
 short hr_format = _12_hour_format; 
 short am_pm = 1;                                                      
                                     
                                       
 void setup();    
 unsigned char getKbd();        
 unsigned char getParameter(unsigned char p , unsigned char max, unsigned char min, unsigned char x_pos, unsigned char y_pos); 
 unsigned char makeNumber(unsigned char _10thDigit, unsigned char _1stDigit);    
 void settings();  
 unsigned char setDay(unsigned char dayValue, unsigned char x_pos, unsigned char y_pos); 
 void showDay(unsigned char day_val, unsigned char x_pos, unsigned char y_pos);  
 void showParameters(); 
                                                                       
                                                                         
 void main()            
 {                                
          setup(); 
          setTime(hr, min, s, am_pm, hr_format);  
          setDate(dy, dt, mt, yr);                    
          while(TRUE)                                  
          {                                    
                   getTime(hr, min, s, am_pm, hr_format); 
                   getDate(dy, dt, mt, yr);    
                   if(getKbd() == 0x0A) 
                   {                
                            settings();  
                   } 
                   showParameters();              
          };                                
 }                                              

                             
 void setup()              
 {    
          disable_interrupts(global);      
          setup_comparator(NC_NC_NC_NC); 
          setup_ADC(ADC_off);                        
          setup_ADC_ports(no_analogs); 
          setup_timer_0(0 | 0| 0);  
          setup_timer_1(T1_disabled); 
          setup_timer_2(T2_disabled, 255, 1); 
          set_timer0(0); 
          set_timer1(0); 
          set_timer2(0); 
          setup_SPI(SPI_disabled | SPI_SS_disabled); 
          setup_PSP(PSP_disabled);    
          setup_CCP1(CCP_off); 
          setup_CCP2(CCP_off); 
          set_tris_B(0x0F);    
          port_B_pullups(TRUE); 
          DS3231_init(); 
          lcd_init();          
          lcd_putc("\f"); 
 }                              
                                 

 unsigned char getKbd() 
 { 
          output_low(rowA); 
          output_high(rowB); 
          output_high(rowC); 
          output_high(rowD); 
          if(col1)                          
          {  
                   return 0x01; 
          }    
          else if(col2) 
          {                    
                   return 0x02; 
          } 
          else if(col3) 
          {  
                   return 0x03; 
          }  

          output_low(rowB); 
          output_high(rowA); 
          output_high(rowC); 
          output_high(rowD); 
          if(col1) 
          {  
                   return 0x04; 
          }    
          else if(col2) 
          {  
                   return 0x05; 
          } 
          else if(col3) 
          {  
                   return 0x06; 
          } 

          output_low(rowC); 
          output_high(rowA); 
          output_high(rowB); 
          output_high(rowD); 
          if(col1) 
          {  
                   return 0x07; 
          }    
          else if(col2) 
          {  
                   return 0x08; 
          } 
          else if(col3) 
          {  
                   return 0x09; 
          }      

          output_low(rowD); 
          output_high(rowA); 
          output_high(rowB); 
          output_high(rowC); 
          if(col1)        
          {                
                   return 0x0A; 
          }    
          else if(col2) 
          {  
                   return 0x00; 
          }                                
          else if(col3)          
          {  
                   return 0x0B; 
          } 
          else 
          { 
                   return 0x10; 
          } 
 }    

                                   
 unsigned char getParameter(unsigned char p , unsigned char max, unsigned char min, unsigned char x_pos, unsigned char y_pos) 
 {                                    
          short char_pos = 1; 
          unsigned char n = 0;                    
          unsigned char n1 = 0;          
          unsigned char n2 = 0;    
          lcd_gotoxy(x_pos, y_pos); 
          printf(lcd_putc, "%02u", p);            
          for(;;)                      
          {                  
                   if(getKbd() != 0x10)              
                   { 
                            n = getKbd();        
                            if((n >= 0) && (n <= 9)) 
                            {  
                                     char_pos = ~char_pos;  
                            } 
                            delay_ms(dly_2); 
                   }              
                   if(!char_pos)    
                   {                                      
                            if((n >= 0) && (n <= 9)) 
                            {        
                                     n1 = n; 
                                     lcd_gotoxy(x_pos, y_pos);  
                                     printf(lcd_putc, "%u", n1); 
                            }  
                   } 
                   else    
                   { 
                            if((n >= 0) && (n <= 9)) 
                            {                
                                     n2 = n; 
                                     lcd_gotoxy((x_pos + 1), y_pos); 
                                     printf(lcd_putc, "%u", n2); 
                            }          
                   }                                              
                   if(getKbd() == 0x0B) 
                   {                                    
                            p = makeNumber(n1, n2); 
                            if((p <= max) && (p >= min)) 
                            {                                              
                                     return p; 
                            } 
                            else 
                            { 
                                     return 1; 
                            } 
                   } 
          }                  
 } 
                                               

 unsigned char makeNumber(unsigned char _10thDigit, unsigned char _1stDigit) 
 { 
          return ((_10thDigit * 10) + _1stDigit); 
 } 


 void settings() 
 { 
          lcd_putc("\f"); 
          lcd_gotoxy(8, 2);        
          lcd_putc("Setup");    
          lcd_gotoxy(6, 3);        
          lcd_putc("Parameters");    
          delay_ms(dly_1); 
          lcd_putc("\f"); 
          lcd_gotoxy(5, 1);      
          lcd_putc("Hour Format");  
          lcd_gotoxy(1, 2); 
          lcd_putc("0. 24 Hour Format"); 
          lcd_gotoxy(1, 3);                        
          lcd_putc("1. 12 Hour Format"); 
          lcd_gotoxy(1, 4);  
          lcd_putc("Selection: ");  
          while(getKbd() > 1) 
          {                          
                   delay_ms(40); 
                   hr_format = getKbd(); 
          } 
          lcd_gotoxy(12, 4); 
          printf(lcd_putc, "%d", hr_format); 
          delay_ms(dly_1); 
          lcd_putc("\f");    
          lcd_gotoxy(7, 1);      
          lcd_putc("Set Time");    
          switch(hr_format)    
          {                  
                   case 1:              
                   { 
                            lcd_gotoxy(4, 2);                  
                            lcd_putc("00:00:00  AM");              
                            hr = getParameter(hr, 12, 1, 4, 2);    
                            lcd_gotoxy(7, 2);                  
                            lcd_putc("00:00  AM");              
                            min = getParameter(min, 59, 0, 7, 2);  
                            lcd_gotoxy(10, 2);                  
                            lcd_putc("00  AM");              
                            s = getParameter(s, 59, 0, 10, 2); 
                            break; 
                   } 
                   default:                              
                   { 
                            lcd_gotoxy(7, 2);                  
                            lcd_putc("00:00:00"); 
                            hr = getParameter(hr, 23, 0, 7, 2);    
                            lcd_gotoxy(10, 2);                  
                            lcd_putc("00:00"); 
                            min = getParameter(min, 59, 0, 10, 2); 
                            lcd_gotoxy(13, 2);                                  
                            lcd_putc("00"); 
                            s = getParameter(s, 59, 0, 13, 2);  
                            break; 
                   } 
          } 
          if(hr_format) 
          { 
                   while(getKbd() > 1) 
                   {                          
                            delay_ms(40); 
                            am_pm = getKbd(); 
                   }                      
                   lcd_gotoxy(14, 2); 
                   switch(am_pm)          
                   { 
                            case 1:                  
                            { 
                                     lcd_putc("PM"); 
                                     break; 
                            }  
                            default: 
                            { 
                                     lcd_putc("AM"); 
                                     break; 
                            } 
                   }      
          }                                                      
          lcd_gotoxy(7, 3);      
          lcd_putc("Set Date"); 
          lcd_gotoxy(6, 4);                  
          lcd_putc("31/12/99 - SUN"); 
          dt = getParameter(dt, 31, 1, 6, 4);    
          lcd_gotoxy(9, 4);                  
          lcd_putc("12/99 -"); 
          mt = getParameter(mt, 12, 1, 9, 4); 
          lcd_gotoxy(12, 4);                                  
          lcd_putc("99 -");                                  
          yr = getParameter(yr, 99, 0, 12, 4);  
          lcd_gotoxy(17, 4);              
          lcd_putc("SUN");                  
          dy = setDay(dy, 17, 4); 
          delay_ms(dly_1); 
          setTime(hr, min, s, am_pm, hr_format);                  
          setDate(dy, dt, mt, yr); 
          delay_ms(100); 
          lcd_putc("\f");      
 } 


 unsigned char setDay(unsigned char dayValue, unsigned char x_pos, unsigned char y_pos) 
 {          
          unsigned char p = 0; 
          for(;;)                            
          {                                    
                   p = getKbd(); 
                   showDay(dayValue, x_pos, y_pos);  
                   if((p >= 1) && (p <= 7)) 
                   { 
                            dayValue = p; 
                   }                                    
                   if(p == 0x0A) 
                   { 
                            return dayValue; 
                   } 
          }          
 } 
                                             

 void showDay(unsigned char day_val, unsigned char x_pos, unsigned char y_pos) 
 {                                              
          lcd_gotoxy(x_pos, y_pos);  
          switch(day_val) 
          { 
                   case 1:                        
                   { 
                            lcd_putc("SUN "); 
                            break; 
                   } 
                   case 2: 
                   { 
                            lcd_putc("MON "); 
                            break; 
                   } 
                   case 3: 
                   { 
                            lcd_putc("TUE "); 
                            break; 
                   }    
                   case 4: 
                   { 
                            lcd_putc("WED "); 
                            break; 
                   } 
                   case 5: 
                   { 
                            lcd_putc("THR "); 
                            break; 
                   }  
                   case 6: 
                   { 
                            lcd_putc("FRI"); 
                            break; 
                   } 
                   case 7: 
                   { 
                            lcd_putc("SAT"); 
                            break; 
                   } 
          } 
 }    

                   
 void showParameters()                                  
 { 
          lcd_gotoxy(6, 1); 
          lcd_putc("DS3231 RTC");              
          lcd_gotoxy(1, 3);                                
          printf(lcd_putc, "Date: %02u/%02u/%02u ", dt, mt, yr); 
          showDay(dy, 16, 3); 
          lcd_gotoxy(1, 4);                                                        
          printf(lcd_putc, "Temp: %2.2g'C ", getTemp());          
          lcd_gotoxy(1, 2); 
          switch(hr_format) 
          {                                  
                   case 1: 
                   {  
                            switch(am_pm) 
                            { 
                                     case 1: 
                                     { 
                                              printf(lcd_putc, "Time: %02u:%02u:%02u PM ", hr, min, s);    
                                              break;    
                                     }                                                                        
                                     default: 
                                     {              
                                              printf(lcd_putc, "Time: %02u:%02u:%02u AM ", hr, min, s);    
                                              break;    
                                     } 
                            }      
                            break; 
                   }              
                   default: 
                   {          
                            printf(lcd_putc, "Time: %02u:%02u:%02u     ", hr, min, s);    
                            break;                                  
                   }    
          }    
          delay_ms(600);          
 }