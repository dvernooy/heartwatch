/* 
    ChibiOS - Copyright (C) 2006-2014 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
	
4-13-18 to do list:
1. [DONE] Add a timer thread ... also changed timing to a divider of 1024
2. [DONE] Speed up scrolling
3. [DONE] Fix button handler, make it more robust (erg?)
4. [DONE] Add a picture show thread + fatFS ... note, disabled RTC & picture-showing thread needs biggish stack
5. [DONE] Add an I2C data gather thread
6. [DONE] Add a data show thread
7. [DONE] speed up SPI bus ... its all due to oversample parameter in MPL3115_A2.c, reduce from 128 & for major
           speed use tweaked code.
8. [DONE] revisit a fixed clock vs. free-running ... fixed clock working
	a. get rid of clock in mmbb.c because it was using the same one
	b. needed to tweak test_001.c to add delay in first test
	c. fixed clock using 8 bit counter working ok
9. [DONE] Figure out chibi/nil speed ... I think this is basically done. Freerunning clock uses timer/counter1
          Periodic clock uses timer/counter 0.
10. [DONE] fix stack smashing in timing thread back out ... give up lcd within the thread, instead of button 
           handler ... &add a very small delay to all exit of thread.
11.[DONE] saw rogue text below menu ... think fix 13 did the trick
12. [DONE] re-allocate stack space among threads
13. [DONE] Test sensormania with periodic clock ... scrolling real time
14. [DONE] Test I2C bus thread-safeness				   
15. [DONE] timer runs slow when lots of threads ... basically fixed with periodic clock
16. [DONE] Compile HR code
17. [DONE] Test HR code
18. [DONE] Fix SPI bus issues: SD card hangs after write new code & sometimes in process. Add 47K to VCC on SD CS Pin
19. [DONE] View pictures does not blank screen ... added screen blank & eliminated re-draw loop
20. [DONE] Add a battery voltage measurement
21. [DONE] Tweaks throughout
22. [DONE] show time
23. [DONE] Add protection skin - bottom black, top colored
24. [DONE] Better voltage display
25. [DONE] show sensors
26. [DONE] Add an SD write thread ... tentatively working ... had to disable simultaneous log to UART
27. [DONE] log file ... tentatively working ... don't read on windows directly from SD ... screws it up
28. [DONE] Add an RTC clock setup thread ... done for clock, add date/other later
29. [DONE] tweaked HR algo to eliminate zero crossing and just find five-in-a-row going up
30. [DONE] optimize hr algo ... table of good parameters
	sd	uart	avg	thdsleep
	3--	---		---	--------
	on	on		16	5
	off	off		16	10
	off	old		16	10
31. [DONE] Add big fonts ... DONE ... use multipliers in current fonts ... fancy fonts later	
32. [DONE] thread 5 can't call a subroutine inside sub-thread 3 to show LCD parameters ... why?
          allocate more memory to thread5? already did 768 ... fprintf_P NEEDS MORE > 1kB!!!!!
33. [DONE] Fix blurbs when select hello1/2 thread ... print to usart the value of cur_menu 
	       and menu_item and use that to figure out screen refreshing logic in Showmenu .. this 
		   bug was fixed awhile ago by paying attention to the thread messaging & signaling
34. [DONE] test losing time on DS3231 ... not sure how/why ... first disable oscillator - done. next synchronize
         wait and measure ... synch done ... after two hours, within 1 second
35. [DONE] more hrm algo tweaks
	a. [DONE] track down time issue .. ST2MS was screwed up ... fixed now by adding to nil.h instead of time.h
	b. [DONE] implement 5 point lpf w. circular buffer on final HRM
	c. [DONE] try 4-in-a-row 3 count & 1000000 limit
	d. [DONE] updated GUI
	e. [DONE] tweaked lower limit to 65 in algo

_____________
v0.7
1.[DONE] change heartbeat state to a structure

_____________
v0.8
1.[DONE] get rid of doubles in algo by adding a locking feature vs. time delay between beats

_____________
v0.9
1.[DONE] add SPO2 - tweaked font for heart, fixed && vs. & in if statements, added structures to hold
         state of algorithms 

_____________
v0.10
1.[DONE] faster sample rate? 8 avg instead of 15
	a. get xprintf_P off web & compile
	b. test writing to variable with it
	c. implement it in HB w/ 8 avgs
	d. write to card & to serial ... card cannot be written, f_sync takes up 20 ms-ish
	e. can get 20 ms without SD card write
	f. look at data, tweak algo(s) .. done

_____________
v0.11
1.[DONE] tweak heartrate.c
	a. -1 the signal
	b. 6 in a row
	c. enforce peak > 0
2.[DONE] back to standard algo 	

_____________
v0.12
1.[DONE] better filter & average
	
_____________
v0.13
1.[DONE] faster write to SD card ... 512 byte buffer in code & remove 100 us delays in write chain
	a. implemented mmc_spi from latest SD fat repo
	b. implemented 32 bytes per line of data, so 16 lines is 512 bytes
	c. aggregate & write a single 512 byte block *ONLY* each time to SD card, w/ fsync each time
	d. This technique allows 20 ms sampling intervals with 8-averages even when writing to SD
	e. average times are 18-20 ms, sD write times are 24 ms
	
	
0. [WAIT] reduced power consumption push
1. [WAIT] Fix timeouts in test 
2. [WAIT] refactor main ... pull out unused code, buffer sizes & thread memory & more optimized menu
          remove extra buffer size of 10 longs in heartrate filter = 40 bytes ... use binary math tricks
		  remove xprintf, xputs ... all in favor of usart_out, add in #IFNDEF DEBUG
3. [WAIT] EEPROM use for major settings & a global settings screen (plus possibly font table?)
4. [WAIT] faster LCD refresh
5. [WAIT] update the switch thread to accommodate other 2 ... include a panic	
6. [WAIT] Add a timer & alarms
7. [WAIT] Add physical protection to HRM
8. [WAIT] enable inverted text in LCD
9. [WAIT] option for Fahrenheit
10.[WAIT] step counting algorithm
11.[WAIT] remove all the other printfs, add a way to do doubles with xprintf
12.[WAIT] data log the heartbeat
13.[WAIT] compass app
*/

#include "hal.h"
#include "nil.h"
#include <avr/pgmspace.h>
#include <stdio.h>
//#include "datatypes.h"
#include <string.h>
#include <avr/io.h>

#include "spi_native.h"
#include "st7735.h"
#include "st7735_gfx.h"
#include "st7735_font.h"

#include "xprintf.h"
#include "ff.h"
#include "diskio.h"
#include "usart.h"

#include "ch_test.h"
#include "avr_heap.h"
#include "time.h"

#include "MPL3115A2.h"
#include "vector.h"
#include "L3GD20.h"
#include "DS3231.h"
#include "LSM303.h"
#include "MAX30105.h"
#include "heartRate.h"

#include "i2cmaster.h"
#include <math.h>

/********************************************************************************
PIN definitions variables
********************************************************************************/
 #define output_low(port,pin) port &= ~(1<<pin)
 #define output_high(port,pin) port |= (1<<pin)
 #define set_input(portdir,pin) portdir &= ~(1<<pin)
 #define set_output(portdir,pin) portdir |= (1<<pin) 

/********************************************************************************
LCD_PWM_setup
********************************************************************************/
volatile uint8_t LCD_PWM_SET = 10;

/********************************************************************************
FatFS setup
********************************************************************************/
# define BUFFPIXEL 40
DWORD acc_size;				/* Work register for fs command */
WORD acc_files, acc_dirs;
FILINFO Finfo;

char Line[128];				/* Console input buffer */

FATFS Fatfs[_VOLUMES];		/* File system object for each logical drive */
BYTE Buff[1024];			/* Working buffer */
uint8_t j;

volatile WORD Timer;		/* 100Hz increment timer */

/********************************************************************************
Global Variables
********************************************************************************/
static FILE st7735_out = FDEV_SETUP_STREAM(st7735_printf, NULL, _FDEV_SETUP_WRITE);
static FILE usart_out = FDEV_SETUP_STREAM(usart_putchar_printf, NULL, _FDEV_SETUP_WRITE);

volatile uint8_t triggered = 0;
volatile uint8_t lcd_context = 1; //which thread has LCD_context
volatile uint8_t lcd_context_count[14] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1}; //start screen within thread
volatile uint8_t lcd_context_count_MAX[14] = {1,1,1,1,3,1,1,1,1,1,1,2,1,4}; //max screens within thread
volatile uint8_t thread_internal_started[9] = {0,0,0,0,0,0,0,0,0}; //flag for whether thread has started
static thread_t *tp[14] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
static thread_reference_t trp[14] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};

uint8_t USART_HB_LOGGING_ON = 1;
uint8_t SD_HB_LOGGING_ON = 1;

/* Declare a semaphore */
semaphore_t lcdUSE;
semaphore_t sdUSE;
semaphore_t spiUSE;
semaphore_t i2cUSE;
semaphore_t usartUSE;
semaphore_t ButtonPressed;
semaphore_t testTHREAD;


// Global menu data
#define maxMenus 7
#define maxItems 7
#define pageSize 7 // available lines minus title line

#define DEBOUNCE_TIME 350        /* time to wait while "de-bouncing" button *///was 300
#define THREAD_EXIT 30        /* time to allow sub-threads to exit *///was 10
/********************************************************************************
Global Variables
********************************************************************************/ 
int curMenu = 0; // menu currently shown
int curItem = 0; // item currently marked (line 0 for menutitle, so starts on one). NB! Item marked may not correspond to line marked (see cursorCount)
int updateFlag = 0; // used to register when there has been an update to the menu (moved around) and wait a little before allowing user input
int cursorCount = 0;  // keeps track of white line of menu the cursor is on. Since there may be multiple pages, this may not be the same as the item marked in the menu
int menuCount = 0;  // keeps track of what line of menu to print
int pageScroll = 0; // for far have we scrolled down - pageSize items on each page.
int menuprintloop = 0; // keep track of numbers of printed items

volatile uint8_t switchtype = 0;

char menutitle[maxMenus][20];
char menuitem[maxMenus][maxItems][20];
int menulink[maxMenus][maxItems]; // a link to a submenu OR
int menuactn[maxMenus][maxItems]; // menu actions - turn on LED or whatever

/********************************************************************************
Stuff for System Time
********************************************************************************/ 
uint8_t elapsed_hours = 0;
uint8_t elapsed_mins = 0;
uint8_t elapsed_secs = 0;
double minute_counter;

/********************************************************************************
Stuff for Voltage
*******************************************************************************/ 

uint32_t battery_average = 0;
uint16_t WWW, XXX, YYY, ZZZ, AAA;

/********************************************************************************
Time info DS3231
********************************************************************************/
 unsigned char s = 0;                    
 unsigned char min = 37;              
 unsigned char hr = 8;    
 unsigned char dy = 3;    
 unsigned char dt = 24;              
 unsigned char mt = 4;                    
 unsigned char yr = 18; 
 short hr_format = _12_hour_format; 
 short am_pm = 0;
 const char *DAY_NAME[7] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
/********************************************************************************
Sensor info
********************************************************************************/
extern char altitude_string[];
extern char pressure_string[];


/********************************************************************************
Heartbeat setup
********************************************************************************/
int32_t Red_LED_output[4] = {0L};
int32_t IR_LED_output[4] = {0L};
int32_t Red_dc_avg = 0L;
int32_t IR_dc_avg = 0L;
uint8_t Red_result = 0;
uint8_t IR_result = 0;
int32_t LED_data_vector[2] = {0L};
uint32_t p1 = 0;


uint8_t HB_ON = 0;//HB beating display
uint8_t HB_locked = 0;
uint32_t ms_current_beat = 0UL;
uint32_t ms_old_beat = 0UL;
uint32_t ms_elapsed_since_last = 0UL;
int32_t ms_elapsed_since_last_right_now = 0UL;
uint32_t blanking_time = 0UL;   

struct HR_algo_state Red_HR_state;
struct HR_algo_state IR_HR_state;

/********************************************************************************
Time
********************************************************************************/
DWORD get_fattime (void)
{
/*
	RTC rtc;

	//Get local time 
	rtc_gettime(&rtc);

	/ Pack date and time into a DWORD variable
	return	  ((DWORD)(rtc.year - 1980) << 25)
			| ((DWORD)rtc.month << 21)
			| ((DWORD)rtc.mday << 16)
			| ((DWORD)rtc.hour << 11)
			| ((DWORD)rtc.min << 5)
			| ((DWORD)rtc.sec >> 1);
	*/
	return 0;
}

/********************************************************************************
Show RTC parameters
display = 0: LCD
display = 1: UART
********************************************************************************/
void DS3231_showParameters(uint8_t display)                                  
{ 
//char todays_date[3];
 
   if (display ==1) {
		fprintf_P(&usart_out, PSTR("Date: %02u/%02u/%02u \r\n"), mt, dt, yr); 
		//showDay(dy, 16, 3); 
		fprintf_P(&usart_out, PSTR("DS3231_Temp: %2.2g'C \r\n"), DS3231_getTemp());          
		switch(hr_format) {                                  
			case 1:   
				switch(am_pm) { 
					case 1: 
						fprintf_P(&usart_out, PSTR("Time: %02u:%02u:%02u PM \r\n"), hr, min, s);    
						break;                                                                            
					default:               
						fprintf_P(&usart_out, PSTR("Time: %02u:%02u:%02u AM \r\n"), hr, min, s);    
						break;    
				}  
				break;               
			default:           
				fprintf_P(&usart_out, PSTR("Time: %02u:%02u:%02u \r\n"), hr, min, s);    
				break;                                      
		}
	}
/*	
	if (display ==0) {
		fprintf_P(&st7735_out, PSTR("%s "), DAY_NAME[dy-1]);   
		fprintf_P(&st7735_out, PSTR("%02u/%02u/%02u\r\n"), dt, mt, yr);
		fprintf_P(&st7735_out, PSTR("\r\n"));
		//fprintf_P(&st7735_out, PSTR("Temp: %2.2g C\r\n"), DS3231_getTemp()); 
		if (hr_format==1) {                                  
			if (am_pm==1) {
				fprintf_P(&st7735_out, PSTR("%02u:%02u:%02u PM \r\n"), hr, min, s);    
			}
			else {
				fprintf_P(&st7735_out, PSTR("%02u:%02u:%02u AM \r\n"), hr, min, s);
			}
		}	
		else {
			fprintf_P(&st7735_out, PSTR("%02u:%02u:%02u \r\n"), hr, min, s); 
		}	
		}//hr_format
	}//if display					
*/
 }//showParameters



/********************************************************************************
Monitor
********************************************************************************/
/*********************************************************************************
 puts a line to the UART
********************************************************************************/ 
/*
static
void put_dump (const BYTE *buff, DWORD ofs, BYTE cnt)
{
	BYTE i;


	xprintf(PSTR("%08lX "), ofs);

	for(i = 0; i < cnt; i++)
		xprintf(PSTR(" %02X"), buff[i]);

	xputc(' ');
	for(i = 0; i < cnt; i++)
		xputc((buff[i] >= ' ' && buff[i] <= '~') ? buff[i] : '.');

	xputc('\n');
}
*/

/*********************************************************************************
 Gets a line from the UART
********************************************************************************/ 
/*
static
void get_line (char *buff, int len)
{
	BYTE c;
	int i = 0;


	for (;;) {
		c = rcvr();
		if (c == '\r') break;
		if ((c == '\b') && i) {
			i--;
			xmit(c);
			continue;
		}
		if (c >= ' ' && i < len - 1) {	// Visible chars
			buff[i++] = c;
			xputc(c);
		}
	}
	buff[i] = 0;
	xmit('\n');
}
*/

/*********************************************************************************
 Reads files in a directory
********************************************************************************/ 
#if (_FS_MINIMIZE == 0)
static
FRESULT scan_files (
	char* path		/* Pointer to the working buffer with start path */
)
{
	DIR dirs;
	FRESULT res;
	int i;
	char *fn;

	res = f_opendir(&dirs, path);
	if (res == FR_OK) {
		i = strlen(path);
		while (((res = f_readdir(&dirs, &Finfo)) == FR_OK) && Finfo.fname[0]) {
			if (_FS_RPATH && Finfo.fname[0] == '.') continue;

			fn = Finfo.fname;

			if (Finfo.fattrib & AM_DIR) {
				acc_dirs++;
				*(path+i) = '/'; strcpy(path+i+1, fn);
				res = scan_files(path);
				*(path+i) = '\0';
				if (res != FR_OK) break;
			} else {
//				xprintf(PSTR("%s/%s\n"), path, fn);
				acc_files++;
				acc_size += Finfo.fsize;
			}
		}
	}

	return res;
}
#endif

/*********************************************************************************
 Outputs result of a FATfs function to the screen
********************************************************************************/ 
static
void put_rc (FRESULT rc)
{
	const prog_char *p;
	static const prog_char str[] =
		"OK\0" "DISK_ERR\0" "INT_ERR\0" "NOT_READY\0" "NO_FILE\0" "NO_PATH\0"
		"INVALID_NAME\0" "DENIED\0" "EXIST\0" "INVALID_OBJECT\0" "WRITE_PROTECTED\0"
		"INVALID_DRIVE\0" "NOT_ENABLED\0" "NO_FILE_SYSTEM\0" "MKFS_ABORTED\0" "TIMEOUT\0"
		"LOCKED\0" "NOT_ENOUGH_CORE\0" "TOO_MANY_OPEN_FILES\0";
	FRESULT i;

	for (p = str, i = 0; i != rc && pgm_read_byte_near(p); i++) {
		while(pgm_read_byte_near(p++));
	}
	xprintf(PSTR("rc=%u FR_%S\n"), rc, p);
	st7735_setCursor(5,20);
    fprintf_P(&st7735_out,PSTR("%S"),p);
}

/*********************************************************************************
Timer for LCD PWM
********************************************************************************/ 

static
void LCD_PWM_Init (void)
{
	
	DDRB |= (1<<PB3);
	OCR2A = LCD_PWM_SET;
	TCCR2B |= (1<<CS22) |(1<<CS21)|(1<<CS20);
	TIMSK2 |= (1<<OCIE2A) |(1<<TOIE2) ;
}



/*********************************************************************************
 * Timer for LCD PWM
 *********************************************************************************/
CH_IRQ_HANDLER(TIMER2_COMPA_vect)
{           
CH_IRQ_PROLOGUE();
chSysLockFromISR();
      PORTB &= ~(1<<PB3);// clear output on timer overflow
chSysUnlockFromISR();
CH_IRQ_EPILOGUE();
}

/*********************************************************************************
 * Timer for LCD PWM
 *********************************************************************************/
CH_IRQ_HANDLER(TIMER2_OVF_vect)
{           
CH_IRQ_PROLOGUE();
chSysLockFromISR();
     PORTB |= (1<<PB3);			// set output on timer overflow
chSysUnlockFromISR();
CH_IRQ_EPILOGUE();
}


/*********************************************************************************
read 16 bits from a .bmp file - little endian
// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.
********************************************************************************/ 
uint16_t read16(FIL* f) {
  uint16_t result;
  uint8_t buffer;
  uint16_t br;
  
  f_read(f, &buffer, sizeof(buffer), &br);
  ((uint8_t *)&result)[0] = buffer; // LSB
  
  f_read(f, &buffer, sizeof(buffer), &br);
  ((uint8_t *)&result)[1] = buffer; // MSB
  return result;
}

/*********************************************************************************
read 32 bits from a bmp file
// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.
********************************************************************************/ 
uint32_t read32(FIL* f) {
  uint32_t result;
  uint8_t buffer;
  uint16_t br;
 
	f_read(f, &buffer, sizeof(buffer), &br);
    ((uint8_t *)&result)[0] = buffer; // LSB

	f_read(f, &buffer, sizeof(buffer), &br);
    ((uint8_t *)&result)[1] = buffer;

	f_read(f, &buffer, sizeof(buffer), &br);
    ((uint8_t *)&result)[2] = buffer;

	f_read(f, &buffer, sizeof(buffer), &br);
    ((uint8_t *)&result)[3] = buffer; // MSB
  return result;
}

/*********************************************************************************
Put a bmp on on the screen
********************************************************************************/ 
void bmpDraw(char *filename, uint8_t x, uint8_t y) {
	FIL      bmpFile;
	int      bmpWidth, bmpHeight;   // W+H in pixels
	uint8_t  bmpDepth;              // Bit depth (currently must be 24)
	uint32_t bmpImageoffset;        // Start of image data in file
	uint32_t rowSize;               // Not always = bmpWidth; may have padding
	uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
	uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
	uint8_t  goodBmp = 0;       // Set to true on valid header parse
	uint8_t  flip    = 1;        // BMP is stored bottom-to-top
	int      w, h, row, col;
	uint8_t  r, g, b;
	uint32_t pos = 0;
	uint16_t br;
	const uint8_t SIZE_OF_SDBUFFER = sizeof(sdbuffer);

	if((x >= st7735_width) || (y >= st7735_height)) {
		xputs(PSTR("bad position \r\n"));
		return;
	}
	xputs(PSTR("Loading image \r\n"));
	// Open requested file on SD card
	chSemWait(&spiUSE);
	if (f_open(&bmpFile, filename, FA_READ) != 0) {
		xputs(PSTR("File open error \r\n"));
		chSemSignal(&spiUSE);
		return;
	}
	chSemSignal(&spiUSE);
	// Parse BMP header
	
	chSemWait(&spiUSE);
	if (read16(&bmpFile) != 0x4D42) {							// BMP signature
		xputs(PSTR("invalid bmp \r\n"));
		chSemSignal(&spiUSE);
		return;
	}
    chSemSignal(&spiUSE);
	
	chSemWait(&spiUSE);
	xprintf(PSTR("File size: %ld \r\n"), read32(&bmpFile));	// File size
	chSemSignal(&spiUSE);
	
	chSemWait(&spiUSE);
	(void)read32(&bmpFile); 									// Read & ignore creator bytes
	chSemSignal(&spiUSE);
	
	chSemWait(&spiUSE);
	bmpImageoffset = read32(&bmpFile); 						// Start of image data
	chSemSignal(&spiUSE);
	
	xprintf(PSTR("Image Offset: %ld \r\n"), bmpImageoffset);

	chSemWait(&spiUSE);
	xprintf(PSTR("Header size: %ld \r\n"),read32(&bmpFile));	// Read DIB header
	chSemSignal(&spiUSE);
	
	chSemWait(&spiUSE);
	bmpWidth  = read32(&bmpFile);							// Width
	chSemSignal(&spiUSE);
	
	chSemWait(&spiUSE);
	bmpHeight = read32(&bmpFile);							// Height
	chSemSignal(&spiUSE);
	
	chSemWait(&spiUSE);
	if(read16(&bmpFile) != 1) {							// # planes -- must be '1'
		xputs(PSTR("planes not 1 \r\n"));
		chSemSignal(&spiUSE);
		return;
	}  
	chSemSignal(&spiUSE);
	
	chSemWait(&spiUSE);
	bmpDepth = read16(&bmpFile); 							// bits per pixel -- should be 24
	chSemSignal(&spiUSE);
	
	xprintf(PSTR("Bit Depth: %d \r\n"), bmpDepth);
	
	chSemWait(&spiUSE);
	if((bmpDepth != 24) && (read32(&bmpFile) != 0)) { 	// 0 = uncompressed
		xputs(PSTR("invalid depth or compression \r\n"));
		chSemSignal(&spiUSE);
		return;
	}
	chSemSignal(&spiUSE);
	
	goodBmp = 1; 										// Supported BMP format -- proceed!
	xprintf(PSTR("Image size: %d by %d pixels \r\n"), bmpWidth, bmpHeight);
	rowSize = (bmpWidth * 3 + 3) & ~3;					// BMP rows are padded (if needed) to 4-byte boundary
	// If bmpHeight is negative, image is in top-down order.
	// This is not canon but has been observed in the wild.
	if(bmpHeight < 0) {
		bmpHeight = -1*bmpHeight;
		flip      = 0;
	}
	// Crop area to be loaded
	w = bmpWidth;
	h = bmpHeight;
	if((x+w-1) >= st7735_width)  w = st7735_width  - x;
	if((y+h-1) >= st7735_height) h = st7735_height - y;
	
	// Set TFT address window to clipped image bounds
	chSemWait(&spiUSE);
	st7735_set_addr_win(x, y, x+w-1, y+h-1);
	chSemSignal(&spiUSE);
	
	for (row=0; row<h; row++) { 						// For each scanline...
	// Seek to start of scan line.  It might seem labor-
	// intensive to be doing this on every line, but this
	// method covers a lot of gritty details like cropping
	// and scanline padding.  Also, the seek only takes
	// place if the file position actually needs to change
	// (avoids a lot of cluster math in SD library).
    
		if(flip) {											// Bitmap is stored bottom-to-top order (normal BMP)
			pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
		}	
		else {										    	// Bitmap is stored top-to-bottom
			pos = bmpImageoffset + row * rowSize;
		}      
		
        chSemWait(&spiUSE);	
		f_lseek(&bmpFile, pos);
		chSemSignal(&spiUSE);
		
		buffidx = SIZE_OF_SDBUFFER; 					// Force buffer reload

		for (col=0; col<w; col++) { 					// For each pixel...
			// Time to read more pixel data?
			if (buffidx >= SIZE_OF_SDBUFFER) { 		// Indeed
				chSemWait(&spiUSE);
				f_read(&bmpFile, sdbuffer, SIZE_OF_SDBUFFER, &br);
				chSemSignal(&spiUSE);
				buffidx = 0; 							// Set index to beginning
			}			
			// Convert pixel from BMP to TFT format, push to display
			b = sdbuffer[buffidx++];
			g = sdbuffer[buffidx++];
			r = sdbuffer[buffidx++];
			chSemWait(&spiUSE);
			st7735_write_self_color(st7735_color(r,g,b)); //exposed from underlying api
			chSemSignal(&spiUSE);
		} // end pixel
	} // end scanline
	
	chSemWait(&spiUSE);
	f_close(&bmpFile);
	chSemSignal(&spiUSE);

} // end bmpDraw


/*********************************************************************************
time parser
********************************************************************************/ 
void parse_time(double time_in_min, uint8_t *hours, uint8_t *mins, uint8_t *secs) {
	double temp_min;
	double temp_secs;
	*hours = (uint8_t) (time_in_min/60);
	temp_min = time_in_min - (double) 60.0 * (*hours);
		*mins = (uint8_t) temp_min;
	temp_secs = 60.0*temp_min -  60.0 *(double) (*mins);
		*secs = (uint8_t) temp_secs;
}

/********************************************************************************
Stuff for Heap Info
********************************************************************************/ 

/*********************************************************************************
 * Print size of all all stacks.
 * @param[in] pr Print stream for output.
********************************************************************************/ 
void chPrintStackSizes(void) {
  const thread_config_t *tcp = nil_thd_configs;
  fprintf_P(&usart_out,PSTR(" Stack Sizes: "));	
  
  while (tcp->wend) {
    fprintf_P(&usart_out,PSTR("%3d "), tcp->wend - tcp->wbase);
    tcp++;
  }
  fprintf_P(&usart_out,PSTR("%d\r\n"), nilHeapIdleSize());
  
}

/*********************************************************************************
 * Print unused byte count for all stacks.
 * @param[in] pr Print stream for output.
********************************************************************************/ 
void chPrintUnusedStack(void) {
  const thread_config_t *tcp = nil_thd_configs;
  fprintf_P(&usart_out,PSTR("Unused Stack: "));	
  
  while (tcp->wend) {
    fprintf_P(&usart_out,PSTR("%3d "), fillSize((uint8_t*)tcp->wbase, (uint8_t*)tcp->wend));
    tcp++;
  }
  fprintf_P(&usart_out,PSTR("%d\r\n"), fillSize((uint8_t*)heapEnd(), (uint8_t*)RAMEND));
  
}


/*********************************************************************************
* paint a section of memory with 0x55
********************************************************************************/ 
static __attribute__((noinline)) void fill8(uint8_t* bgn, uint8_t* end) {
  while (bgn < end) *bgn++ = 0X55;
}

/*********************************************************************************
 * @brief   Returns the semaphore counter current value.
 * @param[in] sp        pointer to a @p Semaphore structure.
 * @return the value of the semaphore counter.
 * @api
********************************************************************************/ 
cnt_t nilSemGetCounter(semaphore_t *sp) {
  chSysLock();
  cnt_t cnt = sp->cnt;
  chSysUnlock();
  return cnt;
}

/*********************************************************************************
 * Determine unused stack for a thread.
 *
 * @param[in] nt Task index.
 *
 * @return Number of unused stack bytes.
********************************************************************************/ 
size_t nilUnusedStack(uint8_t nt) {
  const thread_config_t *tcp = &nil_thd_configs[nt];
  return fillSize((uint8_t*)tcp->wbase, (uint8_t*)tcp->wend);
}


/*********************************************************************************
 * Determine unused bytes in the heap and idle thread stack area.
 *
 * @return Number of unused bytes.
********************************************************************************/ 
size_t nilUnusedHeapIdle(void) {
  return fillSize((uint8_t*)heapEnd(), (uint8_t*)RAMEND);
}


/*********************************************************************************
* Fill stacks with 0X55
********************************************************************************/ 
static void nilFillStacks(void) {
  const thread_config_t *tcp = nil_thd_configs;
  
  while (tcp->wend) {
    fill8((uint8_t*)tcp->wbase, (uint8_t*)tcp->wend);
    tcp++;
  }
  // fill heap/idle stack
  fill8((uint8_t*)heapEnd(), (uint8_t*)&tcp - 16);
}

/*********************************************************************************
 * Start Nil RTOS with raw uninitialized stack memory.
 * This call saves a little flash compared to nilSysBegin().
 *
 * @return TRUE for success else FALSE.
********************************************************************************/ 
/*
bool nilSysBeginNoFill(void) {
  if (!nil_thd_count) return FALSE;
  chSysLock();
  boardInit();
  chSysInit();
  return TRUE;
}
*/

/*********************************************************************************
 * Start Nil RTOS with all stack memory initialized to a known value.
 *
 * @return TRUE for success else FALSE.
********************************************************************************/ 
/*
bool nilSysBegin(void) {
  nilFillStacks();
  return nilSysBeginNoFill();
}
*/

/*********************************************************************************
 * Delay a thread
********************************************************************************/ 
void nilThdDelay(systime_t time) {
  //systime_t t0 = port_timer_get_time();
  //while ((port_timer_get_time() - t0) < time) {}
  systime_t t0 = chVTGetSystemTimeX();
  while ((chVTGetSystemTimeX() - t0) < time) {}
}

/*********************************************************************************
 * Delay a Thread until a certain time
********************************************************************************/ 
void nilThdDelayUntil(systime_t time) {
  //nilThdDelay(time - port_timer_get_time());
  nilThdDelay(time - chVTGetSystemTimeX());
}

/********************************************************************************
Main Menu info
********************************************************************************/
void initMenu(void) {
    strcpy(menutitle[0], "MAIN MENU");
    strcpy(menuitem[0][0], "RTOS Tests");
    menulink[0][0] = 1; // link to menutitle[1];
    menuactn[0][0] = 0; // No action - just a sub menu
    strcpy(menuitem[0][1], "Information");
    menulink[0][1] = 2;
    menuactn[0][1] = 0;
    strcpy(menuitem[0][2], "SD Card Images");
    menulink[0][2] = 3;
    menuactn[0][2] = 0;
    strcpy(menuitem[0][3], "Sensor Data");
    menulink[0][3] = 4;
    menuactn[0][3] = 0;
    strcpy(menuitem[0][4], "Menu 2");
    menulink[0][4] = 5;
    menuactn[0][4] = 0;
 
    strcpy(menuitem[0][5], "\0"); // Need to initialize the end str of array
 
    strcpy(menutitle[1], "RTOS Tests");
    strcpy(menuitem[1][0], "NIL RTOS Test");
    menulink[1][0] = 0; 
    menuactn[1][0] = 1;
    strcpy(menuitem[1][1], "Thread Stack Data");
    menulink[1][1] = 0;
    menuactn[1][1] = 2;
    strcpy(menuitem[1][2], "Thread Status");
    menulink[1][2] = 0;
    menuactn[1][2] = 3;
	strcpy(menuitem[1][3], "//");
    menulink[1][3] = 0;
    menuactn[1][3] = 999;
    strcpy(menuitem[1][4], "\0"); // Need to initialize the end str of array
 
    strcpy(menutitle[2], "Information");
    strcpy(menuitem[2][0], "System Time");
    menulink[2][0] = 0; // no sub menu, just action
    menuactn[2][0] = 4;
    strcpy(menuitem[2][1], "HeartBeat");
    menulink[2][1] = 0;
    menuactn[2][1] = 11;
	strcpy(menuitem[2][2], "Info Scroll");
    menulink[2][2] = 0;
    menuactn[2][2] = 5;
	strcpy(menuitem[2][3], "Simple Timer");
    menulink[2][3] = 0;
    menuactn[2][3] = 6;
	strcpy(menuitem[2][4], "Set the Time");
    menulink[2][4] = 0;
    menuactn[2][4] = 12;
    strcpy(menuitem[2][5], "//");
    menulink[2][5] = 0; // code to trigger return to main menu.  Can't use 0 as that is considered no action here
    menuactn[2][5] = 999;
    strcpy(menuitem[2][6], "\0"); // Need to initialize the end str of array
 
    strcpy(menutitle[3], "SD Card Images");
    strcpy(menuitem[3][0], "Show image 1");
    menulink[3][0] = 0; // no sub menu, just action
    menuactn[3][0] = 7;
    strcpy(menuitem[3][1], "Show image 2");
    menulink[3][1] = 0;
    menuactn[3][1] = 8;
    strcpy(menuitem[3][2], "//");
    menulink[3][2] = 0; // code to trigger return to main menu.  Can't use 0 as that is considered no action here
    menuactn[3][2] = 999;
    strcpy(menuitem[3][3], "\0"); // Need to initialize the end str of array
 
    strcpy(menutitle[4], "Sensor Data");
    strcpy(menuitem[4][0], "Real Time Clock");
    menulink[4][0] = 0; // no sub menu, just action
    menuactn[4][0] = 9;
    strcpy(menuitem[4][1], "Sensormania");
    menulink[4][1] = 0;
    menuactn[4][1] = 10;
    strcpy(menuitem[4][2], "//");
    menulink[4][2] = 0; // code to trigger return to main menu.  Can't use 0 as that is considered no action here
    menuactn[4][2] = 999;
    strcpy(menuitem[4][3], "\0"); // Need to initialize the end str of array
 
    strcpy(menutitle[5], "Menu 2");
    strcpy(menuitem[5][0], "Menu 3");
    menulink[5][0] = 6; // go to menu 6
    menuactn[5][0] = 0;
    strcpy(menuitem[5][1], "//");
    menulink[5][1] = 0; // code to trigger return to main menu.  Can't use 0 as that is considered no action here
    menuactn[5][1] = 999;
    strcpy(menuitem[5][2], "\0"); // Need to initialize the end str of array
 
    strcpy(menutitle[6], "Menu 3");
    strcpy(menuitem[6][0], "//");
    menulink[6][0] = 5; // code to trigger return to main menu.  Can't use 0 as that is considered no action here
    menuactn[6][0] = 0;
    strcpy(menuitem[6][1], "\0"); // Need to initialize the end str of array
 
    // END of menu data
}

/*********************************************************************************
 * Menu Show routine
 *********************************************************************************/
void showMenu(void) {
	if (switchtype ==1) {//we are scrolling, no need to clear the screen	
	}
	else {
	chSemWait(&spiUSE);
	st7735_fill_rect(0, 0, 160, 128, ST7735_COLOR_BLACK);//lcd_clear();
	chSemSignal(&spiUSE);
	}
	chSemWait(&spiUSE);
    st7735_setCursor(5,10);//lcd_goto_xy(1,1);
    fprintf_P(&st7735_out, PSTR("%s"), menutitle[curMenu]);
	chSemSignal(&spiUSE);
    //print menu content
    while ((menuprintloop < (pageSize)) && (menuitem[curMenu][menuCount][0] != '\0')) {
        chSemWait(&spiUSE);
		st7735_setCursor(5, 10*(menuprintloop+2)); // +1 to leave first line for menu title
		chSemSignal(&spiUSE);
		
        if ((cursorCount + (pageSize*pageScroll)) == menuCount) {
            // item currently indicated by cursor
            chSemWait(&spiUSE);
			fprintf_P(&st7735_out, PSTR(">%s"), menuitem[curMenu][menuCount]);
			 chSemSignal(&spiUSE);
        } else {
            chSemWait(&spiUSE);
			fprintf_P(&st7735_out, PSTR(" %s"), menuitem[curMenu][menuCount]);
			chSemSignal(&spiUSE);
        }
        menuprintloop++;
        menuCount++;
    }
    menuprintloop = 0;
}

/*********************************************************************************
 * Button Handler
 *********************************************************************************/
CH_IRQ_HANDLER(PCINT0_vect)
{           

CH_IRQ_PROLOGUE();
chSysLockFromISR();
			triggered = 1;
			if (bit_is_clear(PINA, PA3)) {
				switchtype = 1; //button 3//
			}	
			else if (bit_is_clear(PINA, PA2)) {
					switchtype = 2;/*it is button 2 */	
			}
			else if (bit_is_clear(PINA, PA4)) {
					switchtype = 4;/* button is pressed */
			}	
		/* Wakes up a thread waiting on a thread reference object*/
		/* Resuming the thread with message, synchronous*/
		/* Synchronizes with thread that is suspended with &trp*/
		/* can also pass the message to that thread	*/
		chThdResumeI(&trp[0], (msg_t)switchtype); 	
chSysUnlockFromISR();
CH_IRQ_EPILOGUE();
}


/*********************************************************************************
 * Thread 1 - button interrupt handler
 *********************************************************************************/
THD_FUNCTION(Thread1, arg) {
  (void)arg;
  tp[0] = chThdGetSelfX();
  msg_t msg;

  PCMSK0 |= (1<<PCINT3)|(1<<PCINT2) |(1<<PCINT4) ;
  PCICR |= (1<<PCIE0);
  PORTA |= _BV(PA2);
  PORTA |= _BV(PA3);
  PORTA |= _BV(PA4);
  
  /* using a variable called lcd_context allows buttons to take */
  /* on different context depending on which screens			*/
  /* were alive when they were used 								*/
  /* this thread does the interpretation after the IRQ traps the button press */

  while (true) {
    chSysLock();
	/* Waiting for button push & IRQ to happen */
	/* Current thread put to sleep & sets up the reference variable trp for the trigger to reference */
	/* Will resume with a chThdResumeI*/
    msg = chThdSuspendTimeoutS(&trp[0], TIME_INFINITE);
    chSysUnlock(); 
	chThdSleepMilliseconds(DEBOUNCE_TIME); //wait and block the button from triggering again
    if (switchtype == 4) {//button4
		//next 3 should be atomic	
		if (LCD_PWM_SET ==50) {
			LCD_PWM_SET = 0;
		}
		else {
			LCD_PWM_SET += 10;
		}	
		chSysLock();	
		OCR2A = LCD_PWM_SET;
		PCIFR |= (1<<PCIF0);
		switchtype = 0;
		triggered = 0;
		chSysUnlock();
	}	
		
   if (triggered ==1){		
		if (lcd_context ==1){//coming from thread 1 
			//next 3 should be atomic	
			chSysLock();
			PCIFR |= (1<<PCIF0);
			triggered = 0;
			/*signal thread 2 ... the menu scrolling thread*/
			chSemSignal(&ButtonPressed);
			chSysUnlock();
		}
		if (lcd_context ==3){ //we were in thread3
			//next 3 should be atomic
			chSysLock();
			PCIFR |= (1<<PCIF0);//clear Pin change interrupt flag 0
			triggered = 0;//reset the trigger flag
			lcd_context =1;//going back to thread1
			
			/*returning from thread3 ... any button did it ... so we will go back to the main menu */
			/* reset the switchtype to 0*/
			switchtype = 0;
			
			/*Send an event signal to thread 3 to go back up to the main entry point*/
			/*Since the button was pressed to get out of that thread */
			/* Note - event masks should be thought of as or'd binary places. So you should use */
			/* 1, 2, 4, 8, 16 etc.. as distinct masks when signaling a specific thread*/
			/* the same mask # can be used when signaling other distinct threads, if you are */
			/* directly signaling that thread anyways */
			chEvtSignalI(tp[2], (eventmask_t)2);  //signal thread 3 that we are back out & its time to give up LCD
			
			chThdSleepMilliseconds(THREAD_EXIT); //give it some time to exit thread 3
			
			/*send semaphore to thread2 that the button is pressed */
			chSemSignal(&ButtonPressed); //need to signal thread1
			chSysUnlock();				
		}
		
		if (lcd_context ==4){ //we were in thread4
			//next 3 should be atomic
			chSysLock();
			PCIFR |= (1<<PCIF0);//clear Pin change interrupt flag 0
			triggered = 0;//reset the trigger flag
			lcd_context =1;//going back to thread1
			
			/*returning from thread4 ... any button did it ... so we will go back to the main menu */
			/* reset the switchtype to 0*/
			switchtype = 0;
			
			/*Send an event signal to thread 4 to go back up to the main entry point*/
			/*Since the button was pressed to get out of that thread */
			/* Note - event masks should be thought of as or'd binary places. So you should use */
			/* 1, 2, 4, 8, 16 etc.. as distinct masks when signaling a specific thread*/
			/* the same mask # can be used when signaling other distinct threads, if you are */
			/* directly signaling that thread anyways */
			chEvtSignalI(tp[3], (eventmask_t)2);  //signal thread 4 that we are back out
			
			chThdSleepMilliseconds(THREAD_EXIT); //give it some time to exit thread 4
			
			
			/*send semaphore to thread2 that the button is pressed */
			chSemSignal(&ButtonPressed); //need to signal thread2
			chSysUnlock();				
		}
		if (lcd_context ==5){ 
			//next 3 should be atomic
			chSysLock();
			PCIFR |= (1<<PCIF0);
			triggered = 0;
			
			
			if (switchtype == 1) { //returning from thread5 back to thread 1
				lcd_context =1; //returning to thread1
				
				/*returning from thread5 ... button 1 did it ... so we will go back to the main menu */
			    /* reset the switchtype to 0*/
				switchtype = 0;
				
				/*Send an event signal to thread 5 to go back up to the main entry point*/
				/*Since the button was pressed to get out of that thread */
				chEvtSignalI(tp[4], (eventmask_t)2);
				
				chThdSleepMilliseconds(THREAD_EXIT); //give it some time to exit thread 5
				
				/*send semaphore to thread2 that the button is pressed */
				chSemSignal(&ButtonPressed); //need to signal the thread 2
				chSysUnlock();		
             } 
             else { /*switchtype ==2, we are staying in thread5*/
				/*don't change lcd_context, we are staying in thread5*/
											
				/*button pressed to cycle through subscreens ... update the subscreen count*/
				lcd_context_count[4] = lcd_context_count[4]+1;
				if (lcd_context_count[4] > lcd_context_count_MAX[4]) {lcd_context_count[4] = 1;}
				
				/*staying in thread 5, signal thread5 to re-enter submenu with the new lcd context*/
				/* note, skip eventmask 3 because that is binary 11, which would signal 2 events*/
				chEvtSignalI(tp[4], (eventmask_t)4);  //stay in thread
				chSysUnlock();
            }/* if (switchtype == 1)*/		
        }/* if (lcd_context == 5)*/		

		if (lcd_context ==9){//returning from thread9 back to thread 1 
			//next 3 should be atomic
			chSysLock();
			PCIFR |= (1<<PCIF0);
			triggered = 0;
			
			lcd_context =1;//returning to thread1
			/*returning from thread9 ... button 1 did it ... so we will go back to the main menu */
			    /* reset the switchtype to 0*/
			switchtype = 0; //returning from thread9
			
			chEvtSignalI(tp[8], (eventmask_t)2);  //signal thread 9 that we are back out
			
			chThdSleepMilliseconds(THREAD_EXIT); //give it some time to exit thread 9

			chSemSignal(&ButtonPressed); //need to signal thread2
			chSysUnlock();				
		}/*if (lcd_context ==9)*/	
		
		if (lcd_context ==12){//returning from thread12 back to thread 1 
			//next 3 should be atomic
			chSysLock();
			PCIFR |= (1<<PCIF0);
			triggered = 0;
			if (switchtype == 1) { //returning from thread12 back to thread 1
				lcd_context =1;//returning to thread1
			
				//returning from thread 12 ... button 1 did it ... so we will go back to the main menu 
				// reset the switchtype to 0
				switchtype = 0; //returning from thread 12
			
				chEvtSignalI(tp[11], (eventmask_t)2);  //signal thread 12 that we are back out
			
				chThdSleepMilliseconds(THREAD_EXIT); //give it some time to exit thread 12

				chSemSignal(&ButtonPressed); //need to signal thread2
				chSysUnlock();
			}
			else {
				/*switchtype ==2, we are staying in thread 12*/
				/*don't change lcd_context, we are staying in thread12*/
				/*button pressed to cycle through subscreens ... update the subscreen count*/
				/*staying in thread 12, signal thread12 to re-enter submenu with the new lcd context*/
				lcd_context_count[11] = lcd_context_count[11]+1;
				if (lcd_context_count[11] > lcd_context_count_MAX[11]) {lcd_context_count[11] = 1;}
				chEvtSignalI(tp[11], (eventmask_t)4);  //stay in thread
				chSysUnlock();
			}	
		
		}//if (lcd_context ==12)	
		
		if (lcd_context ==14){ 
			//next 3 should be atomic
			chSysLock();
			PCIFR |= (1<<PCIF0);
			triggered = 0;
			
			if (switchtype == 1) { //either cycling or returning from thread14 back to thread 1
				
				/*button pressed to cycle through subscreens ... update the subscreen count*/
				lcd_context_count[13]++;
				if (lcd_context_count[13] > lcd_context_count_MAX[13]) {
					lcd_context_count[13] = 1;//reset it
					lcd_context =1; //returning to thread1
				
					/*returning from thread5 ... button 1 did it ... so we will go back to the main menu */
					/* reset the switchtype to 0*/
					switchtype = 0;
				
					/*Send an event signal to thread 14 to go back up to the main entry point*/
					/*Since the button was pressed to get out of that thread */
					chEvtSignalI(tp[13], (eventmask_t)8);
				
					chThdSleepMilliseconds(THREAD_EXIT); //give it some time to exit thread 14
				
					/*send semaphore to thread2 that the button is pressed */
					chSemSignal(&ButtonPressed); //need to signal the thread 2
					chSysUnlock();
				}
				else {
					/*Send an event signal to thread 14 to change to the next part of the time editing*/
					/*Since the button was pressed to get out of that thread */
					chEvtSignalI(tp[13], (eventmask_t)2);
					chSysUnlock();
				}	
             } 
             else { /*switchtype ==2, we are staying in thread14*/
				/*don't change lcd_context, we are staying in thread14*/
				
				/*staying in thread 14, signal thread5 to re-enter submenu with the new lcd context*/
				/* note, skip eventmask 3 because that is binary 11, which would signal 2 events*/
				chEvtSignalI(tp[13], (eventmask_t)4);  //stay in thread
				chSysUnlock();
            }/* if (switchtype == 1)*/		
        }/* if (lcd_context == 14)*/		
	}//if triggered ==1
  }//while true
}//THREAD1

/*********************************************************************************
 * Thread 2 - LCD Display Handler
 *********************************************************************************/
THD_FUNCTION(Thread2, arg) {

  (void)arg;
    tp[1] = chThdGetSelfX();
    initMenu(); // initialize menu by adding menu data to menu globals
	/*Wait for the lcdUSe semaphore to be free & take it */
	chSemWait(&lcdUSE);
	LCD_PWM_Init();
	
	chSemWait(&spiUSE);
	spi_init();
	st7735_init();//lcd_init();
	chSemSignal(&spiUSE);
	
	//lcd_contrast(0x36);//0x36
	chSemWait(&spiUSE);
	st7735_set_orientation(ST7735_LANDSCAPE_INV);
	st7735_fill_rect(0, 0, 160, 128, ST7735_COLOR_BLACK);
	chSemSignal(&spiUSE);
	
	chSemWait(&spiUSE);
	st7735_setTextColor(ST7735_COLOR_CYAN,ST7735_COLOR_BLACK);
	st7735_setTextSize(1);
	st7735_setTextWrap(1);
	chSemSignal(&spiUSE);

	/*give back lcd semaphore */
	chSemSignal(&lcdUSE);
	chThdSleepMilliseconds(100); 
 
    while(true) { 
	/*********************************************************************************
		Update LCD when triggered, then wait
	********************************************************************************/ 
	/*Wait for the lcdUSe semaphore to be free & take it again */
	chSemWait(&lcdUSE);
		
	/*show the menu*/
	
	showMenu();
		
	/*give back the lcd semaphore */
	chSemSignal(&lcdUSE);	

	/*********************************************************************************
			wait for button press semaphore to be free to do any more processing
	********************************************************************************/
	chSemWait(&ButtonPressed);
		
        /*********************************************************************************
			No action
		********************************************************************************/ 
		if (switchtype == 0) { ///we are returning from a sub-menu action
                if (menuitem[curMenu][curItem][0] == '\0') { 
				   curItem = 0; 
				   pageScroll = 0; 
				   cursorCount = 0;
				}
                if (cursorCount >= pageSize) {
                    // we have scrolled past this page, go to next
                    // remember, we check if we have scrolled off the MENU under clicks.  This is off the PAGE.
                    pageScroll++;  // next "page"
                    cursorCount=0; // reset cursor location
                }
            menuCount = pageScroll*pageSize;
        } //switchtype 0	
		
		/*********************************************************************************
			Scrolling
		********************************************************************************/ 
		if (switchtype == 1) { //we are scrolling
			curItem++; // add one to curr item
            cursorCount++;
                if (menuitem[curMenu][curItem][0] == '\0') { 
				   curItem = 0; 
				   pageScroll = 0; 
				   cursorCount = 0;
				}
                if (cursorCount >= pageSize) {
                    // we have scrolled past this page, go to next
                    // remember, we check if we have scrolled off the MENU under clicks.  This is off the PAGE.
                    pageScroll++;  // next "page"
                    cursorCount=0; // reset cursor location
                }
            menuCount = pageScroll*pageSize;
        } //switchtype 1				
		
		/*********************************************************************************
			Selecting and triggering
		********************************************************************************/ 
		if (switchtype == 2) { //we are selecting 
			// handle user input
			if (menuactn[curMenu][curItem]) {
				// has an action
				switch (menuactn[curMenu][curItem]) {
					case 1:	// action 1.1
						//chEvtSignalI(tp[5], (eventmask_t)1); //signal thread 6, Test thread
						chSysLock();
						chThdResumeI(&trp[5], (msg_t)0x55);
						chSysUnlock();
						break;
					case 2:	// action 1.2
						chEvtSignalI(tp[6], (eventmask_t)1); //signal thread 7, Stack usage
						break;
					case 3:	// action 1.2
						chEvtSignalI(tp[7], (eventmask_t)1); //signal thread 8, Thread status
						break;	
					case 4: // action 2.1 - Example 1
						/*signals first thread, which is sitting there waiting */
						/*with chEvtWaitAnyTimeout. Can also send flags */
						chEvtSignalI(tp[3], (eventmask_t)1);  // blank screen
						break;             
					case 5: // action 2.2 - Example 2
						chEvtSignalI(tp[4], (eventmask_t)1);  // blank screen
						break;
                    case 6: // action 2.3 - Example 2
						if (thread_internal_started[8] ==0)
						{						
							chEvtSignalI(tp[8], (eventmask_t)1);
							break;
						}
						else{
						chEvtSignalI(tp[8], (eventmask_t)1);  // Turn on LCD counter
						break;
						}
						break;
					case 7: //Show picture 1
						chSysLock();
						chThdResumeI(&trp[2], (msg_t)0x01);
						chSysUnlock();
						break;
					case 8: //show picture 2
						chSysLock();
						chThdResumeI(&trp[2], (msg_t)0x02);
						chSysUnlock();
						break;
					case 9: //show real time clock
						chSysLock();
						chThdResumeI(&trp[9], (msg_t)0x01);
						chSysUnlock();
						break;
					case 10: //show sensors
						chSysLock();
						chThdResumeI(&trp[10], (msg_t)0x01);
						chSysUnlock();
						break;
					case 11: //show heartbeat
						chEvtSignalI(tp[11], (eventmask_t)1); //signal thread 12, Thread status
						break;
					case 12: //set the Time
						chEvtSignalI(tp[13], (eventmask_t)1); //signal thread 14, Thread status
						break;	
					case 999:
						curMenu = 0; // return to main menu
						curItem = 0; // reset menu item to which cursor point
						pageScroll = 0; // reset menu page scroll
						cursorCount = 0; // reset menu location of page
						//menuCount = pageScroll*pageSize; // reprint from first line of this page
					    break;
				}//switch	
				menuCount = 0;
			} //menuaction
			else {
				curMenu = menulink[curMenu][curItem];  // set to menu selected by cursor
				curItem = 0; // reset menu item to which cursor point
				pageScroll = 0; // reset menu page scroll
				cursorCount = 0; // reset menu location of page
				menuCount = pageScroll*pageSize; // reprint from first line of this page
			}//end if menuaction
			updateFlag = 1; // we have updated the menu.  Flag is used to delay user input
		} // switchtype = 2
	
	} // end while (1)	
  
}//THREAD2


/*********************************************************************************
 * Thread 3 - visualization of bmp's, tp[2], trp[2], Menu actions 7 & 8
 *********************************************************************************/
THD_FUNCTION(Thread3, arg) {
	(void)arg;
	// initialize the SD card
	BYTE res1, res2;
	uint8_t pic_is_drawn = 0;

	tp[2] = chThdGetSelfX();
	msg_t msg;
	
	while (true) {
		/* Waiting for signal to show a picture - transfer via a message*/
		chSysLock();
		msg = chThdSuspendTimeoutS(&trp[2], TIME_INFINITE);
		chSysUnlock();
		chSemWait(&lcdUSE);
		/*switch lcd_context to thread 5*/
		lcd_context = 3;
		
		chSemWait(&spiUSE);
		st7735_fill_rect(0, 0, 160, 128, ST7735_COLOR_BLACK);//lcd_clear();
		st7735_setCursor(5,10);
		chSemSignal(&spiUSE);
		
		chSemWait(&sdUSE);
		chSemWait(&spiUSE);
		res1 = disk_initialize(0);
		chSemSignal(&spiUSE);
		chSemSignal(&sdUSE);
		
		//fprintf_P(&st7735_out,PSTR("Disk Init: %d (0 = OK)"), res1);
		chThdSleepMilliseconds(500);
		//st7735_setCursor(5,20);
		
		chSemWait(&sdUSE);
		chSemWait(&spiUSE);
		res2 = f_mount(0, &Fatfs[0]);
		chSemSignal(&spiUSE);
		chSemSignal(&sdUSE);
		
		//fprintf_P(&st7735_out,PSTR("f_mount: %d (0 = OK)"), res2);
		
		chThdSleepMilliseconds(500);
		
		while(true) {
            if(chEvtWaitAnyTimeout((eventmask_t)2, TIME_IMMEDIATE) == 2){
				pic_is_drawn = 0;
				chSemSignal(&lcdUSE); /*thread 3 gives back the LCD*/
				break;
			}		
			if ((!res1) && (!res2) && (!pic_is_drawn)) {	
				if ((uint8_t)msg == 1) {
					chSemWait(&sdUSE);
					bmpDraw("img00001.bmp",0,0);
					chSemSignal(&sdUSE);
				}
				if ((uint8_t)msg == 2) {
					chSemWait(&sdUSE);
					bmpDraw("img00002.bmp",0,0);
					chSemSignal(&sdUSE);
				}
				pic_is_drawn = 1;
			} 
			else if (!pic_is_drawn) {
					chSemWait(&spiUSE);
					fprintf_P(&st7735_out,PSTR("can't open file"));
					chSemSignal(&spiUSE);
			}	
			chThdSleepMilliseconds(300);
		}
	}
}//THREAD3

/*********************************************************************************
 * Thread 4 - Self-contained system time printout thread, triggered from menu
 *********************************************************************************/
THD_FUNCTION(Thread4, arg) {
	(void)arg;
	tp[3] = chThdGetSelfX();
	while (true) {
        /* wait for signal coming from main menu thread 2*/	
		chEvtWaitAnyTimeout((eventmask_t)1, TIME_INFINITE);
		/*take control of LCD semaphore*/
		chSemWait(&lcdUSE);
		/*switch lcd_context to thread 4*/
		lcd_context = 4; 
		
		chSemWait(&spiUSE);
		st7735_fill_rect(0, 0, 160, 128, ST7735_COLOR_BLACK);//lcd_clear();
		st7735_setCursor(5,10);//lcd_goto_xy(1,1);
		fprintf_P(&st7735_out, PSTR("System time:"));
		chSemSignal(&spiUSE);
		
		while(true) {
			/*now inside loop, wait for signal from thread1 button handler to get out*/
		    if(chEvtWaitAnyTimeout((eventmask_t)2, TIME_IMMEDIATE) == 2){
			chSemSignal(&lcdUSE);/*thread4 gives back the lcd */ 
			  break;
            }
			else {
			parse_time((double) minute_counter, &elapsed_hours, &elapsed_mins, &elapsed_secs);
			chSemWait(&spiUSE);
			st7735_setCursor(5,20);//lcd_goto_xy(1,2);
			fprintf_P(&st7735_out,PSTR("%02d:%02d \r\n"),elapsed_mins, elapsed_secs);
			chSemSignal(&spiUSE);
			chThdSleepMilliseconds(25);
    		/* could be doing other stuff here */
			}
		}
	}
}//THREAD4
	
/*********************************************************************************
 * Thread 5 - test thread with multiple options
 *********************************************************************************/
THD_FUNCTION(Thread5, arg) {
	(void)arg;
	//setup SD card
	double corrected_heading;
	
	uint8_t entry_flag;
	tp[4] = chThdGetSelfX();//returns a pointer to current thread
	while (true) {
		/* wait for signal coming from main menu thread 2*/	
		chEvtWaitAnyTimeout((eventmask_t)1, TIME_INFINITE);
		/*take control of LCD semaphore*/
		chSemWait(&lcdUSE);
		/*switch lcd_context to thread 5*/
		lcd_context = 5;
		//st7735_fill_rect(0, 0, 160, 128, ST7735_COLOR_BLACK);//lcd_clear();
		entry_flag = 1;
		while(true) {
		    /*now inside loop, wait for signal from thread1 button handler to get out*/
			if(chEvtWaitAnyTimeout((eventmask_t)2, TIME_IMMEDIATE) == 2){
				chSemSignal(&lcdUSE); /*thread5 gives back the lcd */
				break;
			}
			/*now inside loop, respond to signal from thread1 button handler to switch sub-menu*/
			if (chEvtWaitAnyTimeout((eventmask_t)4, TIME_IMMEDIATE) == 4) {
				entry_flag =1;//we have switched menu pages, clear the lcd
			}
			if(entry_flag ==1) {
			chSemWait(&spiUSE);
			st7735_fill_rect(0, 0, 160, 128, ST7735_COLOR_BLACK);//lcd_clear();
			st7735_setCursor(0,10);//lcd_goto_xy(1,1);
			chSemSignal(&spiUSE);
			entry_flag = 0; 
			}
			switch (lcd_context_count[4]) {
				case 1:	// action 5.2a					
					chSemWait(&i2cUSE);
					chSemWait(&spiUSE);
					fprintf_P(&st7735_out, PSTR("   Altitude: %-4.0fm\r\n"), MPL3115A2_getAltitude());// Display altitude value
					chSemSignal(&spiUSE);
					chSemSignal(&i2cUSE);
					
					chSemWait(&i2cUSE);
					chSemWait(&spiUSE);
					fprintf_P(&st7735_out, PSTR("   Pressure: %-7.0fPa\r\n"), MPL3115A2_getPressure());// Display altitude value
					chSemSignal(&spiUSE);
					chSemSignal(&i2cUSE);
					
					chSemWait(&i2cUSE);
					chSemWait(&spiUSE);
					fprintf_P(&st7735_out, PSTR("Temperature: %-4.1f C\r\n"), MPL3115A2_getTemperature());// Display altitude value
					chSemSignal(&spiUSE);
					chSemSignal(&i2cUSE);
					
					WWW = (uint16_t) battery_average;
					XXX = WWW/100;
					YYY = WWW - 100*XXX;
					ZZZ = YYY/10;
					AAA = YYY - 10*ZZZ;
					
					chSemWait(&spiUSE);st7735_setCursor(0,90);//lcd_goto_xy(1,1)
					fprintf_P(&st7735_out, PSTR("   DS3231 status: 0x%02X\r\n"),DS3231_getStatus());
					fprintf_P(&st7735_out, PSTR(" Battery Voltage: %u.%u%u\r\n"),XXX,ZZZ,AAA);
					fprintf_P(&st7735_out, PSTR("Firmware Version: 0.13\r\n")); 
					// screen2
					chSemSignal(&spiUSE);
					break;
				case 2:	// action 5.2b
					chSemWait(&i2cUSE);	
					L3GD20_read();
					chSemSignal(&i2cUSE);
					
					chSemWait(&spiUSE);
					fprintf_P(&st7735_out, PSTR("Rotation   x: %-7.0f\r\n"), g.x);// Display altitude value	
					fprintf_P(&st7735_out, PSTR("[mdeg/s]   y: %-7.0f\r\n"), g.y);// Display altitude value	
					fprintf_P(&st7735_out, PSTR("           z: %-7.0f\r\n"), g.z);// Display altitude value	
					fprintf_P(&st7735_out, PSTR("\r\n")); // screen2
					chSemSignal(&spiUSE);
					
					chSemWait(&i2cUSE);	
					LSM303_read();
					chSemSignal(&i2cUSE);
					
					chSemWait(&spiUSE);
					fprintf_P(&st7735_out, PSTR("Accel'n    x: %-4.1f\r\n"),a.x);
					fprintf_P(&st7735_out, PSTR("[g's]      y: %-4.1f\r\n"),a.y);
					fprintf_P(&st7735_out, PSTR("           z: %-4.1f\r\n"),a.z);
					fprintf_P(&st7735_out, PSTR("\r\n")); // screen2
					fprintf_P(&st7735_out, PSTR("Mag Field  x: %-6.1f\r\n"),m.x);
					fprintf_P(&st7735_out, PSTR("[mgauss]   y: %-6.1f\r\n"),m.y);
					fprintf_P(&st7735_out, PSTR("           z: %-6.1f\r\n"),m.z);
					fprintf_P(&st7735_out, PSTR("\r\n"));
					chSemSignal(&spiUSE);

					chSemWait(&i2cUSE);	
					corrected_heading = LSM303_heading();
					chSemSignal(&i2cUSE);	
					
					if (corrected_heading < 180.0) {
						corrected_heading += 180.0;
					}
					else {
						corrected_heading -= 180.0;
					}
					
					chSemWait(&spiUSE);
					fprintf_P(&st7735_out, PSTR("^ North %3.0f degrees East ^\r\n"),corrected_heading);
					chSemSignal(&spiUSE);
					
					break;
				case 3:// action 5.2c
					chSemWait(&i2cUSE);
					DS3231_getTime(&hr, &min, &s, &am_pm, hr_format); 
					chSemSignal(&i2cUSE);

					chSemWait(&i2cUSE);
					DS3231_getDate(&dy, &dt, &mt, &yr);
					chSemSignal(&i2cUSE);
					
					chSemWait(&spiUSE);
					st7735_setTextSize(2);
					fprintf_P(&st7735_out, PSTR("%s"), DAY_NAME[dy-1]);
					st7735_setCursor(64,10);//lcd_goto_xy(1,1);
					fprintf_P(&st7735_out, PSTR("%2u/%02u/%02u\r\n"), mt, dt, yr);
					chSemSignal(&spiUSE);
					
					if (hr_format==1) {                                  
						if (am_pm==1) {
							chSemWait(&spiUSE);
							st7735_setCursor(0,50);//lcd_goto_xy(1,1);
							st7735_setTextSize(3);
							fprintf_P(&st7735_out, PSTR("%2u:%02u:%02u\r\n"), hr, min, s);    
							st7735_setTextSize(1);
							st7735_setCursor(148,50);//lcd_goto_xy(1,1);
							fprintf_P(&st7735_out, PSTR("PM\r\n"));    
							st7735_setTextSize(3);
							chSemSignal(&spiUSE);
						}
						else {
							chSemWait(&spiUSE);
							st7735_setCursor(0,50);//lcd_goto_xy(1,1);
							st7735_setTextSize(3);
							fprintf_P(&st7735_out, PSTR("%2u:%02u:%02u\r\n"), hr, min, s);
							st7735_setTextSize(1);
							st7735_setCursor(148,50);//lcd_goto_xy(1,1);
							fprintf_P(&st7735_out, PSTR("AM\r\n"));    
							st7735_setTextSize(3);
							chSemSignal(&spiUSE);
						}
					}	
					else {
						chSemWait(&spiUSE);
						st7735_setCursor(0,50);//lcd_goto_xy(1,1);
						st7735_setTextSize(3);
						fprintf_P(&st7735_out, PSTR("%02u:%02u:%02u\r\n"), hr, min, s);
						st7735_setTextSize(1);
						chSemSignal(&spiUSE);
					}//hr_format				
					//DS3231_showParameters(0);
					chSemWait(&spiUSE);
					st7735_setCursor(0,120);//lcd_goto_xy(1,1);
					st7735_setTextSize(1);
					chSemWait(&i2cUSE);
					fprintf_P(&st7735_out, PSTR("Temp: %-3.1f C\r\n"), DS3231_getTemp()); 
					chSemSignal(&i2cUSE);
					chSemSignal(&spiUSE);
					break;
			}
			chSemWait(&spiUSE);
			st7735_setCursor(0,10);//lcd_goto_xy(1,1);
			chSemSignal(&spiUSE);
			chThdSleepMilliseconds(150);	
		}
	}
}//THREAD5

/*********************************************************************************
 * Thread 6 - Test Thread
 *
 * !!!!!!!!!!!!!!! need to use &trp[5] to signal this, not sure why !!!!!!!!!!!
 * !!!!!!!!!!!!!!! also, why do you need the other thread? !!!!!!!!!!!!!!!!!!!!!
 * >>>> I think that other signalling thread is the whole problem .. get rid of it
 * >>>> & see if it helps
 *
 *********************************************************************************/
THD_FUNCTION(Thread6, arg) {
  (void)arg;
  tp[5] = chThdGetSelfX();
  msg_t msg;
  /*
   * Activates the serial driver 1 using the driver default configuration.
   * PA9 and PA10 are routed to USART1.
   */
  sdStart(&SD1, NULL);
  usart_init(MYUBRR);
  /* Welcome message.*/
  chSemWait(&usartUSE);
  fprintf_P(&usart_out,PSTR("\r\n NILRTOS 1.1 \r\n"));
  chSemSignal(&usartUSE);
 
  while (true) {
		/* Waiting for button push and activation of the test suite.*/
		chSysLock();
		msg = chThdSuspendTimeoutS(&trp[5], TIME_INFINITE);
		chSysUnlock();
		
		//chEvtWaitAnyTimeout((eventmask_t)1, TIME_INFINITE);
		chSemWait(&usartUSE);	
		test_execute((BaseSequentialStream *)&SD1);
	    chSemSignal(&usartUSE);
  }		
}//THREAD6

/*********************************************************************************
 * Thread 7 - Print Stack Information - tp[6]
 *********************************************************************************/
THD_FUNCTION(Thread7, arg) {

  (void)arg;
  tp[6] = chThdGetSelfX();
  systime_t wakeTime;
  
  while (true) {
	chEvtWaitAnyTimeout((eventmask_t)1, TIME_INFINITE);
	wakeTime = chVTGetSystemTimeX();
	wakeTime += MS2ST(120);
    // Sleep until next second.
    chThdSleepUntil(wakeTime);
	chSemWait(&usartUSE); 
	// Print unused stack for thread 1, thread 2, and idle thread.
    chPrintStackSizes();
    // Print unused stack for thread 1, thread 2, and idle thread.
    chPrintUnusedStack();
	chSemSignal(&usartUSE);
	// Add ticks for one second.  The MS2ST macro converts ms to system ticks.
       }
}//THREAD7


/*********************************************************************************
 * Thread 8 - Print Thread Information
 *********************************************************************************/
THD_FUNCTION(Thread8, arg) {
  (void)arg;
  uint8_t j;
  tp[7] = chThdGetSelfX();

  /*
   * Activates the serial driver 1 using the driver default configuration.
   * PA9 and PA10 are routed to USART1.
   */
   
  while (true) {
    /* Waiting for button push to show thread status.*/	
	chEvtWaitAnyTimeout((eventmask_t)1, TIME_INFINITE);
    
	chSemWait(&usartUSE); 
	fprintf_P(&usart_out,PSTR("THREAD STATUS\r\n"));
	chSemSignal(&usartUSE);	
	
	for  (j = 0; j<NIL_CFG_NUM_THREADS; j++){
		chSemWait(&usartUSE);  
		fprintf_P(&usart_out,PSTR("%3u"), j);
		chSemSignal(&usartUSE);
	}
	
	chSemWait(&usartUSE);  
	fprintf_P(&usart_out,PSTR("\r\n"));
	chSemSignal(&usartUSE);	
	
	for  (j = 0; j<NIL_CFG_NUM_THREADS; j++){
		chSemWait(&usartUSE);  
		fprintf_P(&usart_out,PSTR(" --"), j);
		chSemSignal(&usartUSE);
	}
	
	chSemWait(&usartUSE);  
	fprintf_P(&usart_out,PSTR("\r\n"));
	chSemSignal(&usartUSE);	
 
	for  (j = 0; j<NIL_CFG_NUM_THREADS; j++){
		chSemWait(&usartUSE);  
		fprintf_P(&usart_out,PSTR("%3u"), tp[j]->state);
		//try this at some point
		//fprintf_P(&usart_out,PSTR("THREAD%u: %u \r\n"), j, &nil.threads[j]->state) 
		chSemSignal(&usartUSE);
	}
	chSemWait(&usartUSE);  
	fprintf_P(&usart_out,PSTR("\r\n"));
	chSemSignal(&usartUSE);	
        /*
		chSemWait(&lcdUSE);
		lcd_goto_xy(5,1);  
		fprintf_P(&st7735_out,PSTR("%2d"), isrSEM.cnt);
		chSemSignal(&lcdUSE);
		chSemWait(&lcdUSE);
		lcd_goto_xy(12,1);  
		fprintf_P(&st7735_out,PSTR("%2d"), lcdUSE.cnt);
		chSemSignal(&lcdUSE);
		*/
  }
}//THREAD8

/*********************************************************************************
 * Thread 9 - Show Semaphore Data
 *********************************************************************************/
THD_FUNCTION(Thread9, arg) {
	(void)arg;
	uint32_t count = 0;
	uint8_t on_off = 0;
	tp[8] = chThdGetSelfX();
	
	// initialize the direction of PORTC LEDs #6 to be an output
    set_output(DDRC, PC2);
    set_output(DDRC, PC3);
	output_high(PORTC, PC2);
	output_high(PORTC, PC3);
	
	while (true) {
		chEvtWaitAnyTimeout((eventmask_t)1, TIME_INFINITE);
		chSemWait(&lcdUSE);
		thread_internal_started[8]=1;
		lcd_context = 9;
		chSemWait(&spiUSE);
		st7735_fill_rect(0, 0, 160, 128, ST7735_COLOR_BLACK);//lcd_clear();
		chSemSignal(&spiUSE);
		
		while(true) {
            if(chEvtWaitAnyTimeout((eventmask_t)2, TIME_IMMEDIATE) == 2){
				output_high(PORTC, PC3); /*stop the LEDS*/
				chSemSignal(&lcdUSE);/*thread 9 releases LCD*/
				break;
				}
			if (lcd_context == 9){
				chSemWait(&spiUSE);
				st7735_setCursor(5,10);//lcd_goto_xy(1,1);
				fprintf_P(&st7735_out, PSTR("count:%u"), count); 
				st7735_setCursor(5,20);//lcd_goto_xy(1,2);
				fprintf_P(&st7735_out,PSTR("LCD SEM:%2d"), lcdUSE.cnt);
				chSemSignal(&spiUSE);
			}
			if (on_off ==1) {
				output_high(PORTC, PC3); 
				on_off = 0;
			} 
			else {
				output_low(PORTC, PC3); on_off = 1; 
				count++;
			}
			chThdSleepMilliseconds(500);
		}
	}
}//THREAD9


/*********************************************************************************
 * Thread 10 - visualization of RTC
 *********************************************************************************/
THD_FUNCTION(Thread10, arg) {
	(void)arg;
	tp[9] = chThdGetSelfX();
	msg_t msg;
	
	chSemWait(&i2cUSE);
	i2c_init();
	chSemSignal(&i2cUSE);
	
	chSemWait(&usartUSE);
	fprintf_P(&usart_out,PSTR("i2c init successful\r\n"));
	chSemSignal(&usartUSE);
	
	chSemWait(&i2cUSE);
	DS3231_init();
	chSemSignal(&i2cUSE);
	
	chSemWait(&usartUSE);
	fprintf_P(&usart_out,PSTR("DS3231 init successful\r\n"));
	chSemSignal(&usartUSE);
	
	/*
	chSemWait(&i2cUSE);
	DS3231_setTime(hr, min, s, am_pm, hr_format);  
	DS3231_setDate(dy, dt, mt, yr);
	chSemSignal(&i2cUSE);
	*/
	
	/* Waiting for signal to show data - transfer via a message*/
	chSysLock();
	msg = chThdSuspendTimeoutS(&trp[9], TIME_INFINITE);
	chSysUnlock();	
	while (true) {				
		
		chSemWait(&i2cUSE);
		DS3231_getTime(&hr, &min, &s, &am_pm, hr_format); 
		DS3231_getDate(&dy, &dt, &mt, &yr);
		chSemSignal(&i2cUSE);
		chSemWait(&usartUSE);
		fprintf_P(&usart_out,PSTR("\r\n")); 
		DS3231_showParameters(1);
		chSemSignal(&usartUSE);	
	
		chThdSleepMilliseconds(50);
		}
}//THREAD10

/*********************************************************************************
 * Thread 11 - visualization of sensors
 *********************************************************************************/
THD_FUNCTION(Thread11, arg) {
	(void)arg;
	tp[10] = chThdGetSelfX();
	msg_t msg;
	vector_t run_min = {0,0,0}, run_max = {0,0,0};

	chSemWait(&i2cUSE);
	i2c_init();
	MPL3115A2_init();               // Initialize barometer
	L3GD20_enableDefault();		
	LSM303_enableDefault();
	chSemSignal(&i2cUSE);
	/* Waiting for signal to show data - transfer via a message*/
	chSysLock();
	msg = chThdSuspendTimeoutS(&trp[10], TIME_INFINITE);
	chSysUnlock();		
	
	while (true) {
		
		chSemWait(&usartUSE);
		fprintf_P(&usart_out,PSTR("\r\n"));
		chSemWait(&i2cUSE);
		fprintf_P(&usart_out,PSTR("A: %-7.3fm P: %-7.3f Pa T: %-7.3f C \r\n"), MPL3115A2_getAltitude(), MPL3115A2_getPressure(), MPL3115A2_getTemperature());// Display altitude value
		chSemSignal(&i2cUSE);
		chSemSignal(&usartUSE);
        
		chSemWait(&i2cUSE);	
		L3GD20_read();
		chSemSignal(&i2cUSE);
		chSemWait(&usartUSE);
		fprintf_P(&usart_out,PSTR("x: %-7.3fm y: %-7.3f Pa z: %-7.3f\r\n"), g.x, g.y, g.z);// Display altitude value	
		chSemSignal(&usartUSE);
		
		
		chSemWait(&i2cUSE);	
		LSM303_read();
		chSemSignal(&i2cUSE);	
		chSemWait(&usartUSE);
		fprintf_P(&usart_out, PSTR("Acceleration g's x:%7.1f y:%7.1f z:%7.1f \r\n"),a.x,a.y,a.z);
		fprintf_P(&usart_out, PSTR("Magnetometer mgauss x:%7.1f y:%7.1f z:%7.1f \r\n"),m.x,m.y,m.z);
		chSemSignal(&usartUSE);
		run_min.x = fmin(run_min.x, m.x);
		run_min.y = fmin(run_min.y, m.y);
		run_min.z = fmin(run_min.z, m.z);

		run_max.x = fmax(run_max.x, m.x);
		run_max.y = fmax(run_max.y, m.y);
		run_max.z = fmax(run_max.z, m.z);
		chSemWait(&usartUSE);
		fprintf_P(&usart_out, PSTR("min: %7.1f %7.1f %7.1f max: %7.1f %7.1f %7.1f \r\n"),run_min.x, run_min.y, run_min.z,run_max.x, run_max.y, run_max.z);
        

		/* When given no arguments, the heading() function returns the angular	difference in the horizontal 
		plane between a default vector and north, in degrees.
  
		The default vector is chosen by the library to point along the surface of the PCB, in the direction 
		of the top of the text on the silkscreen. This is the +X axis on the Pololu LSM303D carrier and the 
		-Y axis on the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH carriers.
  
		To use a different vector as a reference, use the version of heading() that takes a vector argument; 
		for example, use compass.heading((LSM303::vector<int>){0, 0, 1}) to use the +Z axis as a reference.*/
	    
		chSemWait(&i2cUSE);	
		fprintf_P(&usart_out, PSTR("Heading: %7.1f \r\n"),LSM303_heading());
		chSemSignal(&i2cUSE);	
		chSemSignal(&usartUSE);
		
		chThdSleepMilliseconds(50);
	}
}//THREAD11

/*********************************************************************************
 * Thread 12 - HeartBeat 
 *********************************************************************************/
THD_FUNCTION(Thread12, arg) {
	(void)arg;
	tp[11] = chThdGetSelfX();
	msg_t msg;
	uint8_t res;
	uint8_t just_entered = 0;
	uint32_t time_now;
	systime_t HB_baseline;
	char data_string[33];
	char data_string_SD[513];
	uint8_t SD_count = 0;
	FIL fdst;
	uint16_t bw;
	int32_t current_HB_average = 85L;
	int32_t current_HB_estimate = 0L;
	int32_t current_HB_average_last = 0L;
	int32_t unlock_test = 0L;
	int32_t coeff = 5L;
	int32_t attack_max = 35L;
	
	uint8_t i;
	
	int32_t SPO2_circular_buffer[7] = {100L,100L,100L,100L,100L,100L,100L};
	int32_t SPO2_sum = 0L;
	uint8_t SPO2_circular_buffer_index = 0;
	uint8_t SPO2_index = 0;
	int32_t SPO2 = 0L;
	int32_t R1 = 0L;
	int32_t R2 = 0L;
	int32_t R = 0L;
	
	struct SPO2_state {
		int32_t baseline;
		int32_t spread;
		uint8_t status;
	};
	
	struct SPO2_state Red_SPO2_state;
	struct SPO2_state IR_SPO2_state;
	
	/* initialize the Red SPO2 state variable*/
	Red_SPO2_state.baseline = 0;
	Red_SPO2_state.spread = 0;
	Red_SPO2_state.status = 0;	
	
	/* initialize the IR SPO2 state variable*/
	IR_SPO2_state.baseline = 0;
	IR_SPO2_state.spread = 0;
	IR_SPO2_state.status = 0;	
	
	/* initialize the Red HR state variable*/
	Red_HR_state.maybe_peak = 0;
	Red_HR_state.maybe_valley = 0;
	Red_HR_state.peak_detect = 0;
	Red_HR_state.valley_detect = 0;
	Red_HR_state.fiar = 0; //stands for five-in-a-row
	Red_HR_state.valley_min = 0L;
	Red_HR_state.peak_max = 0L;
	Red_HR_state.offset_DC = 0;
	Red_HR_state.offset_filt = 0;
	for(i = 0; i < 32; i++){
		if (i < 4){
			Red_HR_state.xbuf_DC[i] = 0L;
			Red_HR_state.abuf_DC[i] = 0L;
		}	
		Red_HR_state.cbuf[i]= 0L;
	}
	
		/* initialize the HR state variable*/
	IR_HR_state.maybe_peak = 0;
	IR_HR_state.maybe_valley = 0;
	IR_HR_state.peak_detect = 0;
	IR_HR_state.valley_detect = 0;
	IR_HR_state.fiar = 0; //stands for five-in-a-row
	IR_HR_state.valley_min = 0L;
	IR_HR_state.peak_max = 0L;
	IR_HR_state.offset_DC = 0;
	IR_HR_state.offset_filt = 0;
	for(i = 0; i < 32; i++){
		if (i < 4){
			IR_HR_state.xbuf_DC[i] = 0L;
			IR_HR_state.abuf_DC[i] = 0L;
		}	
		IR_HR_state.cbuf[i]= 0L;
	}

	
	chThdSleepMilliseconds(300);
	
	chSemWait(&i2cUSE);
	i2c_init();
	if (!MAX30105_init()) {//Use default I2C port, 400kHz speed
		chSemWait(&usartUSE);
		fprintf_P(&usart_out,PSTR("MAX30105 was not found. Please check wiring/power. \r\n"));
		chSemSignal(&usartUSE);
	}
	
	if (USART_HB_LOGGING_ON ==1){
		MAX30105_setup(0x3F, 8, 2, 400, 411, 4096); //Configure sensor with default settings
	}
	else {
		if (SD_HB_LOGGING_ON ==1){
			MAX30105_setup(0x3F, 8, 2, 400, 411, 4096); //Configure sensor with default settings
		}
		else {
			MAX30105_setup(0x3F, 8, 2, 400, 411, 4096); //Configure sensor with default settings
		}	
	}
	chSemSignal(&i2cUSE);
	
	chSemWait(&usartUSE);
	fprintf_P(&usart_out,PSTR("MAX30105 set up .. lets go. \r\n"));
	chSemSignal(&usartUSE);

	//	immediately start to measure heartbeat - no coming back from this while loop
	chThdSleepMilliseconds(500);
	
	chSemWait(&sdUSE);	
	chSemWait(&spiUSE);
	res = disk_initialize(0);
	chSemSignal(&spiUSE);
	
	chThdSleepMilliseconds(500);
	
	chSemWait(&usartUSE);
	fprintf_P(&usart_out,PSTR("Disk Init: %d (0 = OK) \r\n"), res);
	chSemSignal(&usartUSE);
	
	chSemWait(&spiUSE);
	res = f_mount(0, &Fatfs[0]);
	chSemSignal(&spiUSE);
	
	chThdSleepMilliseconds(500);
	
	chSemWait(&usartUSE);
	fprintf_P(&usart_out,PSTR("f_mount: %d (0 = OK) \r\n"), res);
	chSemSignal(&usartUSE);
	
	chSemWait(&spiUSE);
	res = f_open(&fdst, "hrm_data.bin", FA_WRITE | FA_CREATE_ALWAYS);
	chSemSignal(&spiUSE);
	
	chThdSleepMilliseconds(500);		

	chSemWait(&usartUSE);
	fprintf_P(&usart_out,PSTR("f_open: %d (0 = OK) \r\n"), res);
	chSemSignal(&usartUSE);
	
	/*
	chSemWait(&spiUSE);
	res = f_lseek(&fdst, f_size(&fdst));
	chSemSignal(&spiUSE);
	
	chThdSleepMilliseconds(500);

	chSemWait(&usartUSE);
	fprintf_P(&usart_out,PSTR("f_lseek: %d (0 = OK) \r\n"), res);
	chSemSignal(&usartUSE);
	
	chSemWait(&spiUSE);
	res = f_write(&fdst, "New data\r\n", 10, &bw); 
	chSemSignal(&spiUSE);

	chThdSleepMilliseconds(500);
	
	chSemWait(&usartUSE);
	fprintf_P(&usart_out,PSTR("f_write1: %d (0 = OK) \r\n"), res);
	chSemSignal(&usartUSE);
		
	f_sync(&fdst);
	*/
	chThdSleepMilliseconds(500);
	chSemSignal(&sdUSE);
	
	xdev_out(usart_putchar_printf);
	
	HB_baseline = chVTGetSystemTimeX();	 

	
	while (1) {	
		if(chEvtWaitAnyTimeout((eventmask_t)1, TIME_IMMEDIATE) == 1){
			chSemWait(&lcdUSE); //grab the lcd/
			//switch lcd_context to thread 5
			lcd_context = 12;
			just_entered = 1;
			chSemWait(&spiUSE);
			st7735_fill_rect(0, 0, 160, 128, ST7735_COLOR_BLACK);//lcd_clear();
			st7735_setCursor(5,10);
			chSemSignal(&spiUSE);
			chSemWait(&sdUSE);
			bmpDraw("img00001.bmp",0,0); 
			chSemSignal(&sdUSE);
		}
		if(chEvtWaitAnyTimeout((eventmask_t)2, TIME_IMMEDIATE) == 2){
			st7735_setTextSize(1);
			lcd_context_count[11] = 1; //reset to HB mode
			chSemSignal(&lcdUSE);//thread 12 releases LCD
		}
		
		if(chEvtWaitAnyTimeout((eventmask_t)4, TIME_IMMEDIATE) == 4){
			just_entered=1;
		}
		
		// get current heartbeat 
		chSemWait(&i2cUSE);
		p1 = MAX30105_getRedIR(LED_data_vector); 
		chSemSignal(&i2cUSE);
		if (p1 > 0){//new data
			IR_result = checkForBeat(IR_LED_output, &IR_dc_avg, LED_data_vector[1], &IR_HR_state);//always
			if (lcd_context_count[11] == 2) { // in SPO2 mode, need red
				Red_result = checkForBeat(Red_LED_output, &Red_dc_avg, LED_data_vector[0], &Red_HR_state);
			}	
			/***********************************
			* logging
			***********************************/
			if ((USART_HB_LOGGING_ON ==1) || (SD_HB_LOGGING_ON ==1)){
				time_now = ST2MS(chVTTimeElapsedSinceX(HB_baseline));	//MS_2ST
				xsprintf_P(data_string, PSTR("%010lu,%11ld,%3u,%3ld\r\n"), time_now, IR_LED_output[0], IR_result,current_HB_estimate); 
			}
			 
			if (USART_HB_LOGGING_ON ==1){
				chSemWait(&usartUSE);
				xfprintf_P(xfunc_out,PSTR("%s"),data_string);				
				chSemSignal(&usartUSE);
			}
			
			if (SD_HB_LOGGING_ON ==1){
				if (SD_count ==0){
					strcpy(data_string_SD, data_string);
					SD_count++;
				}
				else {
					strcat(data_string_SD, data_string);
					SD_count++;
					if (SD_count == 16) {
						chSemWait(&sdUSE);
						chSemWait(&spiUSE);
						res = f_write(&fdst, data_string_SD,512, &bw); 
						f_sync(&fdst);
						chSemSignal(&spiUSE);
						chSemSignal(&sdUSE);
						SD_count = 0;
					}	
				}					
			}
			/***********************************
			* logging
			***********************************/
			
			if (IR_result == 1) { //found peak, happens rarely
				ms_current_beat = ST2MS(chVTTimeElapsedSinceX(HB_baseline));
				//need to be careful here if it is a double, we throw it out ... get times right!!!
				ms_elapsed_since_last = ms_current_beat - ms_old_beat;
				ms_old_beat = ms_current_beat;
				if (just_entered ==1){
					if(lcd_context ==12){
							if (lcd_context_count[11] == 1) { // in HB mode
								chSemWait(&spiUSE);
								st7735_fill_rect(32, 83, 96, 30, ST7735_COLOR_BLACK);//second coordinates are offset!!!
								st7735_setCursor(107,86);
								st7735_setTextSize(1);
								fprintf_P(&st7735_out, PSTR("BPM"));    
								chSemSignal(&spiUSE);
							}
							if (lcd_context_count[11] == 2) { // in SPO2 mode
								chSemWait(&spiUSE);
								st7735_fill_rect(32, 83, 96, 30, ST7735_COLOR_BLACK);//second coordinates are offset!!!
								st7735_setCursor(50,86);
								st7735_setTextSize(3);
								fprintf_P(&st7735_out,PSTR("---"));
								st7735_setCursor(107,86);
								st7735_setTextSize(1);
								fprintf_P(&st7735_out, PSTR("%%O"));
								st7735_setCursor(119,90);
								fprintf_P(&st7735_out, PSTR("2"));
								chSemSignal(&spiUSE);
							}
					}
					just_entered =0;
					blanking_time = 0UL;
				}//if just_entered ==1
				if (ms_elapsed_since_last > blanking_time) {
					current_HB_estimate = 60000L/(int32_t)ms_elapsed_since_last;
					if ((current_HB_estimate < 20L) || (current_HB_estimate > 230L)) { //then we have a bad value
					}
					else {
						//HEART BEAT FILTER & test whether its "locked"
						//ADJUST HB with a partial step to current value
						current_HB_average_last = current_HB_average;
						HB_ON = 1;
						if ((current_HB_estimate>(current_HB_average>>2)) && (current_HB_estimate < (current_HB_average<<2))){
							coeff =  (90L*60L*3L)/(8L*current_HB_average);
							if (coeff > attack_max) coeff = attack_max; 
						}		
						else coeff = 5L;
						if (current_HB_average > current_HB_estimate) {
							//K --;
							unlock_test = current_HB_average - current_HB_estimate;
							current_HB_average -= (coeff*(current_HB_average-current_HB_estimate))/100;
						}		
						else {
							//K++;
							unlock_test = current_HB_estimate - current_HB_average;
							current_HB_average += (coeff*(current_HB_estimate-current_HB_average))/100;
						}
						if (current_HB_average <=0) current_HB_average = current_HB_average_last;
						blanking_time = 30000L/(uint32_t)current_HB_average;
						if ((100L*unlock_test) < (10L*current_HB_average_last)){
							HB_locked = 1;
						}
						else {
							HB_locked = 0;
						}	
					}//HB filter		
					
					if(lcd_context ==12){
						if (lcd_context_count[11] == 1) { // in HB mode
							chSemWait(&spiUSE);
							st7735_setCursor(50,86);
							st7735_setTextSize(3);
							fprintf_P(&st7735_out,PSTR("%3d"), current_HB_average);
							st7735_setCursor(35,86);
							st7735_setTextSize(2);
							fprintf_P(&st7735_out,PSTR("%c"), 0x03);//should print heart
							chSemSignal(&spiUSE);
						}
						if ((lcd_context_count[11] == 2) && (HB_locked ==1)) { // in SPO2 mode, validated peak, grab IR data
							IR_SPO2_state.baseline = IR_dc_avg;
							IR_SPO2_state.spread = IR_HR_state.peak_max - IR_HR_state.valley_min;
							IR_SPO2_state.status = 1;	
						}
					}	
				}//ms_elapsed > blanking_time
			}//IR_result==1, found peak
			else {//result = 0, didn't find peak, finish HB visualization
				ms_elapsed_since_last_right_now = ST2MS(chVTTimeElapsedSinceX(HB_baseline)) - ms_old_beat;
				// HEART BEAT LCD FLASHER - LOGIC
				if (( HB_ON == 1) && (ms_elapsed_since_last_right_now > ((ms_elapsed_since_last)>>2))) {
					HB_ON = 0;
					if(lcd_context ==12){
						if (lcd_context_count[11] == 1) { // in HB mode
							chSemWait(&spiUSE);
							st7735_setCursor(35,86);
							st7735_setTextSize(2);
							fprintf_P(&st7735_out,PSTR(" "));
							chSemSignal(&spiUSE);
						}
					}	
				}
			}

			if ((lcd_context_count[11] == 2)&&(IR_SPO2_state.status == 1)&&(Red_result == 1)){ //SPO2, in a good HB regime, IR SPO2 is good, Red found 
				Red_SPO2_state.baseline = Red_dc_avg;
				Red_SPO2_state.spread = Red_HR_state.peak_max - Red_HR_state.valley_min;
				Red_SPO2_state.status = 1;
				if (Red_SPO2_state.status == 1) {//calculate SPO2
					//calculate SPO2
					R1 = (100L*Red_SPO2_state.spread)/IR_SPO2_state.spread;
					R2 = (100L*IR_SPO2_state.baseline)/Red_SPO2_state.baseline;
					R = (R1*R2)/10L;
					if ((R>300L) && (R<2300L)){
						//SPO2 = 106900L - ((39L*R)/10L) - ((156L*R*R)/10000L);
						SPO2 = 110000L - 25L*R; //simpler to start
						SPO2 /= 1000L;
						//fill and average circular buffer
						SPO2_circular_buffer[SPO2_circular_buffer_index] = SPO2;
						SPO2_circular_buffer_index++;
						SPO2_circular_buffer_index %= 7; //Wrap condition
					}
					SPO2_sum = 0L;
					for(SPO2_index = 0; SPO2_index < 7; SPO2_index++){
						SPO2_sum += SPO2_circular_buffer[SPO2_index];
					}
					SPO2_sum /= 7L;
					if(lcd_context ==12){
						chSemWait(&spiUSE);
						st7735_setCursor(35,86);
						st7735_setTextSize(2);
						fprintf_P(&st7735_out,PSTR(" "));
						st7735_setCursor(50,86);
						st7735_setTextSize(3);
						fprintf_P(&st7735_out,PSTR("%3ld"), SPO2_sum);
						chSemSignal(&spiUSE);
					}
					//reset all variables
					IR_SPO2_state.status = 0;
					Red_SPO2_state.status = 0;
				}
			}//red detected
		}//if (p2 > 0) - new data
		chThdSleepMilliseconds(2);
}//while(1)

}//THREAD12

/*********************************************************************************
 * Thread 13 - ADC
*********************************************************************************/
THD_FUNCTION(Thread13, arg) {
	(void)arg;
	uint8_t count = 0;
	uint8_t ch = 0b00000101;
	uint16_t average = 0;
	//setup ADC
	
	
	// AREF = AVcc
    ADMUX = (1<<REFS0);
    // ADC Enable and prescaler of 128, 16000000/128 = 125000
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	chThdSleepMilliseconds(200);
	// select the corresponding channel 0~7
	ADMUX = (ADMUX & 0xF8)|ch;     // clears the bottom 3 bits before ORing	
	
	
	while (1) {	
	count ++;
    
	// start single conversion
    // write '1' to ADSC
    ADCSRA |= (1<<ADSC);

    // wait for conversion to complete
    // ADSC becomes '0' again
    // till then, run loop continuously
    while(ADCSRA & (1<<ADSC)) chThdSleepMilliseconds(2);
	average += ADC;
	
	if (count > 30) {
		average = average/(uint16_t)count;
		battery_average = 660*(uint32_t)average/1024;
		average = 0;
		count = 0;
	}	
	
	chThdSleepMilliseconds(200);

}//while(1)

	

}//THREAD13

/*********************************************************************************
 * Thread 14 - Set Time 
 *********************************************************************************/
THD_FUNCTION(Thread14, arg) {
	(void)arg;
	// initialize the SD card

	tp[13] = chThdGetSelfX();
	msg_t msg;
	uint8_t time_position;
	
	while (true) {
		/* Waiting for signal to edit time - transfer via a message*/
		chEvtWaitAnyTimeout((eventmask_t)1, TIME_INFINITE);
		
		chSemWait(&lcdUSE);//grab lcd
		/*switch lcd_context to thread 14*/
		lcd_context = 14;
		
		//get current time dummy reference & print it and use it as starting point
		chSemWait(&i2cUSE);
		DS3231_getTime(&hr, &min, &s, &am_pm, hr_format); 
		chSemSignal(&i2cUSE);
                              
		chSemWait(&spiUSE);
		st7735_fill_rect(0, 0, 160, 128, ST7735_COLOR_BLACK);//lcd_clear();
		chSemSignal(&spiUSE);
		
		chSemWait(&spiUSE); 
		st7735_setCursor(0,50);//lcd_goto_xy(1,1);
		if (am_pm==1) {
			fprintf_P(&st7735_out, PSTR("%2u:%02u:%02u PM"), hr, min, s);       
		}
		else {
			fprintf_P(&st7735_out, PSTR("%2u:%02u:%02u AM"), hr, min, s);   
		}	
		chSemSignal(&spiUSE);
		
		time_position = 1;
		while(true) {
            
			if(chEvtWaitAnyTimeout((eventmask_t)8, TIME_IMMEDIATE) == 8){
				//write out time back to DS3231
				chSemWait(&i2cUSE);
				DS3231_setTime(hr, min, s, am_pm, hr_format);  
				chSemSignal(&i2cUSE);
				chSemSignal(&lcdUSE); /*thread 14 gives back the LCD*/
				break;
			}
			
			if(chEvtWaitAnyTimeout((eventmask_t)2, TIME_IMMEDIATE) == 2){
				time_position = lcd_context_count[13];
			}
			
			if(chEvtWaitAnyTimeout((eventmask_t)4, TIME_IMMEDIATE) == 4){
				if (time_position == 1) {
					hr++;
					if (hr>12) {
						hr = 1;
					}	
				}
				if (time_position == 2) {
					min++;
					if (min>59) {
						min = 0;
					}	
				}
				if (time_position == 3) {
					s++;
					if (s>59) {
						s = 0;
					}
				}	
				if (time_position == 4) {
					am_pm++;
					if (am_pm>1) {
						am_pm = 0;
					}	
				}
			}
			//print out time
			chSemWait(&spiUSE);
			st7735_setCursor(0,50);//lcd_goto_xy(1,1);
			if (am_pm==1) {
				fprintf_P(&st7735_out, PSTR("%2u:%02u:%02u PM"), hr, min, s);    
			}
			else {
				fprintf_P(&st7735_out, PSTR("%2u:%02u:%02u AM"), hr, min, s);   
			}	
			chSemSignal(&spiUSE);
			
			//give up slot
			chThdSleepMilliseconds(100);
		}//inner while
	}//outer while		
}//THREAD14

/*********************************************************************************
 * Memory Allocations
 *********************************************************************************/
THD_WORKING_AREA(waThread1, 128);
THD_WORKING_AREA(waThread2, 256);
THD_WORKING_AREA(waThread3, 768);
THD_WORKING_AREA(waThread4, 256);
THD_WORKING_AREA(waThread5, 768);
THD_WORKING_AREA(waThread6, 128);
THD_WORKING_AREA(waThread7, 256);
THD_WORKING_AREA(waThread8, 128);
THD_WORKING_AREA(waThread9, 256);
THD_WORKING_AREA(waThread10,256);
THD_WORKING_AREA(waThread11,256);
THD_WORKING_AREA(waThread12,1684);
THD_WORKING_AREA(waThread13,256);
THD_WORKING_AREA(waThread14,256);

/*********************************************************************************
 * Thread Table
 *
 * Remember to change threadcount!
 *********************************************************************************/

THD_TABLE_BEGIN
  THD_TABLE_ENTRY(waThread1, "ButtonHdl", Thread1, NULL)
  THD_TABLE_ENTRY(waThread2, "MenuHdl", Thread2, NULL)
  THD_TABLE_ENTRY(waThread3, "Blinky", Thread3, NULL)
  THD_TABLE_ENTRY(waThread4, "LCD_stuff", Thread4, NULL)
  THD_TABLE_ENTRY(waThread5, "LCD_but_nav", Thread5, NULL)
  THD_TABLE_ENTRY(wa_test_support, "test_support", test_support, (void *)&nil.threads[6]) //declared in test_root.c
  THD_TABLE_ENTRY(waThread6, "TestCode", Thread6, NULL)
  THD_TABLE_ENTRY(waThread7, "Thread info", Thread7, NULL)
  THD_TABLE_ENTRY(waThread8, "Thread status", Thread8, NULL)
  THD_TABLE_ENTRY(waThread9, "Blink Counter", Thread9, NULL)
  THD_TABLE_ENTRY(waThread10, "RTC", Thread10, NULL)
  THD_TABLE_ENTRY(waThread11, "Sensors", Thread11, NULL)
  THD_TABLE_ENTRY(waThread12, "HeartBeat", Thread12, NULL)
  THD_TABLE_ENTRY(waThread14, "Set Time", Thread14, NULL) //Set time, run at v. low priority
  THD_TABLE_ENTRY(waThread13, "ADC", Thread13, NULL) //ADC run at v. low priority
  THD_TABLE_ENTRY(waTimer, "time", timer, NULL) //declared in time.c, run at lowest priority
THD_TABLE_END


/*********************************************************************************
 * Application Entry Point
 *********************************************************************************/
int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  chSemObjectInit(&lcdUSE,1);
  chSemObjectInit(&sdUSE,1);
  chSemObjectInit(&spiUSE,1);
  chSemObjectInit(&i2cUSE,1);
  chSemObjectInit(&usartUSE,1);
  chSemObjectInit(&ButtonPressed,1);
  chSemObjectInit(&testTHREAD,1);

  
  nilFillStacks();
  halInit();
  chSysInit();
  /* This is now the idle thread loop, you may perform here a low priority
     task but you must never try to sleep or wait in this loop. Note that
     this tasks runs at the lowest priority level so any instruction added
     here will be executed after all other tasks have been started.*/
  while (true) {
  }
}
