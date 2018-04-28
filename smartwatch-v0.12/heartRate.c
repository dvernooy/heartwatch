/*
 Optical Heart Rate Detection (PBA Algorithm)
 By: Nathan Seidle
 SparkFun Electronics
 Date: October 2nd, 2016
 
 Given a series of IR samples from the MAX30105 we discern when a heart beat is occurring

 Let's have a brief chat about what this code does. We're going to try to detect
 heart-rate optically. This is tricky and prone to give false readings. We really don't
 want to get anyone hurt so use this code only as an example of how to process optical
 data. Build fun stuff with our MAX30105 breakout board but don't use it for actual
 medical diagnosis.

 Excellent background on optical heart rate detection:
 http://www.ti.com/lit/an/slaa655/slaa655.pdf

 Good reading:
 http://www.techforfuture.nl/fjc_documents/mitrabaratchi-measuringheartratewithopticalsensor.pdf
 https://fruct.org/publications/fruct13/files/Lau.pdf

 This is an implementation of Maxim's PBA (Penpheral Beat Amplitude) algorithm. It's been 
 converted to work within the Arduino framework.
*/

/* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
* 
*/

#include "heartRate.h"
#include "MAX30105.h"

#define FIAR_PARAM 6
#define FILT_AV 32 //change also in heartrate.h
#define LOG_2_FILT_AV 5

//1.5Hz
//static const int32_t FIRCoeffs[12] = {172L, 321L, 579L, 927L, 1360L, 1858L, 2390L, 2916L, 3391L, 3768L, 4012L, 4096L};//the original

//2Hz
static const int32_t FIRCoeffs[12] = {-111L, -83L, -10L, 202L, 603L, 1212L, 2009L, 2923L, 3842L, 4632L, 5167L, 5356L};//the original

//3Hz
//static const int32_t FIRCoeffs[12] = {-150L, -265L, -376L, -360L, -96L, 515L, 1502L, 2794L, 4210L, 5502L, 6409L, 6735L};//higher bw


/**********************************************
state parameters for detection algorithm
**********************************************/

//  Heart Rate Monitor functions takes a sample value and the sample number
//  Returns true if a beat is detected
//  A running average of four samples is recommended for display on the screen.
uint8_t checkForBeat(int32_t *output_buffer, int32_t *avg_estimate, int32_t sample, struct HR_algo_state* HR_state) {
  uint8_t beatDetected = 0;
  uint8_t i = 0;
  int32_t z = 0;
  *avg_estimate = 0L;

  //  Save current state
  output_buffer[2] = output_buffer[1];
  output_buffer[1] = output_buffer[0];
  
  HR_state->xbuf_DC[HR_state->offset_DC] = sample;
  
  HR_state->abuf_DC[HR_state->offset_DC] = 0L;
  
  /*****  Strip out the DC *********/
  for (i = 0 ; i < FILT_AV ; i++) {
	HR_state->abuf_DC[HR_state->offset_DC] += HR_state->xbuf_DC[i];
  }
  HR_state->abuf_DC[HR_state->offset_DC] = (HR_state->abuf_DC[HR_state->offset_DC])>>LOG_2_FILT_AV;
  
  for (i = 0 ; i < FILT_AV ; i++) {
	*avg_estimate += HR_state->abuf_DC[i];
  }
  *avg_estimate = *avg_estimate>>LOG_2_FILT_AV;

  output_buffer[0] = (*avg_estimate - HR_state->xbuf_DC[(HR_state->offset_DC+1)%FILT_AV])<<2 ; 
  HR_state->offset_DC++;
  HR_state->offset_DC %= FILT_AV; //Wrap condition
  /*****  Strip out the DC *********/
  
  /*****  Low Pass FIR Filter *********/
  HR_state->cbuf[HR_state->offset_filt] = output_buffer[0];
  z = FIRCoeffs[11] * HR_state->cbuf[(HR_state->offset_filt - 11) & 0x1F];
  
  for (uint8_t i = 0 ; i < 11 ; i++)
  {
    z += FIRCoeffs[i] *(HR_state->cbuf[(HR_state->offset_filt - i) & 0x1F] + HR_state->cbuf[(HR_state->offset_filt - 22 + i) & 0x1F]);
  }

  HR_state->offset_filt++;
  HR_state->offset_filt %= 32; //Wrap condition
	
  output_buffer[0] = (z>>8);//the original
  output_buffer[0] *= -1;//the original	
  /*****  Low Pass FIR Filter *********/

  /*****  Actual Algorithm *********/
  if ((output_buffer[2]  == output_buffer[1]) && (output_buffer[1] < output_buffer[0])) {
	if ( HR_state->maybe_valley==1) {
		 HR_state->valley_detect = 1;
		 HR_state->valley_min = output_buffer[1];
		 HR_state->maybe_valley = 0;
	}	
	if ( HR_state->maybe_peak==1) {
		 HR_state->maybe_peak = 0;
	}
  }
  
  //dot-flat-down
  if ((output_buffer[2]  == output_buffer[1]) && (output_buffer[1] > output_buffer[0])) {
	if ( HR_state->maybe_valley ==1) {
		 HR_state->maybe_valley = 0;
	}	
	if (( HR_state->maybe_peak==1)&&(output_buffer[1] >0)) {
		 HR_state->maybe_peak = 0;
		 HR_state->peak_detect = 1;
		 HR_state->peak_max = output_buffer[1];
	}
  }
  
  //dot-up-flat
  if ((output_buffer[2] < output_buffer[1]) && (output_buffer[1] == output_buffer[0])) {
	 HR_state->maybe_peak = 1;
  }
  
  //dot-up-down
  if ((output_buffer[2]  < output_buffer[1]) && (output_buffer[1] > output_buffer[0]) && (output_buffer[1] >0)) {
	 HR_state->peak_detect = 1;
	 HR_state->peak_max = output_buffer[1];
  }
  
  //dot-down-up
  if ((output_buffer[2]  > output_buffer[1]) && (output_buffer[1] < output_buffer[0])) {
	 HR_state->valley_detect = 1;
	 HR_state->valley_min = output_buffer[1];
  }
  
  //dot-down-flat
  if ((output_buffer[2]  > output_buffer[1]) && (output_buffer[1] == output_buffer[0])) {
	 HR_state->maybe_valley = 1;
  }
  
  //  special: detect fiar (five-in-a-row going up)
  if ( HR_state->fiar < FIAR_PARAM) {
	if (output_buffer[1] <= output_buffer[0]) {
		 HR_state->fiar++;
	} // detect fiar
	else {
		if (HR_state->fiar >0){
			 HR_state->fiar = 0;
		}
	}
  }
  
  //  if detected peak
  if ( HR_state->peak_detect ==1) {
	// check whether its a valid beat
	if(( HR_state->valley_detect ==1) && ( HR_state->fiar == FIAR_PARAM) && (( HR_state->peak_max -  HR_state->valley_min) > 75L) && (( HR_state->peak_max -  HR_state->valley_min) < 2500000L)) {
      //Heart beat!!!
      beatDetected = 1;
    }
	//peak was detected, so no matter what, reset everything
	 HR_state->maybe_peak = 0;
	 HR_state->maybe_valley = 0;
	 HR_state->peak_detect = 0;
	 HR_state->valley_detect = 0;
	 HR_state->fiar = 0;
	 //HR_state->valley_min = 0;
	 //HR_state->peak_max = 0;
  }//detect peak
 
 return(beatDetected);
}

