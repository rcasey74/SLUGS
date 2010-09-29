/*
The MIT License

Copyright (c) 2009 UCSC Autonomous Systems Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include "groundStationDriver.h"





unsigned char EEPInit(void){
	unsigned char eepInitMsg = 0;
	
	// Initialize the EEPROM emulation and read the PID Data
	eepInitMsg = DataEEInit();
	
	if (eepInitMsg == 1){
		aknControlData.pidCal = PIDEEP_PAGE_EXP; // Page Expired
	} else if (eepInitMsg == 6){
		aknControlData.pidCal = PIDEEP_MEMORY_CORR; // Memory Corrupted
	}
	
	return eepInitMsg;
}

void loadEEPData(void){
	unsigned char i;
	
	for(  i = 0; i < 10; i++ )
	{
		// PID Gains
		pidControlData.P[i].shData[0]= DataEERead(PID_OFFSET+i*6);
		pidControlData.P[i].shData[1]= DataEERead(PID_OFFSET+i*6+1);
		pidControlData.I[i].shData[0]= DataEERead(PID_OFFSET+i*6+2);
		pidControlData.I[i].shData[1]= DataEERead(PID_OFFSET+i*6+3);
		pidControlData.D[i].shData[0]= DataEERead(PID_OFFSET+i*6+4);
		pidControlData.D[i].shData[1]= DataEERead(PID_OFFSET+i*6+5);
		
		// Way Points
		wpsControlData.lat[i].shData[0]= DataEERead(WPS_OFFSET+i*8);   
		wpsControlData.lat[i].shData[1]= DataEERead(WPS_OFFSET+i*8+1);      
		wpsControlData.lon[i].shData[0]= DataEERead(WPS_OFFSET+i*8+2);      
		wpsControlData.lon[i].shData[1]= DataEERead(WPS_OFFSET+i*8+3);      
		wpsControlData.hei[i].shData[0]= DataEERead(WPS_OFFSET+i*8+4);      
		wpsControlData.hei[i].shData[1]= DataEERead(WPS_OFFSET+i*8+5);      
		wpsControlData.typ[i]		  = (unsigned char)DataEERead(WPS_OFFSET+i*8+6);
		wpsControlData.val[i].shData   = DataEERead(WPS_OFFSET+i*8+7);         	
	}
	// load WP 11 that contains the GS location
	i = 10;
	wpsControlData.lat[i].shData[0]= DataEERead(WPS_OFFSET+i*8);   
	wpsControlData.lat[i].shData[1]= DataEERead(WPS_OFFSET+i*8+1);      
	wpsControlData.lon[i].shData[0]= DataEERead(WPS_OFFSET+i*8+2);      
	wpsControlData.lon[i].shData[1]= DataEERead(WPS_OFFSET+i*8+3);      
	wpsControlData.hei[i].shData[0]= DataEERead(WPS_OFFSET+i*8+4);      
	wpsControlData.hei[i].shData[1]= DataEERead(WPS_OFFSET+i*8+5);      
	wpsControlData.typ[i]		  = (unsigned char)DataEERead(WPS_OFFSET+i*8+6);
	wpsControlData.val[i].shData   = DataEERead(WPS_OFFSET+i*8+7);         	

	// Compute the waypoint count
	i = 0;
	while ((int)(wpsControlData.lat[i].flData) != 0 && i < 9 ) i++;{
		wpsControlData.wpCount = i;
	}
}










