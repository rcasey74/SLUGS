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

#include "eepLoader.h"

unsigned char EEPInit(void){
	unsigned char eepInitMsg = 0;
	
	// Initialize the EEPROM emulation and read the PID Data
	eepInitMsg = DataEEInit();
	
	if (eepInitMsg == 1){
		mlActionAck.action = SLUGS_ACTION_EEPROM; // EEPROM Action
		mlActionAck.result = EEP_PAGE_EXP; // Page Expired
	} else if (eepInitMsg == 6){
		mlActionAck.action = SLUGS_ACTION_EEPROM; // EEPROM Action
		mlActionAck.result = EEP_MEMORY_CORR; // Memory Corrupted
	}
	
	return eepInitMsg;
}

void loadEEPData(void){
	unsigned char i;
	tFloatToChar tempShData;
	
	for(  i = 0; i < MAX_NUM_PIDS; i++ ){
		// PID Gains
		
		tempShData.shData[0]= DataEERead(PID_OFFSET+i*6);
		tempShData.shData[1]= DataEERead(PID_OFFSET+i*6+1);
		mlPidValues.P[i]	= tempShData.flData;
		
		tempShData.shData[0]= DataEERead(PID_OFFSET+i*6+2);
		tempShData.shData[1]= DataEERead(PID_OFFSET+i*6+3);
		mlPidValues.I[i] = tempShData.flData;
		
		tempShData.shData[0]= DataEERead(PID_OFFSET+i*6+4);
		tempShData.shData[1]= DataEERead(PID_OFFSET+i*6+5);
		mlPidValues.D[i] = tempShData.flData;
	}
	
	for(i = 0; i < MAX_NUM_WPS; i++ ){		
		// Way Points
		tempShData.shData[0]= DataEERead(WPS_OFFSET+i*8);   
		tempShData.shData[1]= DataEERead(WPS_OFFSET+i*8+1);      
		mlWpValues.lat[i] = tempShData.flData;      
		
		tempShData.shData[0]= DataEERead(WPS_OFFSET+i*8+2);      
		tempShData.shData[1]= DataEERead(WPS_OFFSET+i*8+3);      
		mlWpValues.lon[i] = tempShData.flData;      
		
		tempShData.shData[0]= DataEERead(WPS_OFFSET+i*8+4);      
		tempShData.shData[1]= DataEERead(WPS_OFFSET+i*8+5);      
		mlWpValues.alt[i] = tempShData.flData;      
		
		mlWpValues.type[i]	= (uint8_t)DataEERead(WPS_OFFSET+i*8+6);
		
		mlWpValues.orbit[i]   = DataEERead(WPS_OFFSET+i*8+7);         	
		         	
	}

	// Compute the waypoint count
	mlWpValues.wpCount = 0;
	while ((int)(mlWpValues.lat[mlWpValues.wpCount]) != 0 && mlWpValues.wpCount< MAX_NUM_WPS-1 ){
		mlWpValues.wpCount++;
	}
}










