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

#include "navSupport.h"

void getMidLevelCommands(float* commands) {
	commands[0] = mlMidLevelCommands.uCommand;
	commands[1] = mlMidLevelCommands.hCommand;
	commands[2] = mlMidLevelCommands.rCommand;
}

unsigned char isApManual (void) {
	return mlApMode.mode == MAV_MODE_MANUAL;
}

void getPidIdx(unsigned char idx, float* PID){
	PID[0] = mlPidValues.P[idx];
	PID[1] = mlPidValues.I[idx];
	PID[2] = mlPidValues.D[idx]; 
}

float getDynamic (void) {
	return mlAirData.dynamicPressure;
} 

void getAttitude ( float* attitude){	
		
	// Return the values to the control algorithm
	attitude[0] = mlAttitudeData.roll;
	attitude[1] = mlAttitudeData.pitch;
	attitude[2] = mlAttitudeData.yaw;
	attitude[3] = mlAttitudeData.rollspeed;
	attitude[4] = mlAttitudeData.pitchspeed;
	attitude[5] = mlAttitudeData.yawspeed;	
	
}

void getXYZ (float* xyz) {
	xyz[0] =  mlLocalPositionData.x;
	xyz[1] =  mlLocalPositionData.y;
	xyz[2] =  mlLocalPositionData.z;
}

void getVned (float* xyz) {
	xyz[0] =  mlLocalPositionData.vx;
	xyz[1] =  mlLocalPositionData.vy;
	xyz[2] =  mlLocalPositionData.vz;
}
uint8_t getMaxWp (void){
	return mlWpValues.wpCount;
}

unsigned char isWpFly (void){
	return mlApMode.mode == MAV_MODE_AUTO;
}

void setDiagnosticFloat(float * flValues){
	mlDiagnosticData.diagFl1 = flValues[0];
	mlDiagnosticData.diagFl2 = flValues[1];
	mlDiagnosticData.diagFl3 = flValues[2];	
}

void setDiagnosticShort(int16_t* shValues){
	mlDiagnosticData.diagSh1 = shValues[0];
	mlDiagnosticData.diagSh2 = shValues[1];
	mlDiagnosticData.diagSh3 = shValues[2];	
}

void getWP (unsigned char idx, float* WPpos){
	WPpos[0] = mlWpValues.lat[idx-1];
	WPpos[1] = mlWpValues.lon[idx-1];
	WPpos[2] = mlWpValues.alt[idx-1];
}


void setLogFloat1(float * flValues){
	mlDataLog.fl_1 = flValues[0];
	mlDataLog.fl_2 = flValues[1];
	mlDataLog.fl_3 = flValues[2];	
}

void setLogFloat2(float * flValues){
	mlDataLog.fl_4 = flValues[0];
	mlDataLog.fl_5 = flValues[1];
	mlDataLog.fl_6 = flValues[2];	
}

unsigned char getApControlType (void) {
	return mlApMode.mode;
}

unsigned char getPassValues (uint8_t* pasVals){
	pasVals[0] = (uint8_t)(mlPassthrough.bitfieldPt & 128);
	pasVals[1] = (uint8_t)(mlPassthrough.bitfieldPt & 64);
	pasVals[2] = (uint8_t)(mlPassthrough.bitfieldPt & 16);
	pasVals[3] = (uint8_t)(mlPassthrough.bitfieldPt & 8);	
}

void setCurrentCommands (float airSpeed){
	mlMidLevelCommands.uCommand = airSpeed;
	mlMidLevelCommands.hCommand = mlLocalPositionData.z;
	mlMidLevelCommands.rCommand = 0.0;
	
}


void setNavLong (float* values) {
	mlNavigation.u_m 		 = values[0];
	mlNavigation.theta_c = values[1];
}

void setNavLat (float* values) {
	mlNavigation.psiDot_c = values[0];
	mlNavigation.phi_c    = values[1];
	mlNavigation.ay_body  = values[2];
}

void setNavNav (float* values) {
	mlNavigation.totalDist = values[0];
	mlNavigation.dist2Go   = values[1];
	mlNavigation.fromWP 	 = (uint8_t) values[2];
	mlNavigation.toWP 		 = (uint8_t) values[3];
}

void getAccels (float * accels){
	accels[0] = mlFilteredData.aX;
	accels[1] = mlFilteredData.aY;
	accels[2] = mlFilteredData.aZ;
}

void getAccBias (float * bias){
	bias[0] = mlSensorBiasData.axBias;
	bias[1] = mlSensorBiasData.ayBias;
	bias[2] = mlSensorBiasData.azBias;
}

// void bufferICValues(unsigned short latest, unsigned short* history){
// 	static unsigned short * oldValues[] = {0, 0, 0, 0, 0, 0, 0};
// 	unsigned char i;
// 	for (i=6; i>0; i--){
// 		oldValues[i] = oldValues[i-1];
// 		history[i] = oldValues[i];
// 	}
// 	
// 	oldValues[0] = latest;
// 	history[0] = latest;
// }

unsigned short meanFilter5(unsigned short * values){
	quickSort(values, 7);
	return values[3];
}

//  quickSort
//
//  This public-domain C implementation by Darel Rex Finley.
//
//  * Returns YES if sort was successful, or NO if the nested
//    pivots went too deep, in which case your array will have
//    been re-ordered, but probably not sorted correctly.
//
//  * This function assumes it is called with valid parameters.
//
//  * Example calls:
//    quickSort(&myArray[0],5); // sorts elements 0, 1, 2, 3, and 4
//    quickSort(&myArray[3],5); // sorts elements 3, 4, 5, 6, and 7
//
// 	source: http://alienryderflex.com/quicksort/

 
unsigned char quickSort(unsigned short *arr, char elements) {

  #define  MAX_LEVELS  8

	short i=0, L, R ;
	unsigned short  piv, beg[MAX_LEVELS], end[MAX_LEVELS];

  beg[0]=0; end[0]=elements;
  while (i>=0) {
    L=beg[i]; R=end[i]-1;
    if (L<R) {
      piv=arr[L]; if (i==MAX_LEVELS-1) return 0;
      while (L<R) {
        while (arr[R]>=piv && L<R) {
        	R--; 
        }
        if (L<R){ 
        	arr[L++]=arr[R];
        }
        while (arr[L]<=piv && L<R) {
        	L++;
        } 
        if (L<R) {
        	arr[R--]=arr[L];
      	}   
      }
      arr[L]=piv; beg[i+1]=L+1; end[i+1]=end[i]; end[i++]=L; 
    } else {
      i--;     
    }
  }
  return 1; 
}
