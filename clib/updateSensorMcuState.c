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


#include "updateSensorMcuState.h"


void updateRawADCData (int16_t* adcData) {
	mlRawPressureData.press_abs 	= (int16_t)adcData[0];   // Baro
	mlRawPressureData.press_diff1 = (int16_t)adcData[1]; // Pito
	mlRawPressureData.press_diff2 = (int16_t)adcData[2]; // Power
	mlRawPressureData.temperature = (int16_t)adcData[3]; // Temp
}

void updateAirData (float* airData) {
	mlAirData.dynamicPressure = airData[0];							//dynamic
	mlAirData.staticPressure 	= airData[1];							//static
	mlAirData.temperature 		= (uint16_t) airData[2];	// temp
}

void updateLoadData (uint8_t load, uint16_t mvPower) {
	mlCpuLoadData.sensLoad 	= load;
	mlCpuLoadData.batVolt 	= mvPower;
}

void updateAttitude (float * attitudeData) {
	
	mlAttitudeData.roll 			= attitudeData[0];
	mlAttitudeData.pitch 			= attitudeData[1];
	mlAttitudeData.yaw 				= attitudeData[2];
	mlAttitudeData.rollspeed 	= attitudeData[3];
	mlAttitudeData.pitchspeed = attitudeData[4];
	mlAttitudeData.yawspeed 	= attitudeData[5];
	
}

void updateTimeStamp (uint32_t timeSt) {
	mlSystemTime.time_usec = (uint64_t)timeSt;
}

void updatePosition (float * posData) {
	
	mlLocalPositionData.x		= posData[0];
	mlLocalPositionData.y		= posData[1];
	mlLocalPositionData.z		= posData[2];
	mlLocalPositionData.vx	= posData[3];
	mlLocalPositionData.vy	= posData[4];
	mlLocalPositionData.vz	= posData[5];			
}

void updatePilotConsole (uint16_t * pilData) {
	mlPilotConsoleData.dt  	= pilData[0];
	mlPilotConsoleData.dla	= pilData[1];
	mlPilotConsoleData.dra	= pilData[2];
	mlPilotConsoleData.dr		= pilData[3];
	mlPilotConsoleData.de		= pilData[4];

}

void updateDiagnosticFl (float* diagFl) {
	mlDiagnosticData.diagFl1 =	diagFl[0];
	mlDiagnosticData.diagFl2 =	diagFl[1];
	mlDiagnosticData.diagFl3 =	diagFl[2];
}

void updateDiagnosticSh (int16_t* diagSh) {
	mlDiagnosticData.diagSh1 =	diagSh[0];
	mlDiagnosticData.diagSh2 =	diagSh[1];
	mlDiagnosticData.diagSh3 =	diagSh[2];
}

void updateBias (float * biasData) {
	mlSensorBiasData.axBias = biasData[0];
	mlSensorBiasData.ayBias = biasData[1];
	mlSensorBiasData.azBias = biasData[2];
	mlSensorBiasData.gxBias = biasData[3];
	mlSensorBiasData.gyBias = biasData[4];
	mlSensorBiasData.gzBias = biasData[5];		
}

void updateSensorData (float* sens){
	mlFilteredData.aX = sens[0];
	mlFilteredData.aY = sens[1];
	mlFilteredData.aZ = sens[2];
	mlFilteredData.mX = sens[3];
	mlFilteredData.mY = sens[4];
	mlFilteredData.mZ = sens[5];
	mlFilteredData.gX = 0;
	mlFilteredData.gY = 0;
	mlFilteredData.gZ = 0;
}

