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
	mlRawPressureData.usec = mlAttitudeData.usec;
	mlRawPressureData.press_abs 	= (int16_t)adcData[0]; // Baro
	mlRawPressureData.press_diff1 = (int16_t)adcData[1]; // Pito
	mlRawPressureData.press_diff2 = (int16_t)adcData[2]; // Power
	mlRawPressureData.temperature = (int16_t)adcData[3]; // Temp
}

void updateAirData (float* airData) {
	mlAirData.usec 				= mlAttitudeData.usec;
	mlAirData.press_diff  = airData[0]*0.01;		//dynamic
	mlAirData.press_abs 	= airData[1]*0.01;		//static
	mlAirData.temperature = (uint16_t) (airData[2]*10.0);	// temp 0.01
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
	mlAttitudeData.usec = (uint64_t)timeSt;
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
	// dT dA dR dE dFail
	
	mlPilotConsoleData.chan1_raw  = pilData[0];
	mlPilotConsoleData.chan2_raw	= pilData[1];
	mlPilotConsoleData.chan3_raw	= pilData[2];
	mlPilotConsoleData.chan4_raw	= pilData[3];
	mlPilotConsoleData.chan5_raw	= pilData[4];
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
	mlFilteredData.xacc = (int16_t)(sens[0]* MPS_TO_MG);
	mlFilteredData.yacc = (int16_t)(sens[1]* MPS_TO_MG);
	mlFilteredData.zacc = (int16_t)(sens[2]* MPS_TO_MG);
	mlFilteredData.xmag = (int16_t)(sens[3]*1000.0); // assuming milligaus
	mlFilteredData.ymag = (int16_t)(sens[4]*1000.0);
	mlFilteredData.zmag = (int16_t)(sens[5]*1000.0);
	mlFilteredData.xgyro = (int16_t)(sens[6]*1000.0);
	mlFilteredData.ygyro = (int16_t)(sens[7]*1000.0);
	mlFilteredData.zgyro = (int16_t)(sens[8]*1000.0);
}

