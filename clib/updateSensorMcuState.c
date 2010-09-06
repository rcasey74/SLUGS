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
	mlRawPressureData.press_abs = (int16_t)adcData[0];   // Baro
	mlRawPressureData.press_diff1 = (int16_t)adcData[1]; // Pito
	mlRawPressureData.press_diff2 = (int16_t)adcData[2]; // Power
	mlRawPressureData.temperature = (int16_t)adcData[3]; // Temp
}

void updateAirData (float* airData) {
	dynTempControlData.dynamic.flData 	= airData[0];
	dynTempControlData.stat.flData 		= airData[1];
	dynTempControlData.temp.shData 		= (short) airData[2];
}

void updateLoadData (unsigned char load, unsigned short mvPower) {
	statusControlData.load = load;
	statusControlData.bVolt.usData = mvPower;
}

void updateAttitude (float * attitudeData) {
	attitudeControlData.roll.flData 	= attitudeData[0];
	attitudeControlData.pitch.flData 	= attitudeData[1];
	attitudeControlData.yaw.flData 		= attitudeData[2];
	attitudeControlData.p.flData 		= attitudeData[3];
	attitudeControlData.q.flData 		= attitudeData[4];
	attitudeControlData.r.flData 		= attitudeData[5];
}

void updateTimeStamp (unsigned short timeSt) {
	attitudeControlData.timeStamp.usData = timeSt;
}

void updatePosition (float * posData) {
	xyzControlData.Xcoord.flData		= posData[0];
	xyzControlData.Ycoord.flData		= posData[1];
	xyzControlData.Zcoord.flData		= posData[2];
	xyzControlData.VX.flData			= posData[3];
	xyzControlData.VY.flData			= posData[4];
	xyzControlData.VZ.flData			= posData[5];
}

void updatePilotConsole (unsigned short * pilData) {
	pilControlData.dt.usData			= pilData[0];
	pilControlData.dla.usData			= pilData[1];
	pilControlData.dra.usData			= pilData[2];
	pilControlData.dr.usData			= pilData[3];
	pilControlData.de.usData			= pilData[4];
}

void updateDiagnosticFl (float* diagFl) {
	diagControlData.fl1.flData =	diagFl[0];
	diagControlData.fl2.flData =	diagFl[1];
	diagControlData.fl3.flData =	diagFl[2];
}

void updateDiagnosticSh (short* diagSh) {
	diagControlData.sh1.shData =	diagSh[0];
	diagControlData.sh2.shData =	diagSh[1];
	diagControlData.sh3.shData =	diagSh[2];
}

void updateBias (float * biasData) {
	biasControlData.gxb.flData		= biasData[0];
	biasControlData.gyb.flData		= biasData[1];
	biasControlData.gzb.flData		= biasData[2];
	biasControlData.axb.flData		= biasData[3];
	biasControlData.ayb.flData		= biasData[4];
	biasControlData.azb.flData		= biasData[5];
}

void updateSensorData (float* sens){
	senControlData.Ax.flData			= sens[0];
	senControlData.Ay.flData			= sens[1];
	senControlData.Az.flData			= sens[2];
	senControlData.Mx.flData			= sens[3];
	senControlData.My.flData			= sens[4];
	senControlData.Mz.flData			= sens[5];
}

