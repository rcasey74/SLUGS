#include "hil.h"

void hilRead(unsigned char* hilChunk){
	extern CBRef uartBufferIn;
	
	// fix the data length so if the interrupt adds data
	// during execution of this block, it will be read
	// until the next hilRead
	unsigned char tmpLen = getLength(uartBufferIn), i=0;
	
	// if the buffer has more data than the max size, set it to max,
	// otherwise set it to the length
	hilChunk[0] =  (tmpLen > MAXSEND -1)? MAXSEND -1: tmpLen;
	
	// read the data 
	for(i = 1; i <= hilChunk[0]; i += 1 )
	{
		hilChunk[i] = readFront(uartBufferIn);
	}
}

void hil_getRawRead(short * rawData){
	rawData[0] =  	rawControlData.gyroX.shData;
	rawData[1] =  	rawControlData.gyroY.shData;
	rawData[2] =  	rawControlData.gyroZ.shData;
	rawData[3] = 	rawControlData.accelX.shData;
	rawData[4] = 	rawControlData.accelY.shData;
	rawData[5] = 	rawControlData.accelZ.shData;
	rawData[6] = 	rawControlData.magX.shData;
	rawData[7] = 	rawControlData.magY.shData;
	rawData[8] = 	rawControlData.magZ.shData;
	rawData[9] = 	rawControlData.baro.shData;
	rawData[10] = 	rawControlData.pito.shData;
	rawData[11] = 	rawControlData.powr.shData;
	rawData[12] = 	rawControlData.ther.shData;
}

void hil_getGPSRead (unsigned char * gpsMsg){
		// write the output data;
		gpsMsg[0]  = gpsControlData.year;					
		gpsMsg[1]  = gpsControlData.month;					
		gpsMsg[2]  = gpsControlData.day;	
		gpsMsg[3]  = gpsControlData.hour;
		gpsMsg[4]  = gpsControlData.min;
		gpsMsg[5]  = gpsControlData.sec;
		gpsMsg[6]  = gpsControlData.lat.chData[0];
		gpsMsg[7]  = gpsControlData.lat.chData[1];
		gpsMsg[8]  = gpsControlData.lat.chData[2];					
		gpsMsg[9]  = gpsControlData.lat.chData[3];
		gpsMsg[10] = gpsControlData.lon.chData[0];
		gpsMsg[11] = gpsControlData.lon.chData[1];
		gpsMsg[12] = gpsControlData.lon.chData[2];					
		gpsMsg[13] = gpsControlData.lon.chData[3];					
		gpsMsg[14] = gpsControlData.height.chData[0];
		gpsMsg[15] = gpsControlData.height.chData[1];
		gpsMsg[16] = gpsControlData.height.chData[2];					
		gpsMsg[17] = gpsControlData.height.chData[3];					
		gpsMsg[18] = gpsControlData.cog.chData[0];					
		gpsMsg[19] = gpsControlData.cog.chData[1];									
		gpsMsg[20] = gpsControlData.sog.chData[0];					
		gpsMsg[21] = gpsControlData.sog.chData[1];
		gpsMsg[22] = gpsControlData.hdop.chData[0];					
		gpsMsg[23] = gpsControlData.hdop.chData[1];
		gpsMsg[24] = gpsControlData.fix;
		gpsMsg[25] = gpsControlData.sats;									
		gpsMsg[26] = gpsControlData.newValue;								
}


void hil_getVned(float* vned){
	vned[0] = xyzControlData.VX.flData;
	vned[1] = xyzControlData.VY.flData;
	vned[2] = xyzControlData.VZ.flData;
}

void hil_getXYZ(float* xyz){
	xyz[0] = xyzControlData.Xcoord.flData;
	xyz[1] = xyzControlData.Ycoord.flData;
	xyz[2] = xyzControlData.Zcoord.flData;
}

void hil_getEuler(float* euler){
	euler[0] = attitudeControlData.roll.flData;
	euler[1] = attitudeControlData.pitch.flData;
	euler[2] = attitudeControlData.yaw.flData;
}

void hil_getRates(float* pqr){
	pqr[0] = attitudeControlData.p.flData;
	pqr[1] = attitudeControlData.q.flData;
	pqr[2] = attitudeControlData.r.flData;
}

unsigned short hil_getTs(void){
	return attitudeControlData.timeStamp.usData;
}

unsigned char getHilOnOff (void){
	return 0;
}
}

