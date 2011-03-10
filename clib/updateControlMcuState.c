
#include "updateControlMcuState.h"

void updatePWM(unsigned short * PWMData){
	// dT dA dE dE Failsafe
	mlPwmCommands.servo1_raw = PWMData[0];
	mlPwmCommands.servo2_raw = PWMData[1];
	mlPwmCommands.servo3_raw = PWMData[2];
	mlPwmCommands.servo4_raw = PWMData[3];
}

void updatePWMTrim(unsigned short PWMData, unsigned char channel ){
	switch (channel){
		case 0:
			mlPwmCommands.servo5_raw = PWMData;
		break; 
		case 1:
			mlPwmCommands.servo6_raw = PWMData;
		break; 
		case 2:
			mlPwmCommands.servo7_raw = PWMData;
		break; 
		case 3:
			mlPwmCommands.servo8_raw = PWMData;
		break; 								
	}
}

void updateLoad (uint8_t mcuLoad){
	mlCpuLoadData.ctrlLoad =  mcuLoad;
	mlSystemStatus.load = mcuLoad*10;
}

void updateEuler(float* newEuler){
	mlAttitudeRotated.roll	= newEuler[0];
	mlAttitudeRotated.pitch = newEuler[1];
	mlAttitudeRotated.yaw   = newEuler[2];
	mlAttitudeRotated.usec  = mlAttitudeData.usec;
		
	// mlAttitudeRotated.roll	= mlAttitudeData.roll;
	// mlAttitudeRotated.pitch = mlAttitudeData.pitch;
	// mlAttitudeRotated.yaw   = mlAttitudeData.yaw;
	// mlAttitudeRotated.usec  = mlAttitudeData.usec;
}


void updatePQR(float* newPQR){
	mlAttitudeRotated.rollspeed 	= newPQR[0];
	mlAttitudeRotated.pitchspeed 	= newPQR[1];
	mlAttitudeRotated.yawspeed 		= newPQR[2];
	
	// mlAttitudeRotated.rollspeed 	= mlAttitudeData.rollspeed;
	// mlAttitudeRotated.pitchspeed 	= mlAttitudeData.pitchspeed;
	// mlAttitudeRotated.yawspeed 		= mlAttitudeData.yawspeed;
}

// void updatePilotCommands (unsigned short*  pilCom){
// 	pilCom[0] = mlPilotConsoleData.dt;
// 	pilCom[1] = mlPilotConsoleData.dla;
// 	pilCom[2] = mlPilotConsoleData.dra;
// 	pilCom[3] = mlPilotConsoleData.dr;
// 	pilCom[4] = mlPilotConsoleData.de;
// }

unsigned char getHilOnOff (void){
	return (mlSystemStatus.mode == (uint8_t)MAV_MODE_TEST3);
}

