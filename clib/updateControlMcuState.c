
#include "updateControlMcuState.h"

void updatePWM(unsigned short * PWMData){
	mlPwmCommands.dt_c 	= PWMData[0];
	mlPwmCommands.dla_c = PWMData[1];
	mlPwmCommands.dle_c = PWMData[2];
	mlPwmCommands.dr_c	= PWMData[3];
	mlPwmCommands.dre_c = PWMData[4];	
}

void updatePWM2(unsigned short PWMData, unsigned char channel ){
	switch (channel){
		case 0:
			mlPwmCommands.dre_c = PWMData;
		break; 
		case 1:
			mlPwmCommands.dlf_c = PWMData;
		break; 
		case 2:
			mlPwmCommands.drf_c = PWMData;
		break; 
		case 3:
			mlPwmCommands.aux1 = PWMData;
		break; 
		case 4:
			mlPwmCommands.aux2 = PWMData;
		break; 
								
	}
}

void updateLoad (uint8_t mcuLoad){
	mlCpuLoadData.ctrlLoad =  mcuLoad;
	mlSystemStatus.load = mcuLoad*10;
}

void updateEuler(float* newEuler){
	// mlAttitudeRotated.roll	= newEuler[0];
	// mlAttitudeRotated.pitch = newEuler[1];
	// mlAttitudeRotated.yaw   = newEuler[2];
	
	mlAttitudeRotated.roll	= mlAttitudeData.roll;
	mlAttitudeRotated.pitch = mlAttitudeData.pitch;
	mlAttitudeRotated.yaw   = mlAttitudeData.yaw;
	mlAttitudeRotated.usec  = mlAttitudeData.usec;
}


void updatePQR(float* newPQR){
	// mlAttitudeRotated.rollspeed 	= newPQR[0];
	// mlAttitudeRotated.pitchspeed 	= newPQR[1];
	// mlAttitudeRotated.yawspeed 		= newPQR[2];
	
	mlAttitudeRotated.rollspeed 	= mlAttitudeData.rollspeed;
	mlAttitudeRotated.pitchspeed 	= mlAttitudeData.pitchspeed;
	mlAttitudeRotated.yawspeed 		= mlAttitudeData.yawspeed;
}

void updatePilotCommands (unsigned short*  pilCom){
	pilCom[0] = mlPilotConsoleData.dt;
	pilCom[1] = mlPilotConsoleData.dla;
	pilCom[2] = mlPilotConsoleData.dra;
	pilCom[3] = mlPilotConsoleData.dr;
	pilCom[4] = mlPilotConsoleData.de;
}

