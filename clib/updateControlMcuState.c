
#include "updateControlMcuState.h"

void updatePWM(unsigned short * PWMData){
	pwmControlData.dt_c.usData 	= PWMData[0];
	pwmControlData.dla_c.usData = PWMData[1];
	pwmControlData.dra_c.usData = PWMData[2];
	pwmControlData.dr_c.usData 	= PWMData[3];
	pwmControlData.dle_c.usData = PWMData[4];	
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
	mlCpuLoad.ctrlLoad =  mcuLoad;
}

void updateEuler(float* newEuler){
	attitudeRotatedControlData.roll.flData = newEuler[0];
	attitudeRotatedControlData.pitch.flData = newEuler[1];
	attitudeRotatedControlData.yaw.flData = newEuler[2];
	attitudeRotatedControlData.timeStamp.usData = attitudeControlData.timeStamp.usData;
}


void updatePQR(float* newPQR){
	attitudeRotatedControlData.p.flData = newPQR[0];
	attitudeRotatedControlData.q.flData = newPQR[1];
	attitudeRotatedControlData.r.flData = newPQR[2];
}

void updatePilotCommands (unsigned short*  pilCom){
	pilCom[0] = pilControlData.dt.usData;
	pilCom[1] = pilControlData.dla.usData;
	pilCom[2] = pilControlData.dra.usData;
	pilCom[3] = pilControlData.dr.usData;
	pilCom[4] = pilControlData.de.usData;
}

