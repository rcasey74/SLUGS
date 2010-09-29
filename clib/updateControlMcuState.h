#ifndef _UPDATECONTROLMCUSTATE_H_
#define _UPDATECONTROLMCUSTATE_H_


#ifdef __cplusplus
       extern "C"{
#endif
      
#include "mavlinkControlMcu.h" 	

void updatePilotCommands (unsigned short* pilCom);
void updatePWM (unsigned short * PWMData);
void updatePWM2 (unsigned short PWMData, unsigned char channel );
void updateLoad (uint8_t mcuLoad);
void updateEuler (float* newEuler);
void updatePQR (float* newPQR);
       
       	
       	
       		
#ifdef __cplusplus
       }
#endif
       
       
#endif /* _UPDATECONTROLMCUSTATE_H_ */
