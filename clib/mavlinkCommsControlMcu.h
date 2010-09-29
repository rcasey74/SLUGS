#ifndef _MAVLINKCOMMSCONTROLMCU_H_
#define _MAVLINKCOMMSCONTROLMCU_H_

#ifdef __cplusplus
       extern "C"{
#endif
       	
#include "circBuffer.h"
#include "apDefinitions.h"
#include "mavlinkControlMcu.h"
#include "apUtils.h"
#include <p33fxxxx.h>       	
       	
	void uart2Init (void);
  void send2GS (unsigned char* protData);
  void gsRead (unsigned char* gsChunk);
  
	void prepareTelemetryMavlink (unsigned char* dataOut);
  void protDecodeMavlink (uint8_t* dataIn, uint8_t commChannel);          
       	
       	
       		
#ifdef __cplusplus
       }
#endif
       

#endif /* _MAVLINKCOMMSCONTROLMCU_H_ */
