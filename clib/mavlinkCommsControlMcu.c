#include "mavlinkCommsControlMcu.h"

struct CircBuffer com2BufferIn;
CBRef uartBufferIn;


struct CircBuffer com1BufferOut;
CBRef uartBufferOut;

struct CircBuffer gsParseBuffer; // from GS (i.e receive commands from GS)
CBRef gsBuffer;

unsigned int BufferB[MAXSEND] __attribute__((space(dma))) = {0};

// UART, DMA and Buffer initialization
void uart2Init (void){
	
	// initialize the circular buffers
	uartBufferIn = (struct CircBuffer* )&com2BufferIn;
	newCircBuffer(uartBufferIn);

	uartBufferOut = (struct CircBuffer* )&com1BufferOut;
	newCircBuffer(uartBufferOut);
	
	gsBuffer = (struct CircBuffer* )&gsParseBuffer;
	newCircBuffer(gsBuffer);
	
	// DMA1REQ Register
	// ================
	DMA1REQ = 0x001F;
	
	// DMA1PAD Register
	// ================
	DMA1PAD = (volatile unsigned int) &U2TXREG;
	
	// DMA1CON Register
	// ================
	DMA1CONbits.AMODE   = 0;        // Register Indirect with post-increment
	DMA1CONbits.MODE    = 1;        // One-shot, No Ping-Pong Mode	
	DMA1CONbits.DIR     = 1;        // Read from RAM and send to Periphereal
	DMA1CONbits.SIZE    = 0;        // Word Data Transfer

	// DMA1CNT Register
	// ==============
	DMA1CNT = MAXSEND-1;

	// DMA1STA Register
	// ================
	DMA1STA= __builtin_dmaoffset(BufferB);

	// Enable DMA1 TX interrupts
	IFS0bits.DMA1IF  = 0;			// Clear DMA Interrupt Flag
	IPC3bits.DMA1IP  = 6;			// interrupt priority to 6
	IEC0bits.DMA1IE  = 1;			// Enable DMA interrupt
	
	// Configure and open the port;
	// U2MODE Register
	// ==============
	U2MODEbits.UARTEN	= 0;		// Disable the port		
	U2MODEbits.USIDL 	= 0;		// Stop on idle
	U2MODEbits.IREN		= 0;		// No IR decoder
	U2MODEbits.RTSMD	= 0;		// Ready to send mode (irrelevant)
	U2MODEbits.UEN		= 0;		// Only RX and TX
	U2MODEbits.WAKE		= 1;		// Enable at startup
	U2MODEbits.LPBACK	= 0;		// Disable loopback
	U2MODEbits.ABAUD	= 0;		// Disable autobaud
	U2MODEbits.URXINV	= 0;		// Normal operation (high is idle)
	U2MODEbits.PDSEL	= 0;		// No parity 8 bit
	U2MODEbits.STSEL	= 0;		// 1 stop bit
	U2MODEbits.BRGH 	= 0;		// Low speed mode
	
	// U1STA Register
	// ==============
	U2STAbits.UTXISEL0	= 0;		// generate interrupt on every char
	U2STAbits.UTXISEL1	= 0;		// for the DMA	
	U2STAbits.URXISEL	= 0;		// RX interrupt when a char is in
	U2STAbits.OERR		= 0;		// clear overun error
	
	// U1BRG Register
	// ==============
	U2BRG = LOG_UBRG;				// Set the baud rate for 115,200
	
	// Initialize the Interrupt  
  	// ========================
	IPC7bits.U2RXIP   = 6;    		// Interrupt priority 6  
  	IFS1bits.U2RXIF   = 0;    		// Clear the interrupt flag
  	IEC1bits.U2RXIE   = 1;    		// Enable interrupts

	// Enable the port;
	U2MODEbits.UARTEN	= 1;		// Enable the port	
	U2STAbits.UTXEN		= 1;		// Enable TX
	
	IEC4bits.U2EIE 		= 0;
	
}


void gsRead(unsigned char* gsChunk){
	// fix the data length so if the interrupt adds data
	// during execution of this block, it will be read
	// until the next gsRead
	unsigned int tmpLen = getLength(uartBufferIn), i=0;
	
	// if the buffer has more data than the max size, set it to max,
	// otherwise set it to the length
	gsChunk[0] =  (tmpLen > MAXSEND -1)? MAXSEND -1: tmpLen;
	
	// read the data 
	for(i = 1; i <= gsChunk[0]; i += 1 )
	{
		gsChunk[i] = readFront(uartBufferIn);
	}
	
	gsChunk[MAXSEND+1] = 1;
}


void prepareTelemetryMavlink( unsigned char* dataOut){
 
	// Generic message container used to pack the messages
	mavlink_message_t msg;
	
	// Generic buffer used to hold data to be streamed via serial port
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
	// Cycles from 1 to 10 to decide which 
	// message's turn is to be sent
	static uint8_t sampleTelemetry = 1;
	
	// Low frequency reporting flags (used inside particular cases)
	static uint8_t sampleBiases = 1;
	
	// Contains the total bytes to send via the serial port
	uint8_t bytes2Send = 0;
	
	memset(&msg,0,sizeof(mavlink_message_t));

	
	switch (sampleTelemetry){
		case 1: // GPS or WPS/PID pending requests
			if ((mlPending.pid > 0) || (mlPending.wp > 0)){
				if (mlPending.pid>0){
					// send PID
					mavlink_msg_pid_pack(SLUGS_SYSTEMID, 
															 SLUGS_COMPID,
															 &msg,
															 GS_SYSTEMID,
															 mlPending.pidIdx, 
															 mlPidValues.P[mlPending.pidIdx], 
															 mlPidValues.I[mlPending.pidIdx],
															 mlPidValues.D[mlPending.pidIdx]);
					
					mlPending.pid = 0;
					mlPending.pidIdx = 0;
					
				} else {
					// Send WP
					mavlink_msg_waypoint_pack(SLUGS_SYSTEMID, 
																		SLUGS_COMPID,
															 			&msg,
															 			GS_SYSTEMID,
															 			GS_COMPID,
															 			mlPending.wpsIdx, 
															 			MAV_FRAME_GLOBAL,
															 			mlWpValues.type[mlPending.wpsIdx], 
															 			(float)mlWpValues.orbit[mlPending.wpsIdx],
																		0,// always clockwise
																		0.0,// Not used
																		0.0,// Not used
																		0,// Nor used
																		mlWpValues.lon[mlPending.wpsIdx],
																		mlWpValues.lat[mlPending.wpsIdx],
																		mlWpValues.alt[mlPending.wpsIdx],
																		0.0, //not used
																		1); // always autocontinue
					
					mlPending.wp = 0;
					mlPending.wpsIdx = 0;
				}
			} else {
			// Pack the GPS message
				mavlink_msg_gps_raw_pack(SLUGS_SYSTEMID, 
																	SLUGS_COMPID, 
																	&msg, 
																	0, 
																	mlGpsData.fix_type, 
																	mlGpsData.lat, 
																	mlGpsData.lon, 
																	mlGpsData.alt, 
																	mlGpsData.eph, 
																	0.0, 
																	mlGpsData.v, 
																	mlGpsData.hdg);
			} 
			// Copy the message to the send buffer
			bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
		break;
		case 2: // LOAD and PWM
			mavlink_msg_cpu_load_pack( SLUGS_SYSTEMID, 
																 SLUGS_COMPID, 
																 &msg, 
																 mlCpuLoadData.sensLoad, 
																 mlCpuLoadData.ctrlLoad, 
																 mlCpuLoadData.batVolt);	  
			// Copy the message to the send buffer
			bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);

			memset(&msg,0,sizeof(mavlink_message_t));
			
			mavlink_msg_pwm_commands_pack( SLUGS_SYSTEMID, 
																 		 SLUGS_COMPID, 
																 		 &msg, 
																 		 mlPwmCommands.dt_c, 
																 		 mlPwmCommands.dla_c, 
																 		 mlPwmCommands.dra_c, 
																 		 mlPwmCommands.dr_c, 
																 		 mlPwmCommands.dle_c, 
																 		 mlPwmCommands.dre_c, 
																 		 mlPwmCommands.dlf_c, 
																 		 mlPwmCommands.drf_c, 
																 		 mlPwmCommands.aux1, 
																 		 mlPwmCommands.aux2);
			// Copy the message to the send buffer
			bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);

		break;
		
		case 3: // XYZ and Heartbeat
			// Pack the Heartbeat message
			mavlink_msg_heartbeat_pack(SLUGS_SYSTEMID, 
																 SLUGS_COMPID, 
																 &msg, 
																 MAV_FIXED_WING, 
																 MAV_AUTOPILOT_SLUGS);
			// Copy the message to the send buffer
			bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
			
			memset(&msg,0,sizeof(mavlink_message_t));
			
			mavlink_msg_local_position_pack( SLUGS_SYSTEMID, 
																 			 SLUGS_COMPID, 
																 			 &msg, 
																 			 mlLocalPositionData.usec, 
																 			 mlLocalPositionData.x, 
																 			 mlLocalPositionData.y, 
																 			 mlLocalPositionData.z, 
																 			 mlLocalPositionData.vx, 
																 			 mlLocalPositionData.vy, 
																 			 mlLocalPositionData.vz);
			// Copy the message to the send buffer
			bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
		
		break;
		
		case 4: // Dynamic and Pilot Console
			mavlink_msg_air_data_pack( SLUGS_SYSTEMID, 
																 SLUGS_COMPID, 
																 &msg, 
																 mlAirData.dynamicPressure, 
																 mlAirData.staticPressure, 
																 mlAirData.temperature);
			// Copy the message to the send buffer
			bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);

			memset(&msg,0,sizeof(mavlink_message_t));
			
			mavlink_msg_pilot_console_pack( SLUGS_SYSTEMID, 
																 			SLUGS_COMPID, 
																 			&msg, 
																 			mlPilotConsoleData.dt, 
																 			mlPilotConsoleData.dla, 
																 			mlPilotConsoleData.dra, 
																 			mlPilotConsoleData.dr, 
																 			mlPilotConsoleData.de);
			// Copy the message to the send buffer
			bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
			
			if (mlPending.pt == 1){
				// clear the message
				memset(&msg,0,sizeof(mavlink_message_t));
			
				mavlink_msg_ctrl_srfc_pt_pack( SLUGS_SYSTEMID, 
																 			SLUGS_COMPID, 
																 			&msg, 
																 			GS_SYSTEMID, 
																 			mlPassthrough.bitfieldPt);
				// Copy the message to the send buffer
				bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
			
				// Clear the flag
				mlPending.pt = 0;
			}
				
		break;
		
		case 5: // Biases or Data Log depending on timing and midLvlCmds if requested
			if (sampleBiases == 1){
				mavlink_msg_sensor_bias_pack( SLUGS_SYSTEMID, 
																 		SLUGS_COMPID, 
																 		&msg, 
																 		mlSensorBiasData.axBias, 
																 		mlSensorBiasData.ayBias, 
																 		mlSensorBiasData.azBias, 
																 		mlSensorBiasData.gxBias, 
																 		mlSensorBiasData.gyBias, 
																 		mlSensorBiasData.gzBias);			
			} else {
				mavlink_msg_data_log_pack( SLUGS_SYSTEMID, 
																 	 SLUGS_COMPID, 
																 	 &msg, 
																 	 mlDataLog.fl_1, 
																 	 mlDataLog.fl_2, 
																 	 mlDataLog.fl_3, 
																 	 mlDataLog.fl_4, 
																 	 mlDataLog.fl_5, 
																 	 mlDataLog.fl_6);
			}
			
			// Copy the message to the send buffer
			bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);

			sampleBiases = (sampleBiases>10)? 1 : sampleBiases + 1;
			
			// if there is a pending request for the Mid Level Commands
			if (mlPending.midLvlCmds == 1){
				// clear the msg
				memset(&msg,0,sizeof(mavlink_message_t));
				
				mavlink_msg_mid_lvl_cmds_pack( SLUGS_SYSTEMID, 
																   		 SLUGS_COMPID, 
																   		 &msg, 
																   		 GS_SYSTEMID,
															 				 mlMidLevelCommands.hCommand,  
															 				 mlMidLevelCommands.uCommand,  
															 				 mlMidLevelCommands.rCommand);  
				
				// Copy the message to the send buffer
				bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
				
				// clear the flag
				mlPending.midLvlCmds = 0;
			}
		break;
		
		case 6: // Diagnostic and Raw Pressure
			mavlink_msg_diagnostic_pack( SLUGS_SYSTEMID, 
																   SLUGS_COMPID, 
																   &msg, 
																   mlDiagnosticData.diagFl1,  
																   mlDiagnosticData.diagFl2,  
																   mlDiagnosticData.diagFl3,  
																   mlDiagnosticData.diagSh1,  
																   mlDiagnosticData.diagSh2,  
																   mlDiagnosticData.diagSh3);
			// Copy the message to the send buffer
			bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
			
			memset(&msg,0,sizeof(mavlink_message_t));

			mavlink_msg_raw_pressure_pack( SLUGS_SYSTEMID, 
																		 SLUGS_COMPID, 
																		 &msg, 
																		 mlRawPressureData.usec,  
																		 mlRawPressureData.press_abs,  
																		 mlRawPressureData.press_diff1,  
																		 mlRawPressureData.press_diff2,
																		 mlRawPressureData.temperature);
			// Copy the message to the send buffer
			bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);	
		break;
		
		case 7: // Raw Sensor Data
			mavlink_msg_raw_imu_pack( SLUGS_SYSTEMID, 
																SLUGS_COMPID, 
																&msg, 
																mlRawImuData.usec,  
																mlRawImuData.xacc,  
																mlRawImuData.yacc,  
																mlRawImuData.zacc,  
																mlRawImuData.xgyro,  
																mlRawImuData.ygyro,  
																mlRawImuData.zgyro,  
																mlRawImuData.xmag,  
																mlRawImuData.ymag,  
																mlRawImuData.zmag);
			// Copy the message to the send buffer
			bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
			
			if (mlPending.mode == 1){
				// clear the msg
				memset(&msg,0,sizeof(mavlink_message_t));
				
				mavlink_msg_set_mode_pack( SLUGS_SYSTEMID, 
																   SLUGS_COMPID, 
																   &msg, 
																   GS_SYSTEMID,
															 		 mlApMode.mode);  
				
				// Copy the message to the send buffer
				bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
				
				// clear the flag
				mlPending.mode = 0;
			}
		break;
		
		case 8: // Aknowledge, GPS Date time || Mid Lvl Commands Report, Ping Request
			if (mlActionAck.action != SLUGS_ACTION_NONE){
				mavlink_msg_action_ack_pack( SLUGS_SYSTEMID, 
																     SLUGS_COMPID, 
																     &msg, 
																     mlActionAck.action,
															 		   mlActionAck.result);  
				
				// Copy the message to the send buffer
				bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
				
				mlActionAck.action = SLUGS_ACTION_NONE;
				
				// clear the msg
				memset(&msg,0,sizeof(mavlink_message_t));
			}
			
			mavlink_msg_gps_date_time_pack( SLUGS_SYSTEMID, 
																   		SLUGS_COMPID, 
																   		&msg, 
																   		mlGpsDateTime.year,
																   		mlGpsDateTime.month,
																   		mlGpsDateTime.day,
																   		mlGpsDateTime.hour,
																   		mlGpsDateTime.min,
																   		mlGpsDateTime.sec,
																   		mlGpsDateTime.visSat);  
				
				// Copy the message to the send buffer
				bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);				
		
				if (mlPending.ping == 1){
					// clear the msg
					memset(&msg,0,sizeof(mavlink_message_t));			
					
					mavlink_msg_ping_pack( SLUGS_SYSTEMID, 
																  SLUGS_COMPID, 
																  &msg, 
																  mlPing.seq,
																  SLUGS_SYSTEMID, 
																  SLUGS_COMPID,
																  mlAttitudeData.usec);	
					
					// Copy the message to the send buffer
					bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);									
				}	
		break;
		
		case 9: // Navigation
			mavlink_msg_slugs_navigation_pack(SLUGS_SYSTEMID, 
																  			SLUGS_COMPID, 
																  			&msg, 
																  			mlNavigation.u_m,
																  			mlNavigation.phi_c,
																  			mlNavigation.theta_c,
																  			mlNavigation.psiDot_c,
																  			mlNavigation.ay_body,
																  			mlNavigation.totalDist,
																  			mlNavigation.dist2Go,
																  			mlNavigation.fromWP,
																  			mlNavigation.toWP);	
					
			// Copy the message to the send buffer
			bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);		
		break;
		
		case 10: // Filtered Data
			mavlink_msg_filtered_data_pack( SLUGS_SYSTEMID, 
																   		SLUGS_COMPID, 
																   		&msg, 
																   		mlFilteredData.aX,
																   		mlFilteredData.aY,
																   		mlFilteredData.aZ,
																   		mlFilteredData.gX,
																   		mlFilteredData.gY,
																   		mlFilteredData.gZ,
																   		mlFilteredData.mX,
																   		mlFilteredData.mY,
																   		mlFilteredData.mZ);	
					
			// Copy the message to the send buffer
			bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
		break;
		
	} // Switch
	
	memset(&msg,0,sizeof(mavlink_message_t));
	
	// if the boot message is set then transmit it urgently
	if (mlBoot.version != 0){
		mavlink_msg_boot_pack(SLUGS_SYSTEMID, 
													SLUGS_COMPID, 
													&msg, 
													mlBoot.version);
											 
		// reset the boot message
		mlBoot.version = 0;	
	} 
	
	
	// Copy the message to the send buffer	
	bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);

	memset(&msg,0,sizeof(mavlink_message_t));
	
	mavlink_msg_attitude_pack( SLUGS_SYSTEMID, 
														 SLUGS_COMPID, 
														 &msg, 
														 mlAttitudeRotated.usec,  
														 mlAttitudeRotated.roll,  
														 mlAttitudeRotated.pitch,  
														 mlAttitudeRotated.yaw,  
														 mlAttitudeRotated.rollspeed,  
														 mlAttitudeRotated.pitchspeed,  
														 mlAttitudeRotated.yawspeed);
	// Copy the message to the send buffer	
	bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
	 
	// Put the length of the message in the first byte of the outgoing array
	*dataOut = bytes2Send;
	
	// increment/overflow the samplePeriod counter
	// configured for 10 Hz in non vital messages
	sampleTelemetry = (sampleTelemetry >= 10)? 1: sampleTelemetry + 1;

}


void lowRateTelemetryMavlink(unsigned char* dataOut){
	// Generic message container used to pack the messages
	mavlink_message_t msg;
	
	// Generic buffer used to hold data to be streamed via serial port
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
	// Cycles from 1 to 10 to decide which 
	// message's turn is to be sent
	static uint8_t sampleTelemetry = 1;
	
	// Low frequency reporting flags (used inside particular cases)
	static uint8_t sampleBiases = 1;
	
	// Contains the total bytes to send via the serial port
	uint8_t bytes2Send = 0;

	// Scheduling varialbes. There are two types, the sw_ prefix means that it is used as a 
	// selector inside the swithces and ct_ is a counter that decides when it is time to 
	// transmit		
	static uint8_t sw_50hz = 0;

	static uint8_t sw_10hz = 1;
	static uint8_t ct_10hz = 1;
	volatile uint8_t sw_10hzod = 0;

	static uint8_t sw_5hz = 1;
	static uint8_t ct_5hz = 1;
	volatile uint8_t sw_5hzod = 0;
	
	memset(&msg,0,sizeof(mavlink_message_t));
	
	if (sw_50hz){
		
		if (ct_10hz == 5){
			
			memset(&msg,0,sizeof(mavlink_message_t));
			
			switch (sw_10hz){
				case 1: // GPS
					mavlink_msg_gps_raw_encode(SLUGS_SYSTEMID, 
																	 	 SLUGS_COMPID, 
																		 &msg,  
																		 &mlGpsData);
				break;
				
				case 2: // PWM Commands
					mavlink_msg_pwm_commands_encode( SLUGS_SYSTEMID, 
																 		 		 	 SLUGS_COMPID, 
																 		 		 	 &msg, 
																 		 		 	 &mlPwmCommands);
				break;
				
				case 3: // XYZ - Local Position
					mavlink_msg_local_position_encode( SLUGS_SYSTEMID, 
																 			 			 SLUGS_COMPID, 
																 			 			 &msg, 
																 			 			 &mlLocalPositionData);
				break;
				
				case 4:	// Air Data
					mavlink_msg_air_data_encode( SLUGS_SYSTEMID, 
																 			 SLUGS_COMPID, 
																 			 &msg, 
																 			 &mlAirData);
				break;
	
				case 5: // Pilot Console
					mavlink_msg_pilot_console_encode( SLUGS_SYSTEMID, 
																 						SLUGS_COMPID, 
																 						&msg, 
																 						&mlPilotConsoleData);
				break;	
	
				case 6: // Diagnostic
					mavlink_msg_diagnostic_encode( SLUGS_SYSTEMID, 
																   			 SLUGS_COMPID, 
																   			 &msg, 
																   			 &mlDiagnosticData);
				break;
	
				case 7: // Raw Pressure
					mavlink_msg_raw_pressure_encode( SLUGS_SYSTEMID, 
																		 			 SLUGS_COMPID, 
																					 &msg, 
																					 &mlRawPressureData);
				break;
	
				case 8: // Raw IMU
					mavlink_msg_raw_imu_encode( SLUGS_SYSTEMID, 
																			SLUGS_COMPID, 
																			&msg, 
																			&mlRawImuData);
				break;
	
				case 9: // Navigation
					mavlink_msg_slugs_navigation_encode(SLUGS_SYSTEMID, 
																  						SLUGS_COMPID, 
																  						&msg, 
																  						&mlNavigation);
				break;
	
				case 10: // Filtered Data
					mavlink_msg_filtered_data_encode( SLUGS_SYSTEMID, 
																   					SLUGS_COMPID, 
																   					&msg, 
																   					&mlFilteredData);
				break;
	
			}// switch sw_10hz
			
			// Copy the message to the send buffer
			bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);

			
			sw_10hz = (sw_10hz>=10)? 1 : sw_10hz + 1;
			ct_10hz = 1;
		}else {
			
			// sorted from lowes to highest priority
			sw_10hzod = (mlPending.ping> 0)? 1 : sw_10hzod;
			sw_10hzod = (mlPending.pid > 0)? 2 : sw_10hzod;
			sw_10hzod = (mlPending.wp >  0)? 3 : sw_10hzod;
			sw_10hzod = (mlPending.midLvlCmds> 0)? 4 : sw_10hzod;
			sw_10hzod = (mlPending.wpTransaction > 0)? 5: sw_10hzod;

			switch (sw_10hzod){
				case 1: // Ping
					memset(&msg,0,sizeof(mavlink_message_t));			
					
					mavlink_msg_ping_pack(  SLUGS_SYSTEMID, 
																  SLUGS_COMPID, 
																  &msg, 
																  mlPing.seq,
																  SLUGS_SYSTEMID, 
																  SLUGS_COMPID,
																  mlAttitudeData.usec);	
					
					// Copy the message to the send buffer
					bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);		
					
					mlPending.ping = 0;
				break;
				
				case 2: // PID
					memset(&msg,0,sizeof(mavlink_message_t));		
					
					// send PID
					mavlink_msg_pid_pack(SLUGS_SYSTEMID, 
															 SLUGS_COMPID,
															 &msg,
															 GS_SYSTEMID,
															 mlPidValues.P[mlPending.pidIdx], 
															 mlPidValues.I[mlPending.pidIdx],
															 mlPidValues.D[mlPending.pidIdx],
															 mlPending.pidIdx);

					// Copy the message to the send buffer
					bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);		
										
					mlPending.pid = 0;
					mlPending.pidIdx = 0;				
				break;
				
				case 3: // Waypoint
					memset(&msg,0,sizeof(mavlink_message_t));		
					
					// Send WP
					mavlink_msg_waypoint_pack(SLUGS_SYSTEMID, 
																		SLUGS_COMPID,
															 			&msg,
															 			GS_SYSTEMID,
															 			GS_COMPID,
															 			mlPending.wpsIdx, 
															 			MAV_FRAME_GLOBAL,
															 			mlWpValues.type[mlPending.wpsIdx], 
															 			(float)mlWpValues.orbit[mlPending.wpsIdx],
																		0,// always clockwise
																		0.0,// Not used
																		0.0,// Not used
																		0,// Nor used
																		mlWpValues.lon[mlPending.wpsIdx],
																		mlWpValues.lat[mlPending.wpsIdx],
																		mlWpValues.alt[mlPending.wpsIdx],
																		0.0, //not used
																		1); // always autocontinue
																		
				  // Copy the message to the send buffer
					bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);		

					mlPending.wp = 0;
					mlPending.wpsIdx = 0;
				break;
				
				case 4: // Mid-level Commands
					// clear the msg
					memset(&msg,0,sizeof(mavlink_message_t));
				
					mavlink_msg_mid_lvl_cmds_pack( SLUGS_SYSTEMID, 
																	   		 SLUGS_COMPID, 
																	   		 &msg, 
																	   		 GS_SYSTEMID,
																 				 mlMidLevelCommands.hCommand,  
																 				 mlMidLevelCommands.uCommand,  
																 				 mlMidLevelCommands.rCommand);  
				
					// Copy the message to the send buffer
					bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
				
					// clear the flag
					mlPending.midLvlCmds = 0;
				break;			
				
				case 5: // Wp Protocol state machine

					switch (mlPending.wpProtState){
						
						case WP_PROT_LIST_REQUESTED:
							// clear the msg
							memset(&msg,0,sizeof(mavlink_message_t));
						
							mavlink_msg_waypoint_count_pack( SLUGS_SYSTEMID, 
																			   		 	 SLUGS_COMPID, 
																			   		 	 &msg, 
																			   		 	 GS_SYSTEMID,
																			   		 	 GS_COMPID,	
																			   		 	 mlWpValues.wpCount);  
						
							// Copy the message to the send buffer
							bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
						
							// Change the state machine state
							mlPending.wpProtState = WP_PROT_NUM_SENT;	
							
							// Reset the timeout
							mlPending.wpTimeOut = 0;
						break;
						
						case WP_PROT_GETTING_WP_IDLE:	
							if (mlPending.wpCurrentWpInTransaction< mlPending.wpTotalWps){
								// clear the msg
								memset(&msg,0,sizeof(mavlink_message_t));
							
								mavlink_msg_waypoint_request_pack( SLUGS_SYSTEMID, 
																				   		 	   SLUGS_COMPID, 
																				   		 	 	 &msg, 
																				   		 	 	 GS_SYSTEMID,
																				   		 	 	 GS_COMPID,
																				   		 	   mlPending.wpCurrentWpInTransaction++);  
							
								// Copy the message to the send buffer
								bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
							
								// Change the state machine state
								mlPending.wpProtState = WP_PROT_RX_WP;	
								
							} else {
								// clear the msg
								memset(&msg,0,sizeof(mavlink_message_t));
							
								mavlink_msg_waypoint_ack_pack( SLUGS_SYSTEMID, 
																				   		 SLUGS_COMPID, 
																				   		 &msg,
																				   		 GS_SYSTEMID,
																				   		 GS_COMPID,
																					   	 0);  // 0 is success
							
								// Copy the message to the send buffer
								bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
							
								// End the transaction
								mlPending.wpTransaction = 0;
								mlPending.wpProtState = WP_PROT_IDLE;
								mlPending.wpCurrentWpInTransaction = 0;
								mlPending.wpTimeOut = 0;
								mlPending.wpTotalWps = 0;
							}
							
							// Reset the timeout
							mlPending.wpTimeOut = 0;
						break;
						
						case WP_PROT_TX_WP:
							memset(&msg,0,sizeof(mavlink_message_t));		
							
							// Send WP
							mavlink_msg_waypoint_pack(SLUGS_SYSTEMID, 
																				SLUGS_COMPID,
																	 			&msg,
																	 			GS_SYSTEMID,
																	 			GS_COMPID,
																	 			mlPending.wpCurrentWpInTransaction, 
																	 			MAV_FRAME_GLOBAL,
																	 			mlWpValues.type[mlPending.wpsIdx], 
																	 			(float)mlWpValues.orbit[mlPending.wpCurrentWpInTransaction],
																	 			0,// always clockwise
																				0.0,// Not used
																				0.0,// Not used
																				0,// Nor used
																				mlWpValues.lon[mlPending.wpCurrentWpInTransaction],
																				mlWpValues.lat[mlPending.wpCurrentWpInTransaction],
																				mlWpValues.alt[mlPending.wpCurrentWpInTransaction],
																				0.0, //not used
																				1); // always autocontinue
																				
						  // Copy the message to the send buffer
							bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);		
		
							// Switch the state waiting for the next request
							// Change the state machine state
							mlPending.wpProtState = WP_PROT_SENDING_WP_IDLE;	
							
							// Reset the timeout
							mlPending.wpTimeOut = 0;
						break;
					} // switch wpProtState
					
					mlPending.wpTimeOut++;
										
					// if Timed out reset the state machine and send an error
					if (mlPending.wpTimeOut > PROTOCOL_TIMEOUT_TICKS){
						memset(&msg,0,sizeof(mavlink_message_t));
							
						mavlink_msg_waypoint_ack_pack( SLUGS_SYSTEMID, 
																		   		 SLUGS_COMPID, 
																		   		 &msg,
																		   		 GS_SYSTEMID,
																				   GS_COMPID, 
																			   	 1);  // 1 is failure
						
						// Copy the message to the send buffer
						bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
						
						mlPending.wpTransaction = 0;
						mlPending.wpProtState = WP_PROT_IDLE;
						mlPending.wpCurrentWpInTransaction = 0;
						mlPending.wpTimeOut = 0;
						mlPending.wpTotalWps = 0;
					}
				break;
			}
			
			ct_10hz++;
		}// if ct_10hz
	} else {
	
		mavlink_msg_attitude_encode( SLUGS_SYSTEMID, 
														 		 SLUGS_COMPID, 
														 		 &msg, 
														 		 &mlAttitudeRotated);
														 		 
		// Copy the message to the send buffer	
		bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
		
		if (ct_5hz == 10){
			
			memset(&msg,0,sizeof(mavlink_message_t));
			
			switch (sw_5hz){
				case 1:
					mavlink_msg_cpu_load_encode( SLUGS_SYSTEMID, 
																 			 SLUGS_COMPID, 
																 			 &msg, 
																 			 &mlCpuLoadData);	
				break;

				case 2:
					mavlink_msg_data_log_encode( SLUGS_SYSTEMID, 
																 	 		 SLUGS_COMPID, 
																 	 		 &msg, 
																 	 		 &mlDataLog);									
				break;
				
				case 3:
					mavlink_msg_gps_date_time_encode( SLUGS_SYSTEMID, 
																   					SLUGS_COMPID, 
																   					&msg, 
																   					&mlGpsDateTime); 
				break;
				
				case 4:
					mavlink_msg_sensor_bias_encode( SLUGS_SYSTEMID, 
																 					SLUGS_COMPID, 
																 					&msg, 
																 					&mlSensorBiasData);	
				break;
	
				case 5:
					mavlink_msg_heartbeat_pack(SLUGS_SYSTEMID, 
																 		 SLUGS_COMPID, 
																 		 &msg, 
																 		 MAV_FIXED_WING, 
																 		 MAV_AUTOPILOT_SLUGS);
				break;
			}// switch sw_ 5hz
		
			// Copy the message to the send buffer
			bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
	
			sw_5hz = (sw_5hz>=5)? 1 : sw_5hz + 1;
			ct_5hz = 1;
		} else {
			
			// sorted from lowes to highest priority
			sw_5hzod = (mlPending.pt> 0)? 1 : sw_5hzod;
			sw_5hzod = (mlPending.mode > 0)? 2 : sw_5hzod;
			sw_5hzod = (mlActionAck.action != SLUGS_ACTION_NONE)? 3 : sw_5hzod;
			sw_5hzod = (mlBoot.version> 0)? 4 : sw_5hzod;

			switch (sw_5hzod){
				case 1: // Passthrough
					// clear the message
					memset(&msg,0,sizeof(mavlink_message_t));
			
					mavlink_msg_ctrl_srfc_pt_pack( SLUGS_SYSTEMID, 
																	 			SLUGS_COMPID, 
																	 			&msg, 
																	 			GS_SYSTEMID, 
																	 			mlPassthrough.bitfieldPt);
					// Copy the message to the send buffer
					bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
			
					// Clear the flag
					mlPending.pt = 0;
				break;
				
				case 2: // Mode Report
					// clear the msg
					memset(&msg,0,sizeof(mavlink_message_t));
				
					mavlink_msg_set_mode_pack( SLUGS_SYSTEMID, 
																	   SLUGS_COMPID, 
																	   &msg, 
																	   GS_SYSTEMID,
																 		 mlApMode.mode);  
				
					// Copy the message to the send buffer
					bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
				
					// clear the flag
					mlPending.mode = 0;
				break;
				
				case 3:			 // Acknowledge				
					// clear the msg
					memset(&msg,0,sizeof(mavlink_message_t));
					
					mavlink_msg_action_ack_pack( SLUGS_SYSTEMID, 
																     SLUGS_COMPID, 
																     &msg, 
																     mlActionAck.action,
															 		   mlActionAck.result);  
				
					// Copy the message to the send buffer
					bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
				
					mlActionAck.action = SLUGS_ACTION_NONE;
				break;
				
				case 4: // Boot
					// clear the msg
					memset(&msg,0,sizeof(mavlink_message_t));
					
					mavlink_msg_boot_pack(SLUGS_SYSTEMID, 
													SLUGS_COMPID, 
													&msg, 
													mlBoot.version);
					
					// Copy the message to the send buffer
					bytes2Send += mavlink_msg_to_send_buffer((dataOut+1+bytes2Send), &msg);
											 
					// reset the boot message
					mlBoot.version = 0;	
				break;			
			}
			ct_5hz++;
		}
	} // if 50 Hz
	
	// Put the length of the message in the first byte of the outgoing array
	*dataOut = bytes2Send;
	
	// Get ready for the next Tx
	sw_50hz= !sw_50hz;
}

 void protDecodeMavlink (uint8_t* dataIn){
	
	uint8_t i, indx, writeSuccess, commChannel = dataIn[MAXSEND+1];
	uint16_t indexOffset;
	uint32_t temp;
	
	tFloatToChar tempFloat;

	static int16_t packet_drops = 0;
	mavlink_message_t msg;
	mavlink_status_t status;
	

	for(i = 1; i <= dataIn[0]; i++ ){
		// Try to get a new message
		if(mavlink_parse_char(commChannel, dataIn[i], &msg, &status)) {
	
			// Handle message
			switch(msg.msgid){
				case MAVLINK_MSG_ID_HEARTBEAT:
					mavlink_msg_heartbeat_decode(&msg, &mlHeartbeat);
				break;
				
				case MAVLINK_MSG_ID_GPS_RAW:
					mavlink_msg_gps_raw_decode(&msg, &mlGpsData);
				break;
				
				case MAVLINK_MSG_ID_CPU_LOAD:
					mavlink_msg_cpu_load_decode(&msg, &mlCpuLoadData);
				break;
				
				case MAVLINK_MSG_ID_LOCAL_POSITION:
					mavlink_msg_local_position_decode(&msg, &mlLocalPositionData);
				break;
				
				case MAVLINK_MSG_ID_AIR_DATA:
					mavlink_msg_air_data_decode(&msg, &mlAirData);
				break;
				
				case MAVLINK_MSG_ID_BOOT:
					temp = mavlink_msg_boot_get_version(&msg);
					
					if (temp != 0 && mlBoot.version != 0){
						mlBoot.version += temp;
					} else if (mlBoot.version == 0) {
						mavlink_msg_boot_decode(&msg, &mlBoot);	
					}
				break;
				
				case MAVLINK_MSG_ID_SENSOR_BIAS:
					mavlink_msg_sensor_bias_decode(&msg, &mlSensorBiasData);
				break;
				
				case MAVLINK_MSG_ID_DIAGNOSTIC:
					mavlink_msg_diagnostic_decode(&msg, &mlDiagnosticData);
				break;
				
				case MAVLINK_MSG_ID_PILOT_CONSOLE:
					mavlink_msg_pilot_console_decode(&msg, &mlPilotConsoleData);
				break;
				
				case MAVLINK_MSG_ID_FILTERED_DATA:
					mavlink_msg_filtered_data_decode(&msg, &mlFilteredData);
				break;
				
				case MAVLINK_MSG_ID_ATTITUDE:
					mavlink_msg_attitude_decode(&msg, &mlAttitudeData);
				break;
				
				case MAVLINK_MSG_ID_RAW_IMU:
					mavlink_msg_raw_imu_decode(&msg, &mlRawImuData);
				break;

				case MAVLINK_MSG_ID_RAW_PRESSURE:
					mavlink_msg_raw_pressure_decode(&msg, &mlRawPressureData);
				break;
				
				case MAVLINK_MSG_ID_GPS_DATE_TIME:
					mavlink_msg_gps_date_time_decode(&msg, &mlGpsDateTime);
				break;
				
				//  End of Sensor MCU exclusive Messages
				// =====================================
				
				case MAVLINK_MSG_ID_SET_MODE:
					mavlink_msg_set_mode_decode(&msg, &mlApMode);
					
					// Report the Change
					mlActionAck.action = SLUGS_ACTION_MODE_CHANGE;
					mlActionAck.result = mlApMode.mode;	

				break;

 				case MAVLINK_MSG_ID_MID_LVL_CMDS:
					mavlink_msg_mid_lvl_cmds_decode(&msg, &mlMidLevelCommands);
					
					// Report the Change
					mlActionAck.action = SLUGS_ACTION_MLC_CHANGE;
					mlActionAck.result = SLUGS_ACTION_SUCCESS;	

				break;				
				case MAVLINK_MSG_ID_PID:
					writeSuccess = 0;
					
					mavlink_msg_pid_decode(&msg, &mlSinglePid);
					
					indx = mlSinglePid.idx;
					
					// store the values in the temp array
					mlPidValues.P[indx] = mlSinglePid.pVal;
					mlPidValues.I[indx] = mlSinglePid.iVal;
					mlPidValues.D[indx] = mlSinglePid.dVal;
				
					// Save the data to the EEPROM
					indexOffset = indx*6;
					
					tempFloat.flData = mlSinglePid.pVal;
					writeSuccess += DataEEWrite(tempFloat.shData[0], PID_OFFSET+indexOffset);
					writeSuccess += DataEEWrite(tempFloat.shData[1], PID_OFFSET+indexOffset+1);
					
					tempFloat.flData = mlSinglePid.iVal;
					writeSuccess += DataEEWrite(tempFloat.shData[0], PID_OFFSET+indexOffset+2);
					writeSuccess += DataEEWrite(tempFloat.shData[1], PID_OFFSET+indexOffset+3);
					
					tempFloat.flData = mlSinglePid.dVal;
					writeSuccess += DataEEWrite(tempFloat.shData[0], PID_OFFSET+indexOffset+4);
					writeSuccess += DataEEWrite(tempFloat.shData[1], PID_OFFSET+indexOffset+5);
				
					// Set the flag of Aknowledge for the AKN Message
					// if the write was successful
					if (writeSuccess==0){
						mlActionAck.action = SLUGS_ACTION_PID_CHANGE;
						mlActionAck.result = indx;	
					} else{
						mlActionAck.action = SLUGS_ACTION_EEPROM;
						mlActionAck.result = EEP_WRITE_FAIL;	
					}

				break;	
				
				case MAVLINK_MSG_ID_WAYPOINT_COUNT:
					if (!mlPending.wpTransaction && (mlPending.wpProtState == WP_PROT_IDLE)){
						
						mavlink_msg_waypoint_count_decode(&msg, &mlWpCount);
						
						// Start the transaction
						mlPending.wpTransaction = 1;
						
						// change the state	
						mlPending.wpProtState = WP_PROT_GETTING_WP_IDLE;
						
						// reset the rest of the state machine
						mlPending.wpTotalWps = mlWpCount.count;
						mlPending.wpCurrentWpInTransaction = 0;
						mlPending.wpTimeOut = 0;	
					}
				
				break;
				
				case MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST:
								
					// if there is no transaction going on
					if (!mlPending.wpTransaction && (mlPending.wpProtState == WP_PROT_IDLE)){
						// Start the transaction
						mlPending.wpTransaction = 1;
						
						// change the state
						mlPending.wpProtState = WP_PROT_LIST_REQUESTED;
						
						
						
						// reset the rest of the state machine
						mlPending.wpCurrentWpInTransaction = 0;
						mlPending.wpTimeOut = 0;	
					}
				break;
				
				case MAVLINK_MSG_ID_WAYPOINT_REQUEST:
					mavlink_msg_waypoint_request_decode(&msg, &mlWpRequest);
					
					if (mlPending.wpTransaction && (mlWpRequest.seq < mlWpValues.wpCount)){
						// change the state
						mlPending.wpProtState = WP_PROT_TX_WP;
						
						// reset the rest of the state machine
						mlPending.wpCurrentWpInTransaction = mlWpRequest.seq;
						mlPending.wpTimeOut = 0;	
					} else {
						// TODO: put here a report for a single WP, i.e. not inside a transaction
					}
				break;
				
				case MAVLINK_MSG_ID_WAYPOINT_ACK:
					mavlink_msg_waypoint_ack_decode(&msg, &mlWpAck);
					
					if(mlPending.wpTransaction){
						// End the transaction
						mlPending.wpTransaction = 0;
						
						// change the state
						mlPending.wpProtState = WP_PROT_IDLE;
						
						// reset the rest of the state machine
						mlPending.wpCurrentWpInTransaction = 0;
						mlPending.wpTimeOut = 0;	
					}
						
				break;
				
				case MAVLINK_MSG_ID_WAYPOINT:
					writeSuccess = 0;
					mavlink_msg_waypoint_decode(&msg, &mlSingleWp);
					
					if (mlPending.wpTransaction && (mlPending.wpProtState == WP_PROT_RX_WP)){
						mlPending.wpProtState = WP_PROT_GETTING_WP_IDLE;
						
					}
					

					
					indx = (uint8_t)mlSingleWp.seq;
					
					mlWpValues.lat[indx] 		= mlSingleWp.y;
					mlWpValues.lon[indx] 		= mlSingleWp.x;
					mlWpValues.alt[indx] 		= mlSingleWp.z;
					mlWpValues.type[indx] 	= mlSingleWp.action;
					mlWpValues.orbit[indx]	= (uint16_t)mlSingleWp.orbit;
					
					// Compute the adecuate index offset
					indexOffset = indx*8;
					
					// Save the data to the EEPROM
					tempFloat.flData = mlSingleWp.y;
					writeSuccess += DataEEWrite(tempFloat.shData[0], WPS_OFFSET+indexOffset);   
					writeSuccess += DataEEWrite(tempFloat.shData[1], WPS_OFFSET+indexOffset+1);
					
					tempFloat.flData = mlSingleWp.x; 
					writeSuccess += DataEEWrite(tempFloat.shData[0], WPS_OFFSET+indexOffset+2);      
					writeSuccess += DataEEWrite(tempFloat.shData[1], WPS_OFFSET+indexOffset+3);
					
					tempFloat.flData = mlSingleWp.z;       
					writeSuccess += DataEEWrite(tempFloat.shData[0], WPS_OFFSET+indexOffset+4);      
					writeSuccess += DataEEWrite(tempFloat.shData[1], WPS_OFFSET+indexOffset+5);
					
					
					writeSuccess += DataEEWrite((unsigned short)mlSingleWp.action, WPS_OFFSET+indexOffset+6);
					
					writeSuccess += DataEEWrite(mlSingleWp.orbit, WPS_OFFSET+indexOffset+7);          
					
					// Set the flag of Aknowledge for the AKN Message
					// if the write was successful
					if (writeSuccess==0){
						mlActionAck.action = SLUGS_ACTION_WP_CHANGE;
						mlActionAck.result = indx+1;	
					} else{
						mlActionAck.action = SLUGS_ACTION_EEPROM;
						mlActionAck.result = EEP_WRITE_FAIL;	
					}
					
					// Update the waypoint count
					mlWpValues.wpCount = 0;
					while ((int)(mlWpValues.lat[mlWpValues.wpCount]) != 0 && mlWpValues.wpCount< MAX_NUM_WPS-1 ){
						mlWpValues.wpCount++;
					}
				break;
				
				case MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL:
				
					writeSuccess = 0;
					
					// clear the WP values in memory;
					memset(&mlWpValues ,0, sizeof(mavlink_waypoint_values_t));	
					
					// erase the flash values in EEPROM emulation
					for (indx = 0; indx < MAX_NUM_WPS-1; indx++){
						// Compute the adecuate index offset
						indexOffset = indx*8;
						
						// Clear the data from the EEPROM
						tempFloat.flData = 0.0;
						writeSuccess += DataEEWrite(tempFloat.shData[0], WPS_OFFSET+indexOffset);   
						writeSuccess += DataEEWrite(tempFloat.shData[1], WPS_OFFSET+indexOffset+1);
						
						tempFloat.flData = 0.0; 
						writeSuccess += DataEEWrite(tempFloat.shData[0], WPS_OFFSET+indexOffset+2);      
						writeSuccess += DataEEWrite(tempFloat.shData[1], WPS_OFFSET+indexOffset+3);
						
						tempFloat.flData = 0.0;       
						writeSuccess += DataEEWrite(tempFloat.shData[0], WPS_OFFSET+indexOffset+4);      
						writeSuccess += DataEEWrite(tempFloat.shData[1], WPS_OFFSET+indexOffset+5);
						
						
						writeSuccess += DataEEWrite((unsigned short)0, WPS_OFFSET+indexOffset+6);
						
						writeSuccess += DataEEWrite((unsigned short)0, WPS_OFFSET+indexOffset+7); 
					}
					
					// Set the flag of Aknowledge for the AKN Message
					// if the write was successful
					if (writeSuccess==0){
						mlActionAck.action = SLUGS_ACTION_WP_CHANGE;
						mlActionAck.result = 0;	
					} else{
						mlActionAck.action = SLUGS_ACTION_EEPROM;
						mlActionAck.result = EEP_WRITE_FAIL;	
					}
					
					// Update the waypoint count
					mlWpValues.wpCount = 0;
					
					// Set the state machine ready to send the WP akn
					mlPending.wpCurrentWpInTransaction = 0;
					mlPending.wpTotalWps = 0;
					mlPending.wpTransaction = 1;
					mlPending.wpProtState = WP_PROT_GETTING_WP_IDLE;

				break;
				
				case MAVLINK_MSG_ID_CTRL_SRFC_PT:
					mavlink_msg_ctrl_srfc_pt_decode(&msg, &mlPassthrough);
					
					// Report the Change
					mlActionAck.action = SLUGS_ACTION_PT_CHANGE;
					mlActionAck.result = SLUGS_ACTION_SUCCESS;	
				break;
				
				case MAVLINK_MSG_ID_PING:
					mavlink_msg_ping_decode(&msg, &mlPing);
					
					mlPending.ping = 1;
				break;
				
				case MAVLINK_MSG_ID_SLUGS_ACTION:
					mavlink_msg_slugs_action_decode(&msg, &mlAction);
					
					switch (mlAction.actionId){
						case SLUGS_ACTION_PT_REPORT:
							mlPending.pt = 1;
						break;
						
						case SLUGS_ACTION_PID_REPORT:
							mlPending.pid = 1;
							mlPending.pidIdx = (uint8_t)mlAction.actionVal;
						break;
						
						case SLUGS_ACTION_WP_REPORT:
							mlPending.wp = 1;
							mlPending.wpsIdx = (uint8_t)mlAction.actionVal;
						break;
						
						case SLUGS_ACTION_MLC_REPORT:
							mlPending.midLvlCmds = 1;
						break;	
						
						case SLUGS_ACTION_MODE_REPORT:
							mlPending.mode = 1;
						break;						
					}// switch
				break;
			}	// switch	
		} // if
	}// for  
}


void copyBufferToDMA1 (unsigned char size){
	unsigned char i;
	for(  i = 0; i < size; i += 1 )
	{
		BufferB[i] = (unsigned int) readFront(uartBufferOut);
	}
}

void send2GS(unsigned char* protData){
	unsigned int bufLen,i;
		
	// add the data to the circular buffer
	for(i = 1; i <= protData[0]; i += 1 )
	{
		writeBack(uartBufferOut, protData[i] );
	}
	
	// get the Length of the logBuffer
	bufLen = getLength(uartBufferOut);
	

	// if the interrupt catched up with the circularBuffer
	// and new data was added then turn on the DMA 
	if(!(DMA1CONbits.CHEN) && (bufLen>0)){
		// Configure the bytes to send
		DMA1CNT =  bufLen<= (MAXSEND-1)? bufLen-1: MAXSEND-1;	
		// copy the buffer to the DMA channel outgoing buffer
		copyBufferToDMA1((unsigned char) DMA1CNT+1);
		// Enable the DMA
		DMA1CONbits.CHEN = 1;
		// Init the transmission
		DMA1REQbits.FORCE = 1;
	}
	
}

// Interrupts

void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void)
{
    // Clear the DMA1 Interrupt Flag;
    IFS0bits.DMA1IF  = 0;		
}

void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void){
 
	// Read the buffer while it has data
	// and add it to the circular buffer
	while(U2STAbits.URXDA == 1){
		writeBack(uartBufferIn, (unsigned char)U2RXREG);
	}
	
	// If there was an overun error clear it and continue
	if (U2STAbits.OERR == 1){
		U2STAbits.OERR = 0;
	}
	
	// clear the interrupt
	IFS1bits.U2RXIF = 0;
	
}


void __attribute__ ((interrupt, no_auto_psv)) _U1ErrInterrupt(void)
{
	IFS4bits.U2EIF = 0; // Clear the UART2 Error Interrupt Flag
}


void sendQGCDebugMessage (const char* dbgMessage, char severity){
		mavlink_message_t msg;
		unsigned char strMessage[50];
		unsigned char packedMessage [100];
		unsigned char bytes2Send = 0;
		char sizeMessage = (sizeof(dbgMessage)>50) ? 50 : sizeof(dbgMessage);
		
		memset(strMessage,0,50);
		memcpy(strMessage, dbgMessage,sizeMessage);
		
		mavlink_msg_statustext_pack (SLUGS_SYSTEMID, 
																 SLUGS_COMPID, 
																 &msg, 
																 severity,
																 strMessage);
																 
		bytes2Send = mavlink_msg_to_send_buffer((packedMessage+1), &msg);
		packedMessage[0] = bytes2Send;
		
		send2GS(packedMessage);
	
}


