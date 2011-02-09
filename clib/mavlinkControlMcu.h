#ifndef _MAVLINKCONTROLMCU_H_
#define _MAVLINKCONTROLMCU_H_

#ifdef __cplusplus
       extern "C"{
#endif
       	
  #include "mavlink.h"
  #include "apDefinitions.h"
  
  // Comments Nomenclature
  // // In init method
  //
  // .. Already in Decoder (thus we can receive messages of this type)
  // -- Does not need Decoder (messages are only outgoing)
	//	
  // == Scheduled for regular transmision
  // ** It does not need to be scheduled
	//	
  // ^^ Temporary value does not require decode nor encode
 
  typedef struct mavlink_pid_values_t {
		float P[MAX_NUM_PIDS];
		float I[MAX_NUM_PIDS];
		float D[MAX_NUM_PIDS];
	}mavlink_pid_values_t;


	typedef struct mavlink_waypoint_values_t{
		float			lat[MAX_NUM_WPS];
		float			lon[MAX_NUM_WPS];
		float			alt[MAX_NUM_WPS];
		uint8_t		type[MAX_NUM_WPS];
		uint16_t	orbit[MAX_NUM_WPS];
		uint8_t		wpCount;
	}mavlink_waypoint_values_t;


	typedef struct mavlink_pending_requests_t{
		// requests
		uint8_t		ping;
		uint8_t		pid;
		uint8_t		wp;
		uint8_t		midLvlCmds;
		uint8_t		pt;
		uint8_t		mode;
		
		// WP Protocol states
		uint8_t		wpTransaction;
		uint8_t		wpProtState;	
		uint8_t		wpCurrentWpInTransaction;
		uint8_t		wpTimeOut;
		uint8_t		wpTotalWps;
		
		// Info
		uint8_t		pidIdx;
		uint8_t		wpsIdx;
		
		//uint8_t 	requestCount;
	}mavlink_pending_requests_t;
	
	extern mavlink_raw_imu_t 					mlRawImuData;					// 	..	==
	extern mavlink_gps_raw_t					mlGpsData;						// 	..	==
	extern mavlink_cpu_load_t 				mlCpuLoadData;				// 	..	== 
	extern mavlink_air_data_t 				mlAirData;						// 	..	== 
	extern mavlink_sensor_bias_t 			mlSensorBiasData;			// 	..	== 	
	extern mavlink_diagnostic_t 			mlDiagnosticData;			// 	..	== 
	extern mavlink_raw_pressure_t 		mlRawPressureData;		// 	..	== 
	extern mavlink_attitude_t 				mlAttitudeData;				// 	..	== 
	extern mavlink_local_position_t 	mlLocalPositionData;	// 	.. 	==
	extern mavlink_pilot_console_t 		mlPilotConsoleData;		// 	..	==	
	extern mavlink_filtered_data_t 		mlFilteredData;				// 	..
	extern mavlink_boot_t 						mlBoot;								//	..	==
	extern mavlink_gps_date_time_t 		mlGpsDateTime;				// 	..
	
	extern mavlink_heartbeat_t 				mlHeartbeat; 					// 	..	== 
	extern mavlink_mid_lvl_cmds_t			mlMidLevelCommands; 	// 	..	**
	extern mavlink_pwm_commands_t			mlPwmCommands; 				// 	--	==
	extern mavlink_pid_values_t				mlPidValues;					// 	..  **	defined in mavlinkControlMcu.h
	extern mavlink_pid_t							mlSinglePid; 					// 	^^	^^
	extern mavlink_waypoint_values_t	mlWpValues;						// 	..	**	defined in mavlinkControlMcu.h
	extern mavlink_waypoint_t					mlSingleWp;						// 	^^	^^
	extern mavlink_slugs_navigation_t	mlNavigation; 				// 	--  ==		
	extern mavlink_data_log_t					mlDataLog; 						// 	--	==
	extern mavlink_ctrl_srfc_pt_t			mlPassthrough; 				// 	..	**
	extern mavlink_attitude_t 				mlAttitudeRotated;		// 	--	==
	extern mavlink_action_ack_t				mlActionAck; 					// 	-- 	==
	extern mavlink_pending_requests_t mlPending; 						//	--	**
	extern mavlink_ping_t							mlPing; 							// 	..	**
	extern mavlink_slugs_action_t			mlAction; 						// 	..	**
	extern mavlink_waypoint_request_t	mlWpRequest; 					// 	..	**
	extern mavlink_waypoint_ack_t			mlWpAck; 							// 	..	**
	extern mavlink_waypoint_count_t		mlWpCount; 						// 	..	**
	extern mavlink_sys_status_t				mlSystemStatus;				//	--	==


	void mavlinkInit (void);
  	
       	
#ifdef __cplusplus
       }
#endif

#endif /* _MAVLINKCONTROLMCU_H_ */
