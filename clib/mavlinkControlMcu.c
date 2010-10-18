#include "mavlinkControlMcu.h"

mavlink_raw_imu_t 					mlRawImuData;
mavlink_gps_raw_t						mlGpsData;
mavlink_cpu_load_t 					mlCpuLoadData;
mavlink_air_data_t 					mlAirData;
mavlink_sensor_bias_t 			mlSensorBiasData;
mavlink_diagnostic_t 				mlDiagnosticData;
mavlink_raw_pressure_t 			mlRawPressureData;
mavlink_attitude_t 					mlAttitudeData;
mavlink_local_position_t 		mlLocalPositionData;
mavlink_pilot_console_t 		mlPilotConsoleData;
mavlink_filtered_data_t 		mlFilteredData;
mavlink_boot_t 							mlBoot;
mavlink_system_time_t 			mlSystemTime;
mavlink_gps_date_time_t 		mlGpsDateTime;

mavlink_heartbeat_t 				mlHeartbeat;
mavlink_mid_lvl_cmds_t 			mlMidLevelCommands;
mavlink_set_mode_t 					mlApMode;
mavlink_pwm_commands_t			mlPwmCommands;
mavlink_pid_values_t				mlPidValues;		//defined in mavlinkControlMcu.h
mavlink_set_pid_t						mlSinglePid;
mavlink_waypoint_values_t		mlWpValues;			//defined in mavlinkControlMcu.h
mavlink_waypoint_t					mlSingleWp;
mavlink_slugs_navigation_t	mlNavigation;		
mavlink_data_log_t					mlDataLog;
mavlink_ctrl_srfc_pt_t			mlPassthrough
mavlink_attitude_t 					mlAttitudeRotated;
mavlink_action_ack_t				mlActionAck;	
mavlink_pending_requests_t  mlPending;
mavlink_ping_t							mlPing;
mavlink_slugs_action_t			mlAction;


void mavlinkInit (void){
	
	// clear all the variables
	memset(&mlGpsData, 0, sizeof(mavlink_gps_raw_t));
	memset(&mlCpuLoadData, 0, sizeof(mavlink_cpu_load_t));
	memset(&mlAirData, 0, sizeof(mavlink_air_data_t));
	memset(&mlSensorBiasData, 0, sizeof(mavlink_sensor_bias_t));
	memset(&mlDiagnosticData, 0, sizeof(mavlink_diagnostic_t));
	memset(&mlPilotConsoleData, 0, sizeof(mavlink_pilot_console_t));
	memset(&mlRawImuData ,0, sizeof(mavlink_raw_imu_t));
	memset(&mlRawPressureData ,0, sizeof(mavlink_raw_pressure_t));
	memset(&mlAttitudeData ,0, sizeof(mavlink_attitude_t));	
	memset(&mlLocalPositionData ,0, sizeof(mavlink_local_position_t));	
	memset(&mlFilteredData ,0, sizeof(mavlink_filtered_data_t));	
	memset(&mlSystemTime ,0, sizeof(mavlink_system_time_t));	
	memset(&mlBoot ,0, sizeof(mavlink_boot_t));
	memset(&mlGpsDateTime ,0, sizeof(mavlink_gps_date_time_t));	
	memset(&mlHeartbeat ,0, sizeof(mavlink_gps_heartbeat_t));	

	memset(&mlMidLevelCommands ,0, sizeof(mavlink_mid_lvl_cmds_t));	
	memset(&mlApMode ,0, sizeof(mavlink_set_mode_t));	
	memset(&mlPwmCommands ,0, sizeof(mavlink_pwm_commands_t));	
	memset(&mlPidValues ,0, sizeof(mavlink_pid_values_t));	
	memset(&mlSinglePid ,0, sizeof(mavlink_pid_t));	
	memset(&mlWpValues ,0, sizeof(mavlink_waypoint_values_t));	
	memset(&mlSingleWp ,0, sizeof(mavlink_waypoint_t));	
	memset(&mlNavigation ,0, sizeof(mavlink_slugs_navigation_t));	
	memset(&mlDataLog ,0, sizeof(mavlink_data_log_t));	
	memset(&mlPassthrough ,0, sizeof(mavlink_ctrl_srfc_pt_t));	
	memset(&mlAttitudeRotated ,0, sizeof(mavlink_attitude_t));	
	memset(&mlActionAck,0, sizeof(mavlink_action_ack_t));	
	memset(&mlPending ,0, sizeof(mavlink_pending_requests_t));	
	memset(&mlPing ,0, sizeof(mavlink_ping_t));	
	memset(&mlAction ,0, sizeof(mavlink_slugs_action_t));	
	
	
	mlPending.requestCount = 0;
	mlActionAck.action = SLUGS_ACTION_NONE;
}

