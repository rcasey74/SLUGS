#include "mavlinkControlMcu.h"

mavlink_raw_imu_t 					mlRawImuData;
mavlink_gps_raw_t						mlGpsData;
mavlink_cpu_load_t 					mlCpuLoadData;
mavlink_scaled_pressure_t 	mlAirData;
mavlink_sensor_bias_t 			mlSensorBiasData;
mavlink_diagnostic_t 				mlDiagnosticData;
mavlink_raw_pressure_t 			mlRawPressureData;
mavlink_attitude_t 					mlAttitudeData;
mavlink_local_position_t 		mlLocalPositionData;
mavlink_rc_channels_raw_t		mlPilotConsoleData;
mavlink_scaled_imu_t		 		mlFilteredData;
mavlink_boot_t 							mlBoot;
mavlink_gps_date_time_t 		mlGpsDateTime;

mavlink_heartbeat_t 				mlHeartbeat;
mavlink_mid_lvl_cmds_t 			mlMidLevelCommands;
mavlink_set_mode_t 					mlApMode;
mavlink_servo_output_raw_t	mlPwmCommands;
mavlink_waypoint_values_t		mlWpValues;			//defined in mavlinkControlMcu.h
mavlink_waypoint_t					mlSingleWp;
mavlink_slugs_navigation_t	mlNavigation;		
mavlink_data_log_t					mlDataLog;
mavlink_ctrl_srfc_pt_t			mlPassthrough;
mavlink_attitude_t 					mlAttitudeRotated;
mavlink_action_ack_t				mlActionAck;	
mavlink_pending_requests_t  mlPending;
mavlink_ping_t							mlPing;
mavlink_slugs_action_t			mlAction;
mavlink_waypoint_request_t	mlWpRequest;
mavlink_waypoint_ack_t			mlWpAck;
mavlink_waypoint_count_t		mlWpCount;
mavlink_sys_status_t				mlSystemStatus;

struct pi_struct 						mlParamInterface;

void mavlinkInit (void){
	
	// clear all the variables
	memset(&mlGpsData, 0, sizeof(mavlink_gps_raw_t));
	memset(&mlCpuLoadData, 0, sizeof(mavlink_cpu_load_t));
	memset(&mlAirData, 0, sizeof(mavlink_scaled_pressure_t));
	memset(&mlSensorBiasData, 0, sizeof(mavlink_sensor_bias_t));
	memset(&mlDiagnosticData, 0, sizeof(mavlink_diagnostic_t));
	memset(&mlPilotConsoleData, 0, sizeof(mavlink_rc_channels_raw_t));
	memset(&mlRawImuData ,0, sizeof(mavlink_raw_imu_t));
	memset(&mlRawPressureData ,0, sizeof(mavlink_raw_pressure_t));
	memset(&mlAttitudeData ,0, sizeof(mavlink_attitude_t));	
	memset(&mlLocalPositionData ,0, sizeof(mavlink_local_position_t));	
	memset(&mlFilteredData ,0, sizeof(mavlink_scaled_imu_t));	
	memset(&mlBoot ,0, sizeof(mavlink_boot_t));
	memset(&mlGpsDateTime ,0, sizeof(mavlink_gps_date_time_t));	
	memset(&mlHeartbeat ,0, sizeof(mavlink_heartbeat_t));	

	memset(&mlMidLevelCommands ,0, sizeof(mavlink_mid_lvl_cmds_t));	
	memset(&mlPwmCommands ,0, sizeof(mavlink_servo_output_raw_t));	
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
	memset(&mlWpRequest ,0, sizeof(mavlink_waypoint_request_t));	
	memset(&mlWpAck ,0, sizeof(mavlink_waypoint_ack_t));	
	memset(&mlWpCount ,0, sizeof(mavlink_waypoint_count_t));	
	memset(&mlSystemStatus ,0, sizeof(mavlink_sys_status_t));	
	
	memset(&mlParamInterface ,0, sizeof(struct pi_struct));	
	

	mlActionAck.action = SLUGS_ACTION_NONE;
	
	mlPending.wpTransaction = 0;
	mlPending.wpProtState = WP_PROT_IDLE;	
	mlPending.wpTimeOut = 0;
	
	mlHeartbeat.mavlink_version = MAVLINK_VERSION;
	
	// Initialize the system Status
	mlSystemStatus.mode 		= MAV_MODE_MANUAL;
	mlSystemStatus.nav_mode = MAV_NAV_GROUNDED;
	mlSystemStatus.status		= MAV_STATE_BOOT;
	mlSystemStatus.load			= 500;
	mlSystemStatus.vbat			= 12000;

	
	populateParameterInterface();
}

void populateParameterInterface (void){
	memcpy(mlParamInterface.param_name[PAR_PID_AIRSPEED_P], "PID_AIRSPD_P", SLUGS_PARAM_NAME_LENGTH );
	memcpy(mlParamInterface.param_name[PAR_PID_AIRSPEED_I], "PID_AIRSPD_I", SLUGS_PARAM_NAME_LENGTH );
	memcpy(mlParamInterface.param_name[PAR_PID_AIRSPEED_D], "PID_AIRSPD_D", SLUGS_PARAM_NAME_LENGTH );
	
	memcpy(mlParamInterface.param_name[PAR_PID_PITCH_FO_P], "PID_PIT_FO_P", SLUGS_PARAM_NAME_LENGTH );
	memcpy(mlParamInterface.param_name[PAR_PID_PITCH_FO_I], "PID_PIT_FO_I", SLUGS_PARAM_NAME_LENGTH );
	memcpy(mlParamInterface.param_name[PAR_PID_PITCH_FO_D], "PID_PIT_FO_D", SLUGS_PARAM_NAME_LENGTH );
	
	memcpy(mlParamInterface.param_name[PAR_PID_ROLL_CON_P], "PID_ROLL_CO_P", SLUGS_PARAM_NAME_LENGTH );
	memcpy(mlParamInterface.param_name[PAR_PID_ROLL_CON_I], "PID_ROLL_CO_I", SLUGS_PARAM_NAME_LENGTH );
	memcpy(mlParamInterface.param_name[PAR_PID_ROLL_CON_D], "PID_ROLL_CO_D", SLUGS_PARAM_NAME_LENGTH );
	
	memcpy(mlParamInterface.param_name[PAR_PID_HE_TO_PI_P], "PID_HE2PITC_P", SLUGS_PARAM_NAME_LENGTH );
	memcpy(mlParamInterface.param_name[PAR_PID_HE_TO_PI_I], "PID_HE2PITC_I", SLUGS_PARAM_NAME_LENGTH );
	
	memcpy(mlParamInterface.param_name[PAR_PID_HEI_ERR_FF], "PID_HERR_FF", SLUGS_PARAM_NAME_LENGTH );
	
	memcpy(mlParamInterface.param_name[PAR_PID_YAW_DAMP_P], "PID_YAW_DA_P", SLUGS_PARAM_NAME_LENGTH );
	memcpy(mlParamInterface.param_name[PAR_PID_YAW_DAMP_I], "PID_YAW_DA_I", SLUGS_PARAM_NAME_LENGTH );
	memcpy(mlParamInterface.param_name[PAR_PID_YAW_DAMP_D], "PID_YAW_DA_D", SLUGS_PARAM_NAME_LENGTH );

	memcpy(mlParamInterface.param_name[PAR_PID_PITC_DT_FF], "PID_PIT_DT_FF", SLUGS_PARAM_NAME_LENGTH );
	
	memcpy(mlParamInterface.param_name[PAR_CONFIG_ROLL_R ], "CONFIG_ROLL_R ", SLUGS_PARAM_NAME_LENGTH );
	memcpy(mlParamInterface.param_name[PAR_CONFIG_PITCH_R], "CONFIG_PITCH_R", SLUGS_PARAM_NAME_LENGTH );
	memcpy(mlParamInterface.param_name[PAR_CONFIG_YAW_R  ], "CONFIG_YAW_R  ", SLUGS_PARAM_NAME_LENGTH );
	
	memcpy(mlParamInterface.param_name[PAR_NAV_L2_BASE  ], "NAV_L2_BASE	 ", SLUGS_PARAM_NAME_LENGTH );
	memcpy(mlParamInterface.param_name[PAR_NAV_PRETURN_K], "NAV_PRETURN_K", SLUGS_PARAM_NAME_LENGTH );
	memcpy(mlParamInterface.param_name[PAR_NAV_SSCOMP_ON], "NAV_SSCOMP_ON", SLUGS_PARAM_NAME_LENGTH );
	
	memcpy(mlParamInterface.param_name[PAR_L1_OMEGA ], "L1_OMEGA ", SLUGS_PARAM_NAME_LENGTH );
	memcpy(mlParamInterface.param_name[PAR_L1_M     ], "L1_M     ", SLUGS_PARAM_NAME_LENGTH );
	memcpy(mlParamInterface.param_name[PAR_L1_GAMMA ], "L1_GAMMA ", SLUGS_PARAM_NAME_LENGTH );
	memcpy(mlParamInterface.param_name[PAR_L1_ON_OFF], "L1_ON_OFF", SLUGS_PARAM_NAME_LENGTH );

				
}

