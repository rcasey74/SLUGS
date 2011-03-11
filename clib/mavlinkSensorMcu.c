
#include "mavlinkSensorMcu.h"


// Declare the global structures that will hold the state of the Sensor MCU

mavlink_raw_imu_t 				mlRawImuData;
mavlink_gps_raw_t					mlGpsData;
mavlink_cpu_load_t 				mlCpuLoadData;
mavlink_scaled_pressure_t mlAirData;
mavlink_sensor_bias_t 		mlSensorBiasData;
mavlink_diagnostic_t 			mlDiagnosticData;
mavlink_raw_pressure_t 		mlRawPressureData;
mavlink_attitude_t 				mlAttitudeData;
mavlink_local_position_t 	mlLocalPositionData;
mavlink_rc_channels_raw_t	mlPilotConsoleData;
mavlink_scaled_imu_t		 	mlFilteredData;
mavlink_boot_t						mlBoot;
mavlink_gps_date_time_t 	mlGpsDateTime;



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
}