#ifndef _MAVLINKSENSORMCU_H_
#define _MAVLINKSENSORMCU_H_

#ifdef __cplusplus
       extern "C"{
#endif
       	
#include "mavlink.h"

  extern mavlink_gps_raw_t				mlGpsData;//
  extern mavlink_cpu_load_t 			mlCpuLoadData;//
  extern mavlink_air_data_t 			mlAirData;//
  extern mavlink_sensor_bias_t 		mlSensorBiasData;//
  extern mavlink_diagnostic_t 		mlDiagnosticData;//
  extern mavlink_rc_channels_raw_t mlPilotConsoleData;//
  extern mavlink_raw_imu_t 				mlRawImuData;//
	extern mavlink_raw_pressure_t 	mlRawPressureData;//
	extern mavlink_attitude_t 			mlAttitudeData;//
  extern mavlink_local_position_t mlLocalPositionData;//
	extern mavlink_scaled_imu_t		 	mlFilteredData;//
	extern mavlink_boot_t 					mlBoot;//
  extern mavlink_gps_date_time_t 	mlGpsDateTime;
  
  void mavlinkInit (void);

#ifdef __cplusplus
       }
#endif
       
#endif /* _MAVLINKSENSORMCU_H_ */
