#ifndef _MAVLINKSENSORMCU_H_
#define _MAVLINKSENSORMCU_H_

#include "mavlink.h"

  extern mavlink_gps_raw_t	mlGpsData;
  extern mavlink_cpu_load_t mlCpuLoadData;
  extern mavlink_air_data_t mlAirData;
  extern mavlink_sensor_bias_t mlSensorBiasData;
  extern mavlink_diagnostic_t mlDiagnosticData;
  extern mavlink_pilot_console_t mlPilotConsoleData;
  extern mavlink_pwm_commands_t mlPwmCommandsData;
  extern mavlink_raw_imu_t mlRawImuData;
	extern mavlink_raw_pressure_t mlRawPressureData;
	extern mavlink_attitude_t mlAttitudeData;
  extern mavlink_local_position_t mlLocalPositionData;
	extern mavlink_filtered_data_t mlFilteredData;
	extern mavlink_boot_t mlBoot;
  extern mavlink_system_time_t mlSystemTime;
  
  void mavlinkInit (void);

#endif /* _MAVLINKSENSORMCU_H_ */
