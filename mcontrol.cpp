#include <px4_config.h>
#include <px4_time.h>
#include <px4_tasks.h>
#include <px4_defines.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_range_finder.h>
#include <drivers/drv_rc_input.h>
#include <time.h>
#include <float.h>
#include <unistd.h>
#ifndef __PX4_POSIX
#include <termios.h>
#endif
#include <errno.h>
#include <stdlib.h>
#include <poll.h>

#include <mathlib/mathlib.h>

#include <conversion/rotation.h>

#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <systemlib/airspeed.h>
#include <commander/px4_custom_mode.h>
#include <geo/geo.h>

#include "mcontrol.h"

// Allow PX4_INFO
#define PX4_WARNINGS

extern "C" {
	__EXPORT int mcontrol_main(int argc, char * argv[]);
}

MControl::MControl() :
	hil_local_pos{},
	hil_land_detector{},
	_control_mode{},
	_global_pos_pub(nullptr),
	_local_pos_pub(nullptr),
	_attitude_pub(nullptr),
	_gps_pub(nullptr),
	_sensors_pub(nullptr),
	_gyro_pub(nullptr),
	_accel_pub(nullptr),
	_mag_pub(nullptr),
	_baro_pub(nullptr),
	_airspeed_pub(nullptr),
	_battery_pub(nullptr),
	_cmd_pub(nullptr),
	_flow_pub(nullptr),
	_hil_distance_sensor_pub(nullptr),
	_flow_distance_sensor_pub(nullptr),
	_distance_sensor_pub(nullptr),
	_offboard_control_mode_pub(nullptr),
	_actuator_controls_pub(nullptr),
	_global_vel_sp_pub(nullptr),
	_att_sp_pub(nullptr),
	_rates_sp_pub(nullptr),
	_force_sp_pub(nullptr),
	_pos_sp_triplet_pub(nullptr),
	_att_pos_mocap_pub(nullptr),
	_vision_position_pub(nullptr),
	_telemetry_status_pub(nullptr),
	_rc_pub(nullptr),
	_manual_pub(nullptr),
	_land_detector_pub(nullptr),
	_time_offset_pub(nullptr),
	_follow_target_pub(nullptr),
	_transponder_report_pub(nullptr),
	_control_mode_sub(orb_subscribe(ORB_ID(vehicle_control_mode))),
	_hil_frames(0),
	_old_timestamp(0),
	_hil_last_frame(0),
	_hil_local_proj_inited(0),
	_hil_local_alt0(0.0f),
	_hil_prev_gyro{},
	_hil_prev_accel{},
	_hil_local_proj_ref{},
	_offboard_control_mode{},
	_att_sp{},
	_rates_sp{},
	_time_offset_avg_alpha(0.6),
	_time_offset(0),
	_orb_class_instance(-1){

	}
MControl::~MControl(){
	//
	orb_unsubscribe(_control_mode_sub);
	}

void MControl::activate_offboard_control_mode( bool flag ){
	
	struct vehicle_command_s offboard_command;
	memset(&offboard_command, 0, sizeof(offboard_command));
	/* Copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
	
		offboard_command.param1 = (float) flag;
		offboard_command.param2 = 0;
		offboard_command.param3 = 0;
		offboard_command.param4 = 0;
		offboard_command.param5 = 0;
		offboard_command.param6 = 0;
		offboard_command.param7 = 0;
			
		offboard_command.command = MAV_CMD_NAV_GUIDED_ENABLE;
	
		offboard_command.target_system = system_id;
	
		offboard_command.target_component = autopilot_id;
	
		offboard_command.source_system = 1;

		offboard_command.source_component = 0;

		offboard_command.confirmation =  true;

		if (_cmd_pub == nullptr) {
			_cmd_pub = orb_advertise(ORB_ID(vehicle_command), &offboard_command);

		} else {
			orb_publish(ORB_ID(vehicle_command), _cmd_pub, &offboard_command);
		}
	}

void MControl::set_position (float x, float y, float z){
	#ifdef PX4_WARNINGS
		PX4_INFO("Setting position to [ %f, %f, %f ]", (double)x, (double)y,(double)z);
	#endif 

	struct offboard_control_mode_s offboard_control_mode = {};

	bool values_finite =
		PX4_ISFINITE(x) && 
		PX4_ISFINITE(y) && 
		PX4_ISFINITE(z);

		#ifdef PX4_WARNINGS
			PX4_INFO("Values finite : %d", values_finite);
		#endif

		if(values_finite){

			offboard_control_mode.ignore_position = false;
			offboard_control_mode.ignore_velocity = true;
			offboard_control_mode.ignore_acceleration_force = true;

			bool is_force_sp = false;

			offboard_control_mode.ignore_attitude = true;
			offboard_control_mode.ignore_bodyrate = true;

			bool is_takeoff_sp = false;
			bool is_land_sp = false;
			bool is_loiter_sp = false;
			bool is_idle_sp = false;

			offboard_control_mode.timestamp = hrt_absolute_time();

			if (_offboard_control_mode_pub == nullptr) {
				_offboard_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &offboard_control_mode);

			} else {
				orb_publish(ORB_ID(offboard_control_mode), _offboard_control_mode_pub, &offboard_control_mode);
			}

			bool updated;
			orb_check(_control_mode_sub, &updated);

			#ifdef PX4_WARNINGS
				PX4_INFO("updated value : %d", updated);
			#endif

			if (updated) {
				orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
			}

			#ifdef PX4_WARNINGS
				PX4_INFO("offboard_control_flag status : %d", _control_mode.flag_control_offboard_enabled);
			#endif

			if (_control_mode.flag_control_offboard_enabled) {
				if (is_force_sp && offboard_control_mode.ignore_position &&
				    offboard_control_mode.ignore_velocity) {
					/* The offboard setpoint is a force setpoint only, directly writing to the force
					 * setpoint topic and not publishing the setpoint triplet topic */

					// NULL 
				} else {
					/* It's not a pure force setpoint: publish to setpoint triplet  topic */
					struct position_setpoint_triplet_s pos_sp_triplet = {};

					pos_sp_triplet.previous.valid = false;
					pos_sp_triplet.next.valid = false;
					pos_sp_triplet.current.valid = true;

					if (is_takeoff_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

					} else if (is_land_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LAND;

					} else if (is_loiter_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;

					} else if (is_idle_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;

					} else {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
					}

					/* set the local pos values */
					if (!offboard_control_mode.ignore_position) {
						pos_sp_triplet.current.position_valid = true;
						pos_sp_triplet.current.x = x;
						pos_sp_triplet.current.y = y;
						pos_sp_triplet.current.z = z;

					} else {
						pos_sp_triplet.current.position_valid = false;
					}

					if (!offboard_control_mode.ignore_velocity) {
						// NULL
						} else {
						// NULL
						}

					if (!offboard_control_mode.ignore_acceleration_force) {
						// NULL
						} else {
						// NULL
						}

					if (!offboard_control_mode.ignore_attitude) {
						// NULL
						} else {
						// NULL
						}

					if (!offboard_control_mode.ignore_bodyrate) {
						// NULL
						} else {
						// NULL
						}

					if (_pos_sp_triplet_pub == nullptr) {
						_pos_sp_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet), &pos_sp_triplet);

					} else {
						orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub, &pos_sp_triplet);
						//_pos_sp_triplet_pub = nullptr;
					}
			}

		}

		}
	}

int MControl::start (void){
	PX4_INFO("Testing..");

	activate_offboard_control_mode(true);

	uint32_t count = 0;
	uint8_t PC = 0;

	while(1){
		switch (PC) {
			case 0 :
						set_position(1, 0 , -5); 
						usleep(100000);
						break;
			case 1 : 
						set_position(1,0, -10);
						usleep(100000);
						break;
			case 2 : 
						set_position(1, 1, -5);
						usleep(100000);
						break;

			default: break;
		}
			
			count++;
			if (count == 100){
				count = 0;
				PC++;
				if (PC == 3) PC = 0;
			}
	}

	return 0;
}

int mcontrol_main(int argc, char * argv[]){
	MControl OControl;

	OControl.start();

	return 0;
	}