#ifndef _THREAD_LAYER_H
#define _THREAD_LAYER_H

#pragma once

#include <systemlib/perf_counter.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/att_pos_mocap.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_force_setpoint.h>
#include <uORB/topics/time_offset.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/follow_target.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/gps_inject_data.h>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <pthread.h>

// Local position topic
#include <uORB/topics/vehicle_local_position.h>

// Read pthread helper function
void * mcontrol_start_read_thread(void *);

// Write pthread helper function
void * mcontrol_start_write_thread(void *);

class mcontrol_thread {
	public:
		mcontrol_thread();
		~mcontrol_thread();

		// Read thread status
		pthread_t read_thread_id;
		bool read_thread_running; 

		// Write thread status
		pthread_t write_thread_id;
		bool write_thread_running; 

		// Read thread starter
		void start_read_thread(void);

		// Write thread starter
		void start_write_thread(void);

		// Offboard control activation
		void activate_offboard_control_mode( bool flag );
		
		// Generic setpoint function
		void set_position(float, float, float);

		// Main initialisation function 
		void start (void);

		// Local Position related data structures
		struct vehicle_local_position_s local_position__global = {};
		struct vehicle_local_position_s initial_position = {};

		// Local Position function 
		void get_local_position(void);

	private:

		int MAV_CMD_NAV_GUIDED_ENABLE = 92;
		
		int system_id = 1;
		int autopilot_id = 1;

		float pos_x, pos_y, pos_z;

		// Generic setpoint function (called by set_position)
		void setpoint_(float, float, float);

		// Read thread
		void read_thread(void);

		// Write thread
		void write_thread(void);

		// Local Position data structure
		struct vehicle_local_position_s local_position = {};
		
		// Local Position initial position lock
		bool initial_position_acquisation = false;


		struct vehicle_local_position_s hil_local_pos;
		struct vehicle_land_detected_s hil_land_detector;
		struct vehicle_control_mode_s _control_mode;
		orb_advert_t _global_pos_pub;
		orb_advert_t _local_pos_pub;
		orb_advert_t _attitude_pub;
		orb_advert_t _gps_pub;
		orb_advert_t _sensors_pub;
		orb_advert_t _gyro_pub;
		orb_advert_t _accel_pub;
		orb_advert_t _mag_pub;
		orb_advert_t _baro_pub;
		orb_advert_t _airspeed_pub;
		orb_advert_t _battery_pub;
		orb_advert_t _cmd_pub;
		orb_advert_t _flow_pub;
		orb_advert_t _hil_distance_sensor_pub;
		orb_advert_t _flow_distance_sensor_pub;
		orb_advert_t _distance_sensor_pub;
		orb_advert_t _offboard_control_mode_pub;
		orb_advert_t _actuator_controls_pub;
		orb_advert_t _global_vel_sp_pub;
		orb_advert_t _att_sp_pub;
		orb_advert_t _rates_sp_pub;
		orb_advert_t _force_sp_pub;
		orb_advert_t _pos_sp_triplet_pub;
		orb_advert_t _att_pos_mocap_pub;
		orb_advert_t _vision_position_pub;
		orb_advert_t _telemetry_status_pub;
		orb_advert_t _rc_pub;
		orb_advert_t _manual_pub;
		orb_advert_t _land_detector_pub;
		orb_advert_t _time_offset_pub;
		orb_advert_t _follow_target_pub;
		orb_advert_t _transponder_report_pub;

		int _gps_inject_data_next_idx = 0;

		// Offboard control mode subscription
		int _control_mode_sub;

		// Local position subscription
		int _local_position_sub;

		int _hil_frames;
		uint64_t _old_timestamp;
		uint64_t _hil_last_frame;
		bool _hil_local_proj_inited;
		float _hil_local_alt0;
		float _hil_prev_gyro[3];
		float _hil_prev_accel[3];
		struct offboard_control_mode_s _offboard_control_mode;

		struct vehicle_attitude_setpoint_s _att_sp;
		struct vehicle_rates_setpoint_s _rates_sp;
		double _time_offset_avg_alpha;
		int64_t _time_offset;
		int	_orb_class_instance;
};


#endif