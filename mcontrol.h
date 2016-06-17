/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mcontrol.h
 * 
 * 
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 *
 * This was largely inspired from mavlink_receive.h
 * Modification made by Mohamed Hage Hassan <mohamed.hagehassan@yahoo.com>
 *
 */

#pragma once

#include <systemlib/perf_counter.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
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

class MControl {
	public:
		// constructor
		MControl();
		// Destructor
		~MControl();
		//start 
		int start(void);
	private:
		int MAV_CMD_NAV_GUIDED_ENABLE = 92;
		
		int system_id = 1;
		int autopilot_id = 1;

		void activate_offboard_control_mode( bool flag );
		void set_position(float, float, float);

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
		int _control_mode_sub;
		int _hil_frames;
		uint64_t _old_timestamp;
		uint64_t _hil_last_frame;
		bool _hil_local_proj_inited;
		float _hil_local_alt0;
		float _hil_prev_gyro[3];
		float _hil_prev_accel[3];
		struct map_projection_reference_s _hil_local_proj_ref;
		struct offboard_control_mode_s _offboard_control_mode;
		struct vehicle_attitude_setpoint_s _att_sp;
		struct vehicle_rates_setpoint_s _rates_sp;
		double _time_offset_avg_alpha;
		int64_t _time_offset;
		int	_orb_class_instance;
};

