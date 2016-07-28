#include "thread_layer.h"

mcontrol_thread::mcontrol_thread():
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
	_local_position_sub(orb_subscribe(ORB_ID(vehicle_local_position))),
	_hil_frames(0),
	_old_timestamp(0),
	_hil_last_frame(0),
	_hil_local_proj_inited(0),
	_hil_local_alt0(0.0f),
	_hil_prev_gyro{},
	_hil_prev_accel{},
	_offboard_control_mode{},
	_att_sp{},
	_rates_sp{},
	_time_offset_avg_alpha(0.6),
	_time_offset(0),
	_orb_class_instance(-1){
		
		// Initialise parameters
		read_thread_id = 0;
		read_thread_running = false;

		write_thread_id = 0;
		write_thread_running = false;

	}
mcontrol_thread::~mcontrol_thread(){
	//
	orb_unsubscribe(_control_mode_sub);
	}

void mcontrol_thread::start (){
	// Handle threads here

	// Read thread
	PX4_WARN("Start Read thread");

	// Disactivaed for the moment
	// pthread_create(&read_thread_id, NULL, &mcontrol_start_read_thread, this);


	// Write thread
	PX4_WARN("Start Write Thread");

	pthread_create(&write_thread_id, NULL, &mcontrol_start_write_thread, this);

	}

void mcontrol_thread::activate_offboard_control_mode( bool flag ){
	
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
void mcontrol_thread::set_position (float x, float y, float z){

		pos_x = x; pos_y = y; pos_z = z;
		}

void mcontrol_thread::setpoint_(float x, float y, float z){
	
	struct offboard_control_mode_s offboard_control_mode = {};

	//	PX4_INFO("Setting position to [ %f, %f, %f ]", (double)x, (double)y,(double)z);

	
		bool values_finite = true;
	
		if(values_finite){

			offboard_control_mode.ignore_position = false;
			offboard_control_mode.ignore_velocity = true;
			offboard_control_mode.ignore_acceleration_force = true;

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

			if (updated) {
				orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
			}
		
			if (_control_mode.flag_control_offboard_enabled) {

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

					// set the local pos values 
					if (!offboard_control_mode.ignore_position) {
						pos_sp_triplet.current.position_valid = true;
						pos_sp_triplet.current.x = x;
						pos_sp_triplet.current.y = y;
						pos_sp_triplet.current.z = z;

					} else {
						pos_sp_triplet.current.position_valid = false;
					}

					if (_pos_sp_triplet_pub == nullptr) {
						_pos_sp_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet), &pos_sp_triplet);

					} else {
						orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub, &pos_sp_triplet);
					}
				}
			

		}
	
	}

void mcontrol_thread::get_local_position(void){
	bool updated = false;

	orb_copy(ORB_ID(vehicle_local_position), _local_position_sub, &local_position);

	/* update local position estimate */
		orb_check(_local_position_sub, &updated);

		if (updated) {
			/* position changed */
			orb_copy(ORB_ID(vehicle_local_position), _local_position_sub, &local_position);
		}

		if(!initial_position_acquisation){
			initial_position.x = local_position.x; initial_position.y = local_position.y; initial_position.z = local_position.z;
			initial_position_acquisation = true;

			PX4_INFO("Initial position [ %f, %f, %f ]", initial_position.x, initial_position.y, initial_position.z);
		} else {
			PX4_INFO("Current position [ %f, %f, %f ]", local_position.x, local_position.y, local_position.z);
		}

		local_position__global.x = local_position.x; local_position__global.y = local_position.y; local_position__global.z = local_position.z;
	}

void mcontrol_thread::read_thread(void){

	read_thread_running = true;

	// Disable this for the moment, since the autopilot gets stuck here
	/*
	while (1) {// Replace later with while not_stop (mcontrol argument)
	}	
	*/

	read_thread_running = false;

	return;
	}
void mcontrol_thread::write_thread(void){
	// Remember to call set_position before activating the offboard control mode

	// Initialise a void setpoint first (once this thread is in execution)
	// WARNING : Modify later to assing to initial position
	// * pos_x = 1.0 ; * pos_y = 1.0 ; * pos_z = 1.0;
	pos_x = 1.0; pos_y = 1.0; pos_z = 1.0;

	write_thread_running = true; 

	while(1){ // Replace later with while not_stop (mcontrol argument)
		//setpoint_(* pos_x, * pos_y, * pos_z);
		setpoint_(pos_x, pos_y, pos_z);
		usleep(100000); // Write at 10 Hz
	}

	write_thread_running = false;
	}

void mcontrol_thread::start_read_thread (void){
	// This function is called by the void * helper function
	// It's made to assure that we don't recall the real read_thread function
	// twice

		if ( read_thread_running ){ // the read_thread_running parameter is activated from within the read thread itself
	
			PX4_WARN("Read thread already running");
			return;

		} else {

			read_thread();
			return;

		}


	}
void mcontrol_thread::start_write_thread(void){
	// This function is called by the void * helper function
	// It's made to assure that we don't recall the real read_thread function
	// twice

		if ( write_thread_running ){ // the read_thread_running parameter is activated from within the read thread itself
	
			PX4_WARN("Read thread already running");
			return;

		} else {

			write_thread();
			return;

		}

	}


void * mcontrol_start_read_thread(void * arguments) {

	// This function is specifically used to pass the "this" or the
	// commending structure through the void pointer to a lower level function

	// (mcontrol_thread * ) arguments casts the "this" structure or the commending one
	// to the mcontrol_thread type

	mcontrol_thread * othreads = (mcontrol_thread *) arguments;

	// Run the newly defined object's read thread
	othreads->start_read_thread();

	// This is required by the pthread_create POSIX function
	return NULL;
	}
void * mcontrol_start_write_thread(void * arguments){

	mcontrol_thread * othreads = (mcontrol_thread *) arguments;

	// Run the newly defined object's write thread
	othreads->start_write_thread();

	// This is required by the pthread_create POSIX function
	return NULL;
	}