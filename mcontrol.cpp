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

// PX4 Thread handle 
int daemon_handle;

// Thread status
bool thread_status_running = false;

extern "C" {
	__EXPORT int mcontrol_main(int argc, char * argv[]);

	int mcontrol_thread_main(int argc, char * argv[]);
	}

MControl::MControl(){

	}
MControl::~MControl(){
	
	}


int MControl::start (void){

	mcontrol_thread control;

	PX4_INFO("Initialising read and write threads");
	control.start();

	PX4_INFO("Getting initial local position");
	control.get_local_position();

	PX4_INFO("Activating offboard control mode");
	control.activate_offboard_control_mode(true);

	sleep(1);

	PX4_INFO("Set position 1");
	control.set_position(control.initial_position.x, control.initial_position.y, control.initial_position.z - 1.0);

	control.get_local_position();

	PX4_INFO("Delay 5 seconds");
	sleep(5);
	
	PX4_INFO("Set position 2");
	control.set_position(control.initial_position.x, control.initial_position.y, control.initial_position.z - 2.0);

	control.get_local_position();

	PX4_INFO("Delay 5 seconds");
	sleep(5);

	PX4_INFO("Set position 3");
	control.set_position(control.initial_position.x, control.initial_position.y, control.initial_position.z);

	PX4_INFO("Delay 5 seconds");
	sleep(5);

	PX4_INFO("Disabling offboard control mode");
	control.activate_offboard_control_mode(false);	

	control.get_local_position();

	return 1;
	}

int mcontrol_main(int argc, char * argv[]){
	thread_status_running = true;

	daemon_handle = px4_task_spawn_cmd ("MControl daemon", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 2000, mcontrol_thread_main, (char *const *)NULL);

	thread_status_running = false;

	return 1;
	}
int mcontrol_thread_main(int argc, char * argv[]){
	MControl OControl;

	OControl.start();

	return 0;
	}