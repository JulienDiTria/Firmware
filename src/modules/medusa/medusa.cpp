/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "medusa.hpp"

#include "medusa_mavlink_debug.h"


#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/uORB.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>


extern "C" __EXPORT int medusa_main(int argc, char *argv[]);

int Medusa::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int Medusa::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Medusa::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Medusa *Medusa::instantiate(int argc, char *argv[])
{
	Medusa *instance = new Medusa();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

Medusa::Medusa(): ModuleParams(nullptr)
{
}

void Medusa::run()
{
	// Subscirbe to "vehicle_status", then set a polling interval of 200ms
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	int vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	int rc_channels_sub = orb_subscribe(ORB_ID(rc_channels));
	int debug_sub = orb_subscribe(ORB_ID(debug_vect));

	orb_set_interval(vehicle_status_sub, 200);
	orb_set_interval(vehicle_land_detected_sub, 200);
	orb_set_interval(rc_channels_sub, 200);
	orb_set_interval(debug_sub, 200);

	px4_pollfd_struct_t fds[] = {
		{ .fd = vehicle_status_sub,   .events = POLLIN },
		{ .fd = vehicle_land_detected_sub,   .events = POLLIN },
		{ .fd = rc_channels_sub,   .events = POLLIN },
		{ .fd = debug_sub,   .events = POLLIN },
	};

	// initialize parameters
	parameters_update(true);

	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else {
			if (fds[0].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &_vehicle_status);
				PX4_INFO("received _vehicle_status");
			}
			if (fds[1].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_land_detected), vehicle_land_detected_sub, &_vehicle_land_detected);
				PX4_INFO("received _vehicle_land_detected");
			}
			if (fds[2].revents & POLLIN) {
				orb_copy(ORB_ID(rc_channels), rc_channels_sub, &_rc_channels);
				PX4_INFO("received _rc_channels");
			}
			if (fds[3].revents & POLLIN) {
				orb_copy(ORB_ID(debug_vect), debug_sub, &_debug_vect);
				PX4_INFO("received _debug_vect");
				parse_mavlink_debug();
			}
		}
		parameters_update();
	}

	orb_unsubscribe(vehicle_status_sub);
	orb_unsubscribe(vehicle_land_detected_sub);
    	orb_unsubscribe(rc_channels_sub);
    	orb_unsubscribe(debug_sub);
}

void Medusa::update(){
	// _vehicle_status.
	// arming_state

}

void Medusa::open_sd_file(uint64_t timestamp){
	snprintf(_sd_filename, 64, _sd_filename_fmt, _debug_vect.timestamp);
	_sd_fd = open(_sd_filename, O_TRUNC | O_WRONLY | O_CREAT, PX4_O_MODE_666);
	if(_sd_fd<0){
		PX4_WARN("can't open sd file '%s'", _sd_filename);
	}
	return ;
}

void Medusa::close_sd_file(){
	if(_sd_fd<0) return;
	close(_sd_fd);
	return;
}

void Medusa::write_to_sd(const char* msg, int sizeof_msg){
	if(_sd_fd<0) {
		PX4_INFO("Sd card file with fd %d, filename '%s' not open", _sd_fd, _sd_filename);
		return;
	}
	int wret = write(_sd_fd, msg, sizeof_msg);
	if (wret != sizeof_msg) {
		PX4_INFO("WRITE ERROR! on sd card with fd %d, filename %s", _sd_fd, _sd_filename);
	}
}

void Medusa::parse_mavlink_debug(){
	if(!strncmp(_debug_vect.name, MD_STATUS, 10)){
		if(!_log_sd && (_debug_vect.x > 0.5f )){
			open_sd_file(_debug_vect.timestamp);
			const char msg[] = "start sd card writting\n";
			write_to_sd(msg, sizeof(msg));
		}
		if(_log_sd && (_debug_vect.x < 0.5f )){
			close_sd_file();
		}
		_log_sd = (_debug_vect.x > 0.5f );
		_curr_smpl_nb = (int) _debug_vect.y;
		_sampling_status = (int) _debug_vect.z;
		const char msg_2_fmt[] = "curr_smpl_nb %d\nsampling_status %d\n";
		char msg_2[64] = "";
		snprintf(msg_2, 64-1, msg_2_fmt, _curr_smpl_nb, _sampling_status);
		write_to_sd(msg_2, sizeof(msg_2));
	}
	else if(!strncmp(_debug_vect.name, MD_STREAM, 10)){
		_depth_cm = _debug_vect.x;
		_delta_p_mbar = _debug_vect.y;
		_volume = _debug_vect.z;
	}
	else if(!strncmp(_debug_vect.name, MD_SAM_1, 10)){
		_nb = (int) _debug_vect.x;
		_volume_ml = _debug_vect.y;
		_depth = _debug_vect.z;
	}
	else if(!strncmp(_debug_vect.name, MD_SAM_2, 10)){
		_time_start = (long) _debug_vect.x;
		_time_end = (long) _debug_vect.y;
		_time_needed = (long)  _debug_vect.z;
	}
	else if(!strncmp(_debug_vect.name, MD_SAM_3, 10)){
		_pressure_dp_start = _debug_vect.x;
		_pressure_dp_end = _debug_vect.y;
		// unused _debug_vect.z;
	}
}

void Medusa::parameters_update(bool force)
{
	// check for parameter updates
	if (force || _parameter_update_sub.updated()) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int Medusa::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

This is the medusa module running as a task in the background with start/stop/status functionality.

### Implementation
It determines the state of the medusa pod and coiler to make the pod active and handling the coiling and uncoiling.

### Examples
CLI usage example:
$ module start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "medusa");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int medusa_main(int argc, char *argv[])
{
	return Medusa::main(argc, argv);
}
