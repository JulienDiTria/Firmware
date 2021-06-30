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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>

using namespace time_literals;

class Medusa : public ModuleBase<Medusa>, public ModuleParams
{
public:
	Medusa();

	virtual ~Medusa() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Medusa *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	// update internal state, data and param of module
	void update();

private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);

	// define new param for medusa
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MDSA_DEPTH_TRGT>) _param_medusa_depth_trgt,   /**< depth target of the submarine */
		(ParamInt<px4::params::MDSA_SMPL_STATUS>) _param_medusa_sample_status, /**< sample status */
		(ParamFloat<px4::params::MDSA_SMPL_DEPTH>) _param_medusa_sample_depth,  /**< sample depth */
		(ParamFloat<px4::params::MDSA_SMPL_VOL>) _param_medusa_sample_volume,  /**< sample volume */
		(ParamFloat<px4::params::MDSA_SMPL_DP>) _param_medusa_sample_dp  /**< sample differential pressure */
	);

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	struct vehicle_status_s _vehicle_status {};
	struct vehicle_land_detected_s _vehicle_land_detected {};
	struct rc_channels_s _rc_channels {};
	struct debug_vect_s _debug_vect {};


	// sd card logging file name
	const char* _sd_filename_fmt = "/fs/microsd/log_%d.txt";
	char _sd_filename[64] = "";
	int _sd_fd = 0;
	void open_sd_file(uint64_t timestamp);
	void close_sd_file();
	void write_to_sd(const char* msg, int sizeof_msg);

	/// mavlink msg

	void parse_mavlink_debug();

	long _timestamp = 0 ;

	// md_status
	bool _log_sd = false;
	int _curr_smpl_nb = 0;
	int _sampling_status = 0;

	// md_stream
	float _depth_cm = 0.;
	float _delta_p_mbar = 0.;
	float _volume = 0.;

	// md_sam_1 2 and 3
	int _nb = 0; // x
	float _volume_ml = 0.; // y
	float _depth = 0.; // z
	long _time_start = 0; // x
	long _time_end = 0; // y
	long _time_needed = 0; // z
	float _pressure_dp_start = 0.; // x
	float _pressure_dp_end = 0.; // y

};

