/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file shadowlift_att_control_main.cpp
 * shadowlift attitude controller.
 *
 * @author Gil Klar	<gil.klar@gmail.com>
 */

#include "shadowlift_att_control.hpp"

using namespace matrix;

ShadowliftAttitudeControl::ShadowliftAttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, "shadowlift_att_control"))
{
}

ShadowliftAttitudeControl::~ShadowliftAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
ShadowliftAttitudeControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
ShadowliftAttitudeControl::parameter_update_poll()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}
}

void ShadowliftAttitudeControl::publishTorqueSetpoint(const hrt_abstime &timestamp_sample)
{
	vehicle_torque_setpoint_s v_torque_sp = {};
	v_torque_sp.timestamp = hrt_absolute_time();
	v_torque_sp.timestamp_sample = timestamp_sample;

	// zero actuators if not armed
	if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		v_torque_sp.xyz[0] = 0.f;
		v_torque_sp.xyz[1] = 0.f;
		v_torque_sp.xyz[2] = _manual_control_setpoint.yaw;
	}

	_vehicle_torque_setpoint_pub.publish(v_torque_sp);
}

void ShadowliftAttitudeControl::publishTorqueSetpoint2(const hrt_abstime &timestamp_sample, const float &current_yaw_rate)
{
	vehicle_torque_setpoint_s v_torque_sp = {};
	v_torque_sp.timestamp = hrt_absolute_time();
	v_torque_sp.timestamp_sample = timestamp_sample;

	float yaw_rate_sp = _manual_control_setpoint.yaw; //  0.f;
	float yaw_rate_err = yaw_rate_sp- current_yaw_rate;

	const double max_yaw_torque = 0.5f;

	double yaw_torque = _param_mc_yawrate_p.get() * yaw_rate_err;
	yaw_torque = yaw_torque > max_yaw_torque ? max_yaw_torque: yaw_torque;
	yaw_torque = yaw_torque < -max_yaw_torque ? -max_yaw_torque: yaw_torque;

	// zero actuators if not armed
	if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		v_torque_sp.xyz[0] = 0.f;
		v_torque_sp.xyz[1] = 0.f;
		v_torque_sp.xyz[2] = yaw_torque;
	}

	_vehicle_torque_setpoint_pub.publish(v_torque_sp);
}

void ShadowliftAttitudeControl::publishThrustSetpoint(const hrt_abstime &timestamp_sample)
{
	vehicle_thrust_setpoint_s v_thrust_sp = {};
	v_thrust_sp.timestamp = hrt_absolute_time();
	v_thrust_sp.timestamp_sample = timestamp_sample;

	// zero actuators if not armed
	if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		v_thrust_sp.xyz[0] = _manual_control_setpoint.pitch;
		v_thrust_sp.xyz[1] = _manual_control_setpoint.roll;
	}

	_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);
}

double quaternionToYaw(double q_w, double q_x, double q_y, double q_z) {
	return std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z));
    }

void
ShadowliftAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	// vehicle_attitude_s v_att;

	// if (_vehicle_attitude_sub.update(&v_att)) {

	// 	// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
	// 	// const float dt = math::constrain(((v_att.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
	// 	_last_run = v_att.timestamp_sample;
	// 	float yaw = quaternionToYaw(v_att.q[0], v_att.q[1], v_att.q[2], v_att.q[3]);
	// 	PX4_INFO("current yaw: %f", (double)yaw);

	// 	double yaw_sp = 0.f;
	// 	double yaw_err = (double)yaw * yaw_sp;

	// 	if (yaw_err > 0.1){
	// 		torque_cmd[2] = -0.2;
	// 	}else if(yaw_err < -0.1){
	// 		torque_cmd[2] = 0.2;
	// 	}else{
	// 		torque_cmd[2] = 0.f;
	// 	}
	// }

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		/* run the rate controller immediately after a gyro update */
		// publishTorqueSetpoint(angular_velocity.timestamp_sample);
		publishTorqueSetpoint2(angular_velocity.timestamp_sample, angular_velocity.xyz[2]);
		publishThrustSetpoint(angular_velocity.timestamp_sample);

		/* check for updates in manual control topic */
		_manual_control_setpoint_sub.update(&_manual_control_setpoint);

		/* check for updates in vehicle status topic */
		_vehicle_status_sub.update(&_vehicle_status);

		parameter_update_poll();
	}

	perf_end(_loop_perf);
}

int ShadowliftAttitudeControl::task_spawn(int argc, char *argv[])
{
	ShadowliftAttitudeControl *instance = new ShadowliftAttitudeControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ShadowliftAttitudeControl::print_status()
{
	PX4_INFO("Running GK modified code");

	perf_print_counter(_loop_perf);

	return 0;
}

int ShadowliftAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ShadowliftAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the shadowlift attitude and rate controller. Ideally it would
take attitude setpoints (`vehicle_attitude_setpoint`) or rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

Currently it is feeding the `manual_control_setpoint` topic directly to the actuators.

### Implementation
To reduce control latency, the module directly polls on the gyro topic published by the IMU driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("shadowlift_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * shadowlift attitude control app start / stop handling function
 */
extern "C" __EXPORT int shadowlift_att_control_main(int argc, char *argv[])
{
	return ShadowliftAttitudeControl::main(argc, argv);
}
