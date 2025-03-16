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


// In shadowlift_att_control.cpp add the implementation of the method:
bool ShadowliftAttitudeControl::applyPidControl(float accel_x, float accel_y, float dt, float &x_output, float &y_output)
{
    // Check if manual control is active (non-zero inputs indicate user control)
    const bool manual_control_active = (fabsf(_manual_control_setpoint.pitch) > 0.01f ||
                                       fabsf(_manual_control_setpoint.roll) > 0.01f);

    // If manual control is active, let it override the PID
    if (manual_control_active) {
        // Reset PID state to prevent windup
        _x_error_integral = 0.0f;
        _y_error_integral = 0.0f;
        _x_error_prev = 0.0f;
        _y_error_prev = 0.0f;
        return false;
    }

    // X-axis PID control (pitch control)
    float x_error = 0.0f - accel_x; // Target is zero acceleration

    // Anti-windup for integrator
    const float max_integral = _param_sl_xy_maxout.get() / _param_sl_xacc_i.get();
    _x_error_integral += x_error * dt;
    _x_error_integral = math::constrain(_x_error_integral, -max_integral, max_integral);

    // Calculate derivative (with filtering)
    float x_error_derivative = (x_error - _x_error_prev) / dt;
    _x_error_prev = x_error;

    // Calculate PID output
    x_output = _param_sl_xacc_p.get() * x_error +
              _param_sl_xacc_i.get() * _x_error_integral +
              _param_sl_xacc_d.get() * x_error_derivative;

    // Y-axis PID control (roll control)
    float y_error = 0.0f - accel_y; // Target is zero acceleration

    // Anti-windup for integrator
    const float max_integral_y = _param_sl_xy_maxout.get() / _param_sl_yacc_i.get();
    _y_error_integral += y_error * dt;
    _y_error_integral = math::constrain(_y_error_integral, -max_integral_y, max_integral_y);

    // Calculate derivative (with filtering)
    float y_error_derivative = (y_error - _y_error_prev) / dt;
    _y_error_prev = y_error;

    // Calculate PID output
    y_output = _param_sl_yacc_p.get() * y_error +
              _param_sl_yacc_i.get() * _y_error_integral +
              _param_sl_yacc_d.get() * y_error_derivative;

    // Limit maximum output
    const float max_output = _param_sl_xy_maxout.get();
    x_output = math::constrain(x_output, -max_output, max_output);
    y_output = math::constrain(y_output, -max_output, max_output);

    return true;
}

// Modify the publishThrustSetpoint method in shadowlift_att_control.cpp to use PID when needed:
void ShadowliftAttitudeControl::publishThrustSetpoint(const hrt_abstime &timestamp_sample)
{
    vehicle_thrust_setpoint_s v_thrust_sp = {};
    v_thrust_sp.timestamp = hrt_absolute_time();
    v_thrust_sp.timestamp_sample = timestamp_sample;

    // Check if we're armed
    if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
        // Get accelerometer data
        sensor_accel_s accel_data;
        if (_sensor_accel_sub.update(&accel_data)) {
            // Calculate time step
            const float dt = math::constrain(((accel_data.timestamp - _last_run) * 1e-6f), 0.0002f, 0.02f);

            // Store current acceleration for PID controller
            float accel_x = accel_data.x;
            float accel_y = accel_data.y;

            // PID outputs for X and Y axes
            float x_output = 0.0f;
            float y_output = 0.0f;

            // Apply PID control if no manual input
            bool pid_active = applyPidControl(accel_x, accel_y, dt, x_output, y_output);

            if (pid_active) {
                // Use PID outputs for thrust setpoints
                v_thrust_sp.xyz[0] = x_output; // Pitch control
                v_thrust_sp.xyz[1] = y_output; // Roll control
            } else {
                // Use manual control inputs
                v_thrust_sp.xyz[0] = _manual_control_setpoint.pitch;
                v_thrust_sp.xyz[1] = _manual_control_setpoint.roll;
            }

            _last_run = accel_data.timestamp;
        } else {
            // If no accelerometer data, fall back to manual control
            v_thrust_sp.xyz[0] = _manual_control_setpoint.pitch;
            v_thrust_sp.xyz[1] = _manual_control_setpoint.roll;
        }
    }

    _vehicle_thrust_setpoint_pub.publish(v_thrust_sp);
}



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
	if (!_vehicle_attitude_sub.registerCallback()) {
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

void ShadowliftAttitudeControl::updateHeadingSetpoint(float current_heading)
{
    // Check if the user is trying to control yaw
    const bool manual_yaw_control = (fabsf(_manual_control_setpoint.yaw) > 0.05f);

    // If user is controlling yaw, update the setpoint to current heading
    if (manual_yaw_control) {
        _yaw_heading_setpoint = current_heading;
        _heading_hold_enabled = false;
        _yaw_heading_error_integral = 0.0f; // Reset integrator
    } else if (!_heading_hold_enabled) {
        // If user has released yaw control, capture current heading as setpoint
        _yaw_heading_setpoint = current_heading;
        _heading_hold_enabled = true;
    }
    _yaw_heading_setpoint = .5f;
}

bool ShadowliftAttitudeControl::applyHeadingPidControl(float current_heading, float dt, float &yaw_output)
{
    // If heading hold is not enabled, don't apply PID
    if (!_heading_hold_enabled) {
        //return false;
    }

    // Calculate heading error (considering wrap-around)
    _yaw_heading_error = _yaw_heading_setpoint - current_heading;
    //_yaw_heading_error = _param_sl_param1.get();

    // Normalize the error to range [-PI, PI]
    if (_yaw_heading_error > M_PI_F) {
        _yaw_heading_error -= 2.0f * M_PI_F;
    } else if (_yaw_heading_error < -M_PI_F) {
        _yaw_heading_error += 2.0f * M_PI_F;
    }

    // Anti-windup for integrator
    const float max_integral = _param_sl_yaw_imax.get();
    _yaw_heading_error_integral += _yaw_heading_error * dt;
    _yaw_heading_error_integral = math::constrain(_yaw_heading_error_integral, -max_integral, max_integral);

    // Calculate derivative (with filtering)
    float yaw_error_derivative = (_yaw_heading_error - _yaw_heading_error_prev) / dt;
    _yaw_heading_error_prev = _yaw_heading_error;

    // Calculate PID output
    yaw_output = _param_sl_yaw_p.get() * _yaw_heading_error +
                _param_sl_yaw_i.get() * _yaw_heading_error_integral +
                _param_sl_yaw_d.get() * yaw_error_derivative;

    // Limit maximum output
    const float max_output = _param_sl_yawmax_p.get();
    yaw_output = math::constrain(yaw_output, -max_output, max_output);

    return true;
}

// Modify the publishTorqueSetpoint2 method in shadowlift_att_control.cpp to use heading PID:
void ShadowliftAttitudeControl::publishTorqueSetpoint3()
{
	vehicle_torque_setpoint_s v_torque_sp = {};

	// Get current vehicle attitude for heading control
	vehicle_attitude_s vehicle_attitude;
	//float current_heading = 0.0f;

	_vehicle_attitude_sub.update(&vehicle_attitude);
		PX4_INFO("Updating torque setpoint");


	if (_vehicle_attitude_sub.updated()) {

		PX4_INFO("Updating torque setpoint");
	    v_torque_sp.timestamp = hrt_absolute_time();
	    v_torque_sp.timestamp_sample = vehicle_attitude.timestamp_sample;
	    // Extract yaw (heading) from quaternion
	    const Quatf q(vehicle_attitude.q);
	    const Eulerf euler(q);
	    //current_heading = euler.psi();

	    // Calculate time step
	   // const float dt = math::constrain(((timestamp_sample - _last_yaw_run) * 1e-6f), 0.0002f, 0.02f);

	//     // Update heading setpoint based on user input
	//     updateHeadingSetpoint(current_heading);

	    // Apply PID control for heading
	    //float yaw_output = 0.0f;
	    //bool pid_active = applyHeadingPidControl(current_heading, dt, yaw_output);

	    v_torque_sp.xyz[2] = 0.1f;

	    // zero actuators if not armed
	    if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		v_torque_sp.xyz[0] = 0.f;
		v_torque_sp.xyz[1] = 0.f;

		bool pid_active = true;
		if (pid_active) {
		    // Use heading PID output
		    // v_torque_sp.xyz[2] = yaw_output;
		    v_torque_sp.xyz[2] = _param_sl_yaw_p.get() * _param_sl_param1.get();
		}
	  }
	    //_last_yaw_run = timestamp_sample;
	}

	_vehicle_torque_setpoint_pub.publish(v_torque_sp);
    }


void ShadowliftAttitudeControl::publishTorqueSetpoint2(const hrt_abstime &timestamp_sample, const float &current_yaw_rate)
{
    vehicle_torque_setpoint_s v_torque_sp = {};
    v_torque_sp.timestamp = hrt_absolute_time();
    v_torque_sp.timestamp_sample = timestamp_sample;

    // Get current vehicle attitude for heading control
    vehicle_attitude_s vehicle_attitude;
    float current_heading = 0.0f;

    //PX4_INFO("stupid log");

    if (_vehicle_attitude_sub.update(&vehicle_attitude)) {
	//PX4_INFO("stupid log2");
        // Extract yaw (heading) from quaternion
        const Quatf q(vehicle_attitude.q);
        const Eulerf euler(q);
        current_heading = euler.psi();

        // Calculate time step
        const float dt = math::constrain(((timestamp_sample - _last_yaw_run) * 1e-6f), 0.0002f, 0.02f);

        // Update heading setpoint based on user input
        updateHeadingSetpoint(current_heading);

        // Apply PID control for heading
        float yaw_output = 0.0f;
        bool pid_active = applyHeadingPidControl(current_heading, dt, yaw_output);

        // zero actuators if not armed
        if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
            v_torque_sp.xyz[0] = 0.f;
            v_torque_sp.xyz[1] = 0.f;

            if (pid_active) {
                // Use heading PID output
                 v_torque_sp.xyz[2] = yaw_output;
		//v_torque_sp.xyz[2] = _param_sl_yaw_p.get() * _param_sl_param1.get();
            } else {
                // Use manual yaw input or rate control
                float yaw_rate_sp = _manual_control_setpoint.yaw;
                float yaw_rate_err = yaw_rate_sp - current_yaw_rate;

                const float max_yaw_torque = _param_sl_yawmax_p.get();
                float yaw_torque = _param_sl_yawrate_p.get() * yaw_rate_err;
                yaw_torque = math::constrain(yaw_torque, -max_yaw_torque, max_yaw_torque);

                v_torque_sp.xyz[2] = yaw_torque;
            }
        }

        _last_yaw_run = timestamp_sample;
    } else {
        // Fallback to simple rate control if attitude data is not available
        // if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
        //     float yaw_rate_sp = _manual_control_setpoint.yaw;
        //     float yaw_rate_err = yaw_rate_sp - current_yaw_rate;

        //     const float max_yaw_torque = _param_sl_yawmax_p.get();
        //     float yaw_torque = _param_sl_yawrate_p.get() * yaw_rate_err;
        //     yaw_torque = math::constrain(yaw_torque, -max_yaw_torque, max_yaw_torque);

        //     v_torque_sp.xyz[0] = 0.f;
        //     v_torque_sp.xyz[1] = 0.f;
        //     v_torque_sp.xyz[2] = yaw_torque;
        // }
    }

    _vehicle_torque_setpoint_pub.publish(v_torque_sp);
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

	    _vehicle_angular_velocity_sub.update(&angular_velocity);
	    //if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		    /* run the rate controller immediately after a gyro update */
		    // Handle yaw control
		    publishTorqueSetpoint2(angular_velocity.timestamp_sample, angular_velocity.xyz[2]);
		//     publishTorqueSetpoint3();
		    // Handle x/y stabilization or manual control
		    //publishThrustSetpoint(angular_velocity.timestamp_sample);

		    /* check for updates in manual control topic */
		    _manual_control_setpoint_sub.update(&_manual_control_setpoint);

		    /* check for updates in vehicle status topic */
		    _vehicle_status_sub.update(&_vehicle_status);

		    parameter_update_poll();
	    //}

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
