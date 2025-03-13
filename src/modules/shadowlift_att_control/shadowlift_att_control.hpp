#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_accel.h>

using namespace time_literals;

class ShadowliftAttitudeControl : public ModuleBase<ShadowliftAttitudeControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	ShadowliftAttitudeControl();

	virtual ~ShadowliftAttitudeControl();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

	bool init();

private:

	/**
	 * Check for parameter update and handle it.
	 */
	void parameter_update_poll();

	void publishTorqueSetpoint(const hrt_abstime &timestamp_sample);
	void publishThrustSetpoint(const hrt_abstime &timestamp_sample);
	void publishTorqueSetpoint2(const hrt_abstime &timestamp_sample, const float &current_yaw_rate);

	/**
	 * Apply PID controller for X and Y axes acceleration control
	 *
	 * @param accel_x Current X acceleration in m/s^2
	 * @param accel_y Current Y acceleration in m/s^2
	 * @param dt Time step in seconds
	 * @param x_output Output control value for X axis
	 * @param y_output Output control value for Y axis
	 * @return true if control is active, false if manual control should be used
	 */
	bool applyPidControl(float accel_x, float accel_y, float dt, float &x_output, float &y_output);

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};

	uORB::Publication<vehicle_thrust_setpoint_s>    _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>    _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

	hrt_abstime _last_run{0};
	manual_control_setpoint_s       _manual_control_setpoint{};
	vehicle_status_s                _vehicle_status{};
	//float torque_cmd[3] = {0,0,0};

	// PID controller state variables for X axis
	float _x_error_prev{0.0f};
	float _x_error_integral{0.0f};
	// PID controller state variables for Y axis
	float _y_error_prev{0.0f};
	float _y_error_integral{0.0f};

	perf_counter_t _loop_perf;

	DEFINE_PARAMETERS(
	(ParamFloat<px4::params::SL_YAWRATE_P>) _param_sl_yawrate_p,
	(ParamFloat<px4::params::SL_YAWMAX_P>) _param_sl_yawmax_p,
	(ParamInt<px4::params::SL_MODE>) _param_sl_mode,
	(ParamFloat<px4::params::SL_XACC_P>) _param_sl_xacc_p,
	(ParamFloat<px4::params::SL_XACC_I>) _param_sl_xacc_i,
	(ParamFloat<px4::params::SL_XACC_D>) _param_sl_xacc_d,
	(ParamFloat<px4::params::SL_YACC_P>) _param_sl_yacc_p,
	(ParamFloat<px4::params::SL_YACC_I>) _param_sl_yacc_i,
	(ParamFloat<px4::params::SL_YACC_D>) _param_sl_yacc_d,
	(ParamFloat<px4::params::SL_XY_MAXOUT>) _param_sl_xy_maxout
	)
};
