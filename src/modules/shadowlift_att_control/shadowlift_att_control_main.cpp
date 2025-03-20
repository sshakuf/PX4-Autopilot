/**
 * @file shadowlift_att_control_main.cpp
 * shadowlift attitude controller.
 *
 * @author Gil Klar	<gil.klar@gmail.com>
 */

#include "shadowlift_att_control.hpp"

using namespace matrix;
int count = 0;
int count1 = 0;
float count2 = 0;
int count3 = 0;
float dbg_current_heading = 0.0f;
float dbg_heading_setpoint = 0.0f;
float dbg_heading_error = 0.0f;
float dbg_accel_x = 0.0f;
float dbg_accel_y = 0.0f;
float dbg_output_x = 0.0f;
float dbg_output_y = 0.0f;



ShadowliftAttitudeControl *ShadowliftAttitudeControl::_object = nullptr;


// In shadowlift_att_control.cpp add the implementation of the method:
bool ShadowliftAttitudeControl::applyPidControl(float accel_x, float accel_y,
                                                float dt, float &x_output,
                                                float &y_output) {
  // Check if manual control is active (non-zero inputs indicate user control)
  const bool manual_control_active =
      (fabsf(_manual_control_setpoint.pitch) > 0.05f ||
       fabsf(_manual_control_setpoint.roll) > 0.05f);

  // If manual control is active, let it override the PID
  if (manual_control_active || _param_sl_xy_pid_enabled.get() == 0) {
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
  _x_error_integral =
      math::constrain(_x_error_integral, -max_integral, max_integral);

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
  const float max_integral_y =
      _param_sl_xy_maxout.get() / _param_sl_yacc_i.get();
  _y_error_integral += y_error * dt;
  _y_error_integral =
      math::constrain(_y_error_integral, -max_integral_y, max_integral_y);

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

// Modify the publishThrustSetpoint method in shadowlift_att_control.cpp to use
// PID when needed:
void ShadowliftAttitudeControl::publishThrustSetpoint(const hrt_abstime &timestamp_sample) {
  vehicle_thrust_setpoint_s v_thrust_sp = {};
  v_thrust_sp.timestamp = hrt_absolute_time();
  v_thrust_sp.timestamp_sample = timestamp_sample;

  // Check if we're armed
  if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {

    const bool manual_control_active =
        (fabsf(_manual_control_setpoint.pitch) > 0.05f ||
         fabsf(_manual_control_setpoint.roll) > 0.05f);

    // If manual control is active, let it override the PID
    if (manual_control_active || _param_sl_xy_pid_enabled.get() == 0) {
      // Use manual control inputs
      v_thrust_sp.xyz[0] = _manual_control_setpoint.pitch;
      v_thrust_sp.xyz[1] = _manual_control_setpoint.roll;

      dbg_output_x = v_thrust_sp.xyz[0];
      dbg_output_y = v_thrust_sp.xyz[1];

      _vehicle_thrust_setpoint_pub.publish(v_thrust_sp);
    } else {
      // PID for x,y
      // Get accelerometer data
      sensor_accel_s accel_data;
      if (_sensor_accel_sub.update(&accel_data)) {
        // Calculate time step
        const float dt = math::constrain(
            ((accel_data.timestamp - _last_run) * 1e-6f), 0.0002f, 0.02f);

        // Store current acceleration for PID controller
        float accel_x = accel_data.x;
        float accel_y = accel_data.y;

        // PID outputs for X and Y axes
        float x_output = 0.0f;
        float y_output = 0.0f;

        // Apply PID control if no manual input
	applyPidControl(accel_x, accel_y, dt, x_output, y_output);

	// dbg_output_x = x_output;
	// dbg_output_y = y_output;
	dbg_accel_x = accel_x;
	dbg_accel_y = accel_y;

        // Use PID outputs for thrust setpoints
        v_thrust_sp.xyz[0] = x_output; // Pitch control
        v_thrust_sp.xyz[1] = y_output; // Roll control
	dbg_output_x = v_thrust_sp.xyz[0];
	dbg_output_y = v_thrust_sp.xyz[1];

        _last_run = accel_data.timestamp;
	// only if we have new accelometer data i want to update the thrust.
        _vehicle_thrust_setpoint_pub.publish(v_thrust_sp);
      }
    }
  }
}

ShadowliftAttitudeControl::ShadowliftAttitudeControl()
    : ModuleParams(nullptr),
      WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
      _loop_perf(perf_alloc(PC_ELAPSED, "shadowlift_att_control")) {}

ShadowliftAttitudeControl::~ShadowliftAttitudeControl() {
  perf_free(_loop_perf);
}

bool ShadowliftAttitudeControl::init() {
  if (!_vehicle_angular_velocity_sub.registerCallback()) {
    PX4_ERR("callback registration failed");
    return false;
  }
  if (!_vehicle_attitude_sub.registerCallback()) {
    PX4_ERR("callback registration failed");
    return false;
  }
  if (!_sensor_accel_sub.registerCallback()) {
    PX4_ERR("Accelerometer callback registration failed");
    return false;
  }
  return true;
}

void ShadowliftAttitudeControl::parameter_update_poll() {
  // check for parameter updates
  if (_parameter_update_sub.updated()) {
    // clear update
    parameter_update_s pupdate;
    _parameter_update_sub.copy(&pupdate);

    // update parameters from storage
    updateParams();
  }
}

void ShadowliftAttitudeControl::publishTorqueSetpoint(
    const hrt_abstime &timestamp_sample) {
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

void ShadowliftAttitudeControl::updateHeadingSetpoint(float current_heading,
                                                      float dt) {
  // Check if the user is trying to control yaw
  const bool manual_yaw_control = (fabsf(_manual_control_setpoint.yaw) > 0.05f);

  // If user is controlling yaw, update the setpoint incrementally based on
  // manual input
  if (manual_yaw_control) {
    // _yaw_heading_setpoint += _manual_control_setpoint.yaw *
    // _param_sl_yawrate_p.get() * dt;
    _yaw_heading_setpoint = current_heading + _manual_control_setpoint.yaw *
                                                  _param_sl_yawrate_p.get() *
                                                  dt;
    _heading_hold_enabled = false;
  } else if (!_heading_hold_enabled) {
    // If user has released yaw control, capture current heading as setpoint
   // _yaw_heading_setpoint = current_heading;
    _heading_hold_enabled = true;
  }
  //_yaw_heading_setpoint = .5f;
}

bool ShadowliftAttitudeControl::applyHeadingPidControl(float current_heading,
                                                       float dt,
                                                       float &yaw_output) {
  // If heading hold is not enabled, don't apply PID
  if (!_heading_hold_enabled || _param_sl_yaw_pid_enabled.get() == 0) {
    return false;
  }

  // Calculate heading error (considering wrap-around)
  _yaw_heading_error = _yaw_heading_setpoint - current_heading;
  dbg_current_heading = current_heading;
  dbg_heading_setpoint = _yaw_heading_setpoint;
  dbg_heading_error = _yaw_heading_error;


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
  _yaw_heading_error_integral =
      math::constrain(_yaw_heading_error_integral, -max_integral, max_integral);

  // Calculate derivative (with filtering)
  float yaw_error_derivative =
      (_yaw_heading_error - _yaw_heading_error_prev) / dt;
  _yaw_heading_error_prev = _yaw_heading_error;

  // Calculate PID output
  yaw_output = _param_sl_yaw_p.get() * _yaw_heading_error +
               _param_sl_yaw_i.get() * _yaw_heading_error_integral +
               _param_sl_yaw_d.get() * yaw_error_derivative;

  // fix dead zone in the engine
  if (yaw_output > 0)
  {
	yaw_output += 0.05f;
  } else if(yaw_output < 0)
  {
	yaw_output -= 0.05f;
  }

  // Limit maximum output
  const float max_output = _param_sl_yawmax_p.get();
  yaw_output = math::constrain(yaw_output, -max_output, max_output);

  return true;
}

void ShadowliftAttitudeControl::publishTorqueSetpoint2(
    const hrt_abstime &timestamp_sample, const float &current_yaw_rate) {
  vehicle_torque_setpoint_s v_torque_sp = {};
  v_torque_sp.timestamp = hrt_absolute_time();
  v_torque_sp.timestamp_sample = timestamp_sample;

  // Get current vehicle attitude for heading control
  vehicle_attitude_s vehicle_attitude;
  float current_heading = 0.0f;

  if (_vehicle_attitude_sub.update(&vehicle_attitude)) {
    count++;
    // Extract yaw (heading) from quaternion
    const Quatf q(vehicle_attitude.q);
    const Eulerf euler(q);
    current_heading = euler.psi();

    // Calculate time step
    const float dt = math::constrain(
        ((timestamp_sample - _last_yaw_run) * 1e-6f), 0.0002f, 0.02f);

    // Update heading setpoint based on user input
    updateHeadingSetpoint(current_heading, dt);

    // Apply PID control for heading
    float yaw_output = 0.0f;
    bool pid_active = applyHeadingPidControl(current_heading, dt, yaw_output);

    // zero actuators if not armed
    if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
      v_torque_sp.xyz[0] = 0.f;
      v_torque_sp.xyz[1] = 0.f;

      if (pid_active) {
        count1++;
        // Use heading PID output
        v_torque_sp.xyz[2] = yaw_output;
        count2 = yaw_output;
      } else {
        // when pid is not active, use manual control
        // Use manual yaw input with scaling
        float yaw_rate_sp = _manual_control_setpoint.yaw;
        float yaw_torque =
            yaw_rate_sp * _param_sl_yawrate_p.get(); // Scale with parameter

        // Apply constraints
        const float max_yaw_torque = _param_sl_yawmax_p.get();
        yaw_torque =
            math::constrain(yaw_torque, -max_yaw_torque, max_yaw_torque);

        v_torque_sp.xyz[2] = yaw_torque; // Apply scaled yaw control
      }
    }

    _last_yaw_run = timestamp_sample;
    _vehicle_torque_setpoint_pub.publish(v_torque_sp);
  }
}

double quaternionToYaw(double q_w, double q_x, double q_y, double q_z) {
  return std::atan2(2.0 * (q_w * q_z + q_x * q_y),
                    1.0 - 2.0 * (q_y * q_y + q_z * q_z));
}

void ShadowliftAttitudeControl::Run() {
  if (should_exit()) {
    _vehicle_angular_velocity_sub.unregisterCallback();
    exit_and_cleanup();
    return;
  }

  perf_begin(_loop_perf);

  /* run controller on gyro changes */
  vehicle_angular_velocity_s angular_velocity;

  _vehicle_angular_velocity_sub.update(&angular_velocity);
  // if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
  /* run the rate controller immediately after a gyro update */
  // Handle yaw control
  /* check for updates in manual control topic */
  _manual_control_setpoint_sub.update(&_manual_control_setpoint);

  publishTorqueSetpoint2(angular_velocity.timestamp_sample,
                         angular_velocity.xyz[2]);
  //     publishTorqueSetpoint3();
  // Handle x/y stabilization or manual control
  publishThrustSetpoint(angular_velocity.timestamp_sample);

  /* check for updates in vehicle status topic */
  _vehicle_status_sub.update(&_vehicle_status);

  parameter_update_poll();
  //}

  perf_end(_loop_perf);
}

int ShadowliftAttitudeControl::task_spawn(int argc, char *argv[]) {
  ShadowliftAttitudeControl *instance = new ShadowliftAttitudeControl();

  if (instance) {
    _object = instance;
    _task_id = task_id_is_work_queue;

    if (instance->init()) {
      return PX4_OK;
    }

  } else {
    PX4_ERR("alloc failed");
  }

  delete instance;
  _object = nullptr;
  _task_id = -1;

  return PX4_ERROR;
}

int ShadowliftAttitudeControl::print_status() {

  //   PX4_INFO("_yaw_heading_setpoint %f", (double)_yaw_heading_setpoint);
//   PX4_INFO("_______________heading_____________");
  PX4_INFO("current_heading  | heading_setpoint | heading_error  | yaw_output ");
  PX4_INFO(" %f |  %f |  %f |  %f", (double)dbg_current_heading, (double)dbg_heading_setpoint, (double)dbg_heading_error, (double)count2);

  //perf_print_counter(_loop_perf);

  return 0;
}
void ShadowliftAttitudeControl::do2(int loop_count)
{
    PX4_INFO("accel_x | accel_y  | output_x | output_y ");
    for (int i = 0; i < loop_count; i++) {
	PX4_INFO(" %f |  %f |  %f |  %f", (double)dbg_accel_x, (double)dbg_accel_y, (double)dbg_output_x, (double)dbg_output_y);
	      px4_usleep(100000); // Sleep for 100ms between iterations
    }

}

void ShadowliftAttitudeControl::do1(int loop_count)
{
    PX4_INFO("current_heading  | heading_setpoint | heading_error  | yaw_output ");
    for (int i = 0; i < loop_count; i++) {
	PX4_INFO(" %f |  %f |  %f |  %f", (double)dbg_current_heading*180/M_PI, (double)dbg_heading_setpoint*180/M_PI, (double)dbg_heading_error*180/M_PI, (double)count2);
	      px4_usleep(500000); // Sleep for 500ms between iterations
    }

}

int ShadowliftAttitudeControl::custom_command(int argc, char *argv[])
{
	ShadowliftAttitudeControl *instance = _object;
	int loop_count = 10; // Default value

	// If argv[2] exists, convert it to an integer
	if (argc >= 2) {
	    loop_count = atoi(argv[1]);
	    if (loop_count <= 0) {
		PX4_WARN("Invalid loop count, using default 10");
		loop_count = 10;
	    }
	}

	if (argc >= 1) {
        if (strcmp(argv[0], "do1") == 0) {
                instance->do1(loop_count);
                return PX4_OK;
        }
	if (strcmp(argv[0], "do2") == 0) {
                instance->do2(loop_count);
                return PX4_OK;
        }

	if (strcmp(argv[0], "status") == 0) {
		instance->do1(1);
	}
    }

    return print_usage("Unknown command");
}

int ShadowliftAttitudeControl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION("Shadowlift Attitude Control module.");
    PRINT_MODULE_USAGE_NAME("shadowlift_att_ctrl", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_ARG("do1", "Example command to modify a class variable", false);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

/**
 * shadowlift attitude control app start / stop handling function
 */
extern "C" __EXPORT int shadowlift_att_control_main(int argc, char *argv[]) {
  return ShadowliftAttitudeControl::main(argc, argv);
}
