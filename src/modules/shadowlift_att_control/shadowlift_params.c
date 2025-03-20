/**
 * @file shadowlift_params.c
 * Parameters for X/Y acceleration PID controllers
 *
 * @author sshakuf
 */

 /**
 * Yaw rate proportional gain
 *
 * Proportional gain for the yaw rate controller.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.01
 * @group Shadowlift Control
 */
PARAM_DEFINE_FLOAT(SL_YAWRATE_P, 0.3f);

/**
 * Maximum yaw rate
 *
 * Maximum allowed yaw rate in degrees per second.
 *
 * @min 0.0
 * @max 360.0
 * @unit deg/s
 * @decimal 1
 * @increment 1
 * @group Shadowlift Control
 */
PARAM_DEFINE_FLOAT(SL_YAWMAX_P, 45.0f);

/**
 * Shadowlift control mode
 *
 * Selects the control mode for the Shadowlift system.
 * 0: Standard mode
 * 1: Alternative mode
 * 2: Advanced mode
 *
 * @min 0
 * @max 2
 * @value 0 Standard
 * @value 1 Alternative
 * @value 2 Advanced
 * @group Shadowlift Control
 */
PARAM_DEFINE_INT32(SL_MODE, 0);


/**
 * X-axis acceleration controller P gain
 *
 * Proportional gain for X-axis acceleration control.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.001
 * @group Shadowlift Control
 */
PARAM_DEFINE_FLOAT(SL_XACC_P, 0.1f);

/**
 * X-axis acceleration controller I gain
 *
 * Integral gain for X-axis acceleration control.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 3
 * @increment 0.001
 * @group Shadowlift Control
 */
PARAM_DEFINE_FLOAT(SL_XACC_I, 0.01f);

/**
 * X-axis acceleration controller D gain
 *
 * Derivative gain for X-axis acceleration control.
 *
 * @min 0.0
 * @max 0.5
 * @decimal 4
 * @increment 0.0001
 * @group Shadowlift Control
 */
PARAM_DEFINE_FLOAT(SL_XACC_D, 0.001f);

/**
 * Y-axis acceleration controller P gain
 *
 * Proportional gain for Y-axis acceleration control.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.001
 * @group Shadowlift Control
 */
PARAM_DEFINE_FLOAT(SL_YACC_P, 0.1f);

/**
 * Y-axis acceleration controller I gain
 *
 * Integral gain for Y-axis acceleration control.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 3
 * @increment 0.001
 * @group Shadowlift Control
 */
PARAM_DEFINE_FLOAT(SL_YACC_I, 0.01f);

/**
 * Y-axis acceleration controller D gain
 *
 * Derivative gain for Y-axis acceleration control.
 *
 * @min 0.0
 * @max 0.5
 * @decimal 4
 * @increment 0.0001
 * @group Shadowlift Control
 */
PARAM_DEFINE_FLOAT(SL_YACC_D, 0.001f);

/**
 * Maximum output value for X/Y acceleration controllers
 *
 * This limits the maximum control output from the PID controllers.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Shadowlift Control
 */
PARAM_DEFINE_FLOAT(SL_XY_MAXOUT, 0.5f);


# Add these to your parameters definition file (likely in shadowlift_params.c)

/**
 * Yaw heading P gain
 *
 * Proportional gain for yaw heading control.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group Shadowlift Attitude Control
 */
PARAM_DEFINE_FLOAT(SL_YAW_P, 2.0f);

/**
 * Yaw heading I gain
 *
 * Integral gain for yaw heading control.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Shadowlift Attitude Control
 */
PARAM_DEFINE_FLOAT(SL_YAW_I, 0.1f);

/**
 * Yaw heading D gain
 *
 * Derivative gain for yaw heading control.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 3
 * @increment 0.01
 * @group Shadowlift Attitude Control
 */
PARAM_DEFINE_FLOAT(SL_YAW_D, 0.0f);

/**
 * Yaw heading integrator limit
 *
 * Maximum value for the yaw heading integrator.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Shadowlift Attitude Control
 */
PARAM_DEFINE_FLOAT(SL_YAW_IMAX, 0.3f);
PARAM_DEFINE_FLOAT(SL_PARAM1, 0.0f);
PARAM_DEFINE_FLOAT(SL_PARAM2, 0.0f);
PARAM_DEFINE_INT32(SL_YAW_PID_EN, 1);
PARAM_DEFINE_INT32(SL_XY_PID_EN, 1);
