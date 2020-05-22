/****************************************************************************
 *
 *   Copyright (c) 2015, 2016 NUS UAV research team. All rights reserved.
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
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 * The controller has two loops: P loop for angular error and PD loop for angular rate error.
 * Desired rotation calculated keeping in mind that yaw response is normally slower than roll/pitch.
 * For small deviations controller rotates copter to have shortest path of thrust vector and independently rotates around yaw,
 * so actual rotation axis is not constant. For large deviations controller rotates copter around fixed axis.
 * These two approaches fused seamlessly with weight depending on angular error.
 * When thrust vector directed near-horizontally (e.g. roll ~= PI/2) yaw setpoint ignored because of singularity.
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 * If rotation matrix setpoint is invalid it will be generated from Euler angles for compatibility with old position controllers.
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/motor_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/odroid_status.h>
#include <uORB/topics/odroid_preflight_status.h>
//#include <uORB/topics/vehicle_serial_command.h>


#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>

/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */

extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[]);

#define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    0.2f
#define ANGLE_I_LIMIT	1.0f
//#define QUAD  //if define, the code is for quadrotor
//#define HEX   //if define, the code is for HEX
#define OCT		//if define, the code is for OCT
//#define FIX_POS_SHUTTER


class MulticopterAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	MulticopterAttitudeControl();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~MulticopterAttitudeControl();

	/**
	 * Start the sensors task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool	_task_should_exit;		/**< if true, sensor task should exit */
	int		_control_task;			/**< task handle for sensor task */

	int		_v_att_sub;				/**< vehicle attitude subscription */
	int		_v_att_sp_sub;			/**< vehicle attitude setpoint subscription */
	int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */
	int		_v_control_mode_sub;	/**< vehicle control mode subscription */
	int		_params_sub;			/**< parameter updates subscription */
	int		_manual_control_sp_sub;	/**< manual control setpoint subscription */
	int		_armed_sub;				/**< arming status subscription */
	int     _home_sub;              /**< home position subscription */
	int		_local_pos_sub;	    /**< local position setpoint */
	int     _gps_sub;				/**< GPS sub*/
	int 	_battery_sub;
	int		_sensor_combined_sub;
	int		_v_status_sub;
//	int		_pos_sp_triplet_sub;
//	int     _odroid_status_sub;
	int     _odroid_preflight_status_sub;
//	int     _vehicle_serial_command_sub;

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_v_rates_sp_pub;		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */
	orb_advert_t    _actuators_1_pub;
	orb_advert_t    _motor_status_pub;

	bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

	struct vehicle_attitude_s			_v_att;				/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_v_att_sp;			/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp;	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode;	/**< vehicle control mode */
	struct actuator_controls_s			_actuators;			/**< actuator controls */
	struct actuator_controls_s			_actuators1;
	struct actuator_armed_s				_armed;				/**< actuator arming status */
	struct home_position_s              _home;
	struct vehicle_local_position_s    _local_pos;
	struct vehicle_gps_position_s       _gps;
	struct battery_status_s				_battery;
	struct motor_status_s				_motor_status;
	struct sensor_combined_s			_sensor_combined;
	struct vehicle_status_s				_v_status;
//	struct position_setpoint_triplet_s		_pos_sp_triplet;
	//struct odroid_status_s                   _odroid_status;
	struct odroid_preflight_status_s _odroid_preflight_status;
//	struct vehicle_serial_command_s        _vehicle_serial_cmd;

	bool gps_valid;
	bool home_valid;
	int64_t armed_timing;
	bool  was_armed;
	float pitch_sp_pre;
	float roll_sp_pre;
	float yaw_sp_pre;
	float yaw_sp_rate_pre;
	float temp_filtered;
	float pressure_filtered;


	perf_counter_t	_loop_perf;			/**< loop performance counter */

	math::Vector<3>		_rates_prev;	/**< angular rates on previous step */
	math::Vector<3>		_rates_sp;		/**< angular rates setpoint */
	math::Vector<3>		_rates_d_prev;
	math::Vector<3>		_rates_d;
	math::Vector<3>		_angle_int;		/**< angular rates integral error */
	float				_thrust_sp;		/**< thrust setpoint */
	math::Vector<3>		_att_control;	/**< attitude control vector */
#ifdef QUAD
	math::Matrix<4,4>     _u_omega2;           /**< Matrix to convert the u to omega2*/
#endif
#ifdef HEX
	math::Matrix<6,4>	  _u_omega2;
#endif
#ifdef OCT
	math::Matrix<8,4>     _u_omega2;
#endif

	math::Vector<4>     _u;              //* the desire control input */
	math::Vector<3>		_angle_error;   /** to indicate the angle error */

	math::Matrix<3, 3>  _I;				/**< identity matrix */

	bool	_reset_yaw_sp;			/**< reset yaw setpoint flag */

	struct {
		param_t roll_p;
		param_t roll_rate_p;
		param_t roll_i;
		param_t roll_rate_d;
		param_t pitch_p;
		param_t pitch_rate_p;
		param_t pitch_i;
		param_t pitch_rate_d;
		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_i;
		param_t yaw_rate_d;
		param_t yaw_ff;
		param_t yaw_rate_max;

		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_yaw_max;
		param_t acro_roll_max;
		param_t acro_pitch_max;
		param_t acro_yaw_max;
		param_t fixed_time_shutter;
		param_t check_current;
		param_t check_on_off;
		param_t turf_club;
		param_t Ix;
		param_t Iy;
		param_t	Iz;
		param_t Kt1;
		param_t Kt2;
		param_t	Kq1;
		param_t Kq2;
		param_t Lm;
		param_t Cm;
		param_t Ct;
		param_t M;
		param_t temp;
	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		math::Vector<3> att_p;					/**< P gain for angular error */
		math::Vector<3> rate_p;				/**< P gain for angular rate error */
		math::Vector<3> angle_i;				/**< I gain for angular rate error */
		math::Vector<3> rate_d;				/**< D gain for angular rate error */
		float yaw_ff;						/**< yaw control feed-forward */
		float yaw_rate_max;					/**< max yaw rate */

		float man_roll_max;
		float man_pitch_max;
		float man_yaw_max;
		math::Vector<3> acro_rate_max;		/**< max attitude rates in acro mode */
		float fixed_time_shutter;
		float check_current;
		float check_on_off;
		float turf_club;
		float Kt1,Kt2;
		float temp;
	}		_params;



	 float max_1_current;
	 float max_2_current;
	 float max_3_current;
	 float max_4_current;
	 float ave_current;
	 bool  motor_checked;
	 int   loop_count;
	 float Ix,Iy,Iz;
	 float Kt1,Kt2,Kq1,Kq2;
	 float Lm,Mg;
	 float Cm,Ct;
	 float min_omega_square;
	 float max_omega_square;
	 float air_density_ratio;

	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update();

	/**
	 * Check for parameter update and handle it.
	 */
	void		parameter_update_poll();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

	void		vehicle_status_poll();

	/**
	 * Check for attitude setpoint updates.
	 */
	void		vehicle_attitude_setpoint_poll();

	/**
	 * Check for rates setpoint updates.
	 */
	void		vehicle_rates_setpoint_poll();

	/**
	 * Check for arming status updates.
	 */
	void		arming_status_poll();

	/**
	 * Check for home position updates.
	 */
    void		home_position_poll();


	/**
	 * Check for local position updates.
	 */
    void		local_position_poll();


    void        gps_poll();

    void 		battery_poll();

    void 		sensor_combined_poll();

//    void 		position_triplet_poll();

    void        odroid_preflight_status_poll();

//   void        vehilcle_serial_comand_poll();

    float	 	scale_control(float ctl, float end, float dz);

    bool		check_motor();

    void        calculate_delta();

    void        pre_arm_check();

	/**
	 * Attitude controller.
	 */
	void		control_attitude(float dt);

	/**
	 * Attitude rates controller.
	 */
	void		control_attitude_rates(float dt);

	void        update_u_omega2();



	float constrain_ref(float x, float cons);

	float limit_range(float x, float min, float max);

	float limit_acc(float cur,float pre,float max_acc,float dt);


	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main();
};

namespace mc_att_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

MulticopterAttitudeControl	*g_control;
}

float MulticopterAttitudeControl::constrain_ref(float x, float cons)
{
	if(x>cons)
		x = cons;
	else if (x< - cons)
		x = - cons;
	return x;
}

float MulticopterAttitudeControl::limit_range(float x, float min, float max)
{
	if(x>max)
		x = max;
	if(x<min)
		x = min;
	return x;

}

float MulticopterAttitudeControl::limit_acc(float cur, float pre, float max_acc, float dt)
{
	float error = cur-pre;
	float error_dot = error/dt;
	error_dot = constrain_ref(error_dot,max_acc);
	return pre + error_dot * dt;
}




MulticopterAttitudeControl::MulticopterAttitudeControl() :

	_task_should_exit(false),
	_control_task(-1),

/* subscriptions */
	_v_att_sub(-1),
	_v_att_sp_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_armed_sub(-1),

/* publications */
	_att_sp_pub(-1),
	_v_rates_sp_pub(-1),
	_actuators_0_pub(-1),
	_actuators_1_pub(-1),
    _motor_status_pub(-1),
	_actuators_0_circuit_breaker_enabled(false),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control"))

{
	memset(&_v_att, 0, sizeof(_v_att));
	memset(&_v_att_sp, 0, sizeof(_v_att_sp));
	memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
	memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
	memset(&_v_control_mode, 0, sizeof(_v_control_mode));
	memset(&_actuators, 0, sizeof(_actuators));
	memset(&_armed, 0, sizeof(_armed));
	memset(&_v_status,0,sizeof(_v_status));
//	memset(&_pos_sp_triplet,0,sizeof(_pos_sp_triplet));
	memset(&_odroid_preflight_status,0,sizeof(_odroid_preflight_status));
//	memset(&_vehicle_serial_cmd,0,sizeof(_vehicle_serial_cmd));

	_params.att_p.zero();
	_params.rate_p.zero();
	_params.angle_i.zero();
	_params.rate_d.zero();
	_params.yaw_ff = 0.0f;
	_params.yaw_rate_max = 0.0f;
	_params.man_roll_max = 0.0f;
	_params.man_pitch_max = 0.0f;
	_params.man_yaw_max = 0.0f;
	_params.acro_rate_max.zero();
	_params.fixed_time_shutter = 0.0f;
	_params.check_current = 0.0f;
	_params.check_on_off = 0.0f;
	_params.turf_club = 0.0f;

	_rates_prev.zero();
	_rates_sp.zero();
	_angle_int.zero();
	_rates_d_prev.zero();
	_rates_d.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();
	_u_omega2.identity();
	_u.zero();
	_angle_error.zero();

	_I.identity();

	_params_handles.roll_p			= 	param_find("MC_ROLL_P");
	_params_handles.roll_rate_p		= 	param_find("MC_ROLLRATE_P");
	_params_handles.roll_i		= 	param_find("MC_ROLL_I");
	_params_handles.roll_rate_d		= 	param_find("MC_ROLLRATE_D");
	_params_handles.pitch_p			= 	param_find("MC_PITCH_P");
	_params_handles.pitch_rate_p	= 	param_find("MC_PITCHRATE_P");
	_params_handles.pitch_i	= 	param_find("MC_PITCH_I");
	_params_handles.pitch_rate_d	= 	param_find("MC_PITCHRATE_D");
	_params_handles.yaw_p			=	param_find("MC_YAW_P");
	_params_handles.yaw_rate_p		= 	param_find("MC_YAWRATE_P");
	_params_handles.yaw_i		= 	param_find("MC_YAW_I");
	_params_handles.yaw_rate_d		= 	param_find("MC_YAWRATE_D");
	_params_handles.yaw_ff			= 	param_find("MC_YAW_FF");
	_params_handles.yaw_rate_max	= 	param_find("MC_YAWRATE_MAX");
	_params_handles.man_roll_max	= 	param_find("MC_MAN_R_MAX");
	_params_handles.man_pitch_max	= 	param_find("MC_MAN_P_MAX");
	_params_handles.man_yaw_max		= 	param_find("MC_MAN_Y_MAX");
	_params_handles.acro_roll_max	= 	param_find("MC_ACRO_R_MAX");
	_params_handles.acro_pitch_max	= 	param_find("MC_ACRO_P_MAX");
	_params_handles.acro_yaw_max		= 	param_find("MC_ACRO_Y_MAX");
	_params_handles.fixed_time_shutter = param_find("FIX_TIME_SHUT");
	_params_handles.check_current = param_find("CHECK_CURRENT");
	_params_handles.check_on_off = param_find("CHECK_ON_OFF");
	_params_handles.turf_club = param_find("TURF_CLUB");
	_params_handles.Ix = param_find("UAV_IX");
	_params_handles.Iy = param_find("UAV_IY");
	_params_handles.Iz = param_find("UAV_IZ");
	_params_handles.Kt1 = param_find("UAV_KT1");
	_params_handles.Kt2 = param_find("UAV_KT2");
	_params_handles.Kq1 = param_find("UAV_KQ1");
	_params_handles.Kq2 = param_find("UAV_KQ2");
	_params_handles.Lm = param_find("UAV_LM");
	_params_handles.Cm = param_find("UAV_CM");
	_params_handles.Ct = param_find("UAV_CT");
	_params_handles.M = param_find("UAV_MASS");
	_params_handles.temp = param_find("UAV_TEMP");

	/* fetch initial parameter values */
	parameters_update();
}

MulticopterAttitudeControl::~MulticopterAttitudeControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	mc_att_control::g_control = nullptr;
}

int
MulticopterAttitudeControl::parameters_update()
{
	float v;

	/* roll gains */
	param_get(_params_handles.roll_p, &v);
	_params.att_p(0) = v;
	param_get(_params_handles.roll_rate_p, &v);
	_params.rate_p(0) = v;
	param_get(_params_handles.roll_i, &v);
	_params.angle_i(0) = v;
	param_get(_params_handles.roll_rate_d, &v);
	_params.rate_d(0) = v;

	/* pitch gains */
	param_get(_params_handles.pitch_p, &v);
	_params.att_p(1) = v;
	param_get(_params_handles.pitch_rate_p, &v);
	_params.rate_p(1) = v;
	param_get(_params_handles.pitch_i, &v);
	_params.angle_i(1) = v;
	param_get(_params_handles.pitch_rate_d, &v);
	_params.rate_d(1) = v;

	/* yaw gains */
	param_get(_params_handles.yaw_p, &v);
	_params.att_p(2) = v;
	param_get(_params_handles.yaw_rate_p, &v);
	_params.rate_p(2) = v;
	param_get(_params_handles.yaw_i, &v);
	_params.angle_i(2) = v;
	param_get(_params_handles.yaw_rate_d, &v);
	_params.rate_d(2) = v;

	param_get(_params_handles.yaw_ff, &_params.yaw_ff);
	param_get(_params_handles.yaw_rate_max, &_params.yaw_rate_max);
	_params.yaw_rate_max = math::radians(_params.yaw_rate_max);

	/* manual control scale */
	param_get(_params_handles.man_roll_max, &_params.man_roll_max);
	param_get(_params_handles.man_pitch_max, &_params.man_pitch_max);
	param_get(_params_handles.man_yaw_max, &_params.man_yaw_max);
	_params.man_roll_max = math::radians(_params.man_roll_max);
	_params.man_pitch_max = math::radians(_params.man_pitch_max);
	_params.man_yaw_max = math::radians(_params.man_yaw_max);

	/* acro control scale */
	param_get(_params_handles.acro_roll_max, &v);
	_params.acro_rate_max(0) = math::radians(v);
	param_get(_params_handles.acro_pitch_max, &v);
	_params.acro_rate_max(1) = math::radians(v);
	param_get(_params_handles.acro_yaw_max, &v);
	_params.acro_rate_max(2) = math::radians(v);

	param_get(_params_handles.fixed_time_shutter, &_params.fixed_time_shutter);
	param_get(_params_handles.check_current, &_params.check_current);
	param_get(_params_handles.check_on_off, &_params.check_on_off);
	param_get(_params_handles.turf_club,&_params.turf_club);
	param_get(_params_handles.Ix,&Ix);
	param_get(_params_handles.Iy,&Iy);
	param_get(_params_handles.Iz,&Iz);
	param_get(_params_handles.Kt1,&_params.Kt1);
	param_get(_params_handles.Kt2,&_params.Kt2);
	param_get(_params_handles.Kq1,&Kq1);
	param_get(_params_handles.Kq2,&Kq2);
	param_get(_params_handles.Lm,&Lm);
	float M;
	param_get(_params_handles.M,&M);
	Mg = M * 9.8f;
	param_get(_params_handles.Cm,&Cm);
	param_get(_params_handles.Ct,&Ct);
	param_get(_params_handles.temp,&_params.temp);
	update_u_omega2();

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	return OK;
}

void MulticopterAttitudeControl::update_u_omega2()
{

	Kt1 = _params.Kt1 * air_density_ratio;
	Kt2 = _params.Kt2 * air_density_ratio;

#ifdef QUAD
	_u_omega2(0,0) = -1/(2.828f*Lm*Kt1);
	_u_omega2(0,1) =  1/(2.828f*Lm*Kt1);
	_u_omega2(0,2) =  0.5f*Kt2/(Kt1*Kq2+Kt2*Kq1);
	_u_omega2(0,3) =  0.5f*Kq2/(Kt1*Kq2+Kt2*Kq1);

	_u_omega2(1,0) =  1/(2.828f*Lm*Kt1);
	_u_omega2(1,1) = - 1/(2.828f*Lm*Kt1);
	_u_omega2(1,2) =  0.5f*Kt2/(Kt1*Kq2+Kt2*Kq1);
	_u_omega2(1,3) =  0.5f*Kq2/(Kt1*Kq2+Kt2*Kq1);

	_u_omega2(2,0) =  1/(2.828f*Lm*Kt2);
	_u_omega2(2,1) =  1/(2.828f*Lm*Kt2);
	_u_omega2(2,2) =  -0.5f*Kt1/(Kt1*Kq2+Kt2*Kq1);
	_u_omega2(2,3) =   0.5f*Kq1/(Kt1*Kq2+Kt2*Kq1);


	_u_omega2(3,0) = -1/(2.828f*Lm*Kt2);
	_u_omega2(3,1) = -1/(2.828f*Lm*Kt2);
	_u_omega2(3,2) = -0.5f*Kt1/(Kt1*Kq2+Kt2*Kq1);
	_u_omega2(3,3) =  0.5f*Kq1/(Kt1*Kq2+Kt2*Kq1);
#endif

#ifdef HEX
	_u_omega2(0,0) = -1/(4*Kt1*Lm);
	_u_omega2(0,1) = 0.2877f/(Kt1*Lm);
	_u_omega2(0,2) = 1/(6*Kq1);
	_u_omega2(0,3) = 1/(6*Kt1);

	_u_omega2(1,0) = -1/(4*Kt2*Lm);
	_u_omega2(1,1) = 0.0f;
	_u_omega2(1,2) = -1/(6*Kq1);
	_u_omega2(1,3) = 1/(6*Kt2);

	_u_omega2(2,0) = -1/(4*Kt1*Lm);
	_u_omega2(2,1) = -0.2877f/(Kt1*Lm);
	_u_omega2(2,2) = 1/(6*Kq1);
	_u_omega2(2,3) = 1/(6*Kt1);

	_u_omega2(3,0) = 1/(4*Kt2*Lm);
	_u_omega2(3,1) = -0.2877f/(Kt2*Lm);
	_u_omega2(3,2) = -1/(6*Kq1);
	_u_omega2(3,3) = 1/(6*Kt2);

	_u_omega2(4,0) = 1/(4*Kt1*Lm);
	_u_omega2(4,1) = 0;
	_u_omega2(4,2) = 1/(6*Kq1);
	_u_omega2(4,3) = 1/(6*Kt1);

	_u_omega2(5,0) = 1/(4*Kt2*Lm);
	_u_omega2(5,1) = 0.2877f/(Kt2*Lm);
	_u_omega2(5,2) = -1/(6*Kq1);
	_u_omega2(5,3) = 1/(6*Kt2);
#endif


#ifdef OCT

	float Den = 4*(Kq1*Kt2 + Kt1*Kq2);
	_u_omega2(0,0) = -1.4142f*Kq2/(Lm*Den);
	_u_omega2(0,1) =  1.4142f*Kq2/(Lm*Den);
	_u_omega2(0,2) =  Kt2/Den;
	_u_omega2(0,3) =  Kq2/Den;

	_u_omega2(1,0) =  1.4142f*Kq2/(Lm*Den);
	_u_omega2(1,1) = -1.4142f*Kq2/(Lm*Den);
	_u_omega2(1,2) =  Kt2/Den;
	_u_omega2(1,3) =  Kq2/Den;


	_u_omega2(2,0) =  1.4142f*Kq1/(Lm*Den);
	_u_omega2(2,1) =  1.4142f*Kq1/(Lm*Den);
	_u_omega2(2,2) = -Kt1/Den;
	_u_omega2(2,3) =  Kq1/Den;

	_u_omega2(3,0) = -1.4142f*Kq1/(Lm*Den);
	_u_omega2(3,1) = -1.4142f*Kq1/(Lm*Den);
	_u_omega2(3,2) = -Kt1/Den;
	_u_omega2(3,3) =  Kq1/Den;

	_u_omega2(4,0) = -1.4142f*Kq1/(Lm*Den);
	_u_omega2(4,1) =  1.4142f*Kq1/(Lm*Den);
	_u_omega2(4,2) = -Kt1/Den;
	_u_omega2(4,3) =  Kq1/Den;

	_u_omega2(5,0) =  1.4142f*Kq1/(Lm*Den);
	_u_omega2(5,1) = -1.4142f*Kq1/(Lm*Den);
	_u_omega2(5,2) = -Kt1/Den;
	_u_omega2(5,3) =  Kq1/Den;

	_u_omega2(6,0) =  1.4142f*Kq2/(Lm*Den);
	_u_omega2(6,1) =  1.4142f*Kq2/(Lm*Den);
	_u_omega2(6,2) =  Kt2/Den;
	_u_omega2(6,3) =  Kq2/Den;

	_u_omega2(7,0) = -1.4142f*Kq2/(Lm*Den);
	_u_omega2(7,1) = -1.4142f*Kq2/(Lm*Den);
	_u_omega2(7,2) =  Kt2/Den;
	_u_omega2(7,3) =  Kq2/Den;
#endif
	min_omega_square = (0.15f*Cm + Ct)*(0.15f*Cm + Ct); // limit the minimal throttle
	max_omega_square = (0.7f *Cm + Ct)*(0.7f*Cm + Ct);
}

void
MulticopterAttitudeControl::parameter_update_poll()
{
	bool updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

void
MulticopterAttitudeControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void
MulticopterAttitudeControl::vehicle_status_poll()
{
	bool updated;
	orb_check(_v_status_sub,&updated);
	if(updated){
		orb_copy(ORB_ID(vehicle_status),_v_status_sub,&_v_status);
	}

}
/*
void
MulticopterAttitudeControl::position_triplet_poll()
{
	bool updated;
	orb_check(_pos_sp_triplet_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
	}
}
*/
void
MulticopterAttitudeControl::odroid_preflight_status_poll()
{
	bool updated;
	orb_check(_odroid_preflight_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(odroid_preflight_status), _odroid_preflight_status_sub, &_odroid_preflight_status);
	}
}
/*
void
MulticopterAttitudeControl::vehilcle_serial_comand_poll()
{
	bool updated;
	orb_check(_vehicle_serial_command_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_serial_command), _vehicle_serial_command_sub, &_vehicle_serial_cmd);
	}
}
*/

void
MulticopterAttitudeControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void
MulticopterAttitudeControl::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}

void
MulticopterAttitudeControl::home_position_poll()
{
	bool updated;
	orb_check(_home_sub,&updated);

	if(updated){
		orb_copy(ORB_ID(home_position),_home_sub,&_home);
        home_valid = true;
	}
}

void
MulticopterAttitudeControl::local_position_poll()
{
	bool updated;
	orb_check(_local_pos_sub,&updated);

	if(updated){
		orb_copy(ORB_ID(vehicle_local_position),_local_pos_sub,&_local_pos);
	}
}

void
MulticopterAttitudeControl::gps_poll()
{
	bool updated;
	orb_check(_gps_sub,&updated);

	if(updated){
		orb_copy(ORB_ID(vehicle_gps_position),_gps_sub,&_gps);
		if (_gps.eph < 20 * 0.7f && _gps.epv < 20 * 0.7f && _gps.fix_type >= 3)
			gps_valid = true;
	}
}

void MulticopterAttitudeControl::battery_poll()
{
	bool updated;
	orb_check(_battery_sub,&updated);
	if(updated)
		orb_copy(ORB_ID(battery_status),_battery_sub,&_battery);
}

void MulticopterAttitudeControl::sensor_combined_poll(){
	bool updated;
	orb_check(_sensor_combined_sub,&updated);
	if(updated)
		orb_copy(ORB_ID(sensor_combined),_sensor_combined_sub,&_sensor_combined);

}

float
MulticopterAttitudeControl::scale_control(float ctl, float end, float dz)
{
	if (ctl > dz) {
		return (ctl - dz) / (end - dz);

	} else if (ctl < -dz) {
		return (ctl + dz) / (end - dz);

	} else {
		return 0.0f;
	}
}


/*
 * Attitude controller.
 * Input: 'manual_control_setpoint' and 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp', 'vehicle_attitude_setpoint' topic (for manual modes)
 */
void
MulticopterAttitudeControl::control_attitude(float dt)
{
	float yaw_sp_move_rate = 0.0f;
	bool publish_att_sp = false;

	if (_v_control_mode.flag_control_manual_enabled) {
		/* manual input, set or modify attitude setpoint */

		if (_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_climb_rate_enabled) {
			/* in assisted modes poll 'vehicle_attitude_setpoint' topic and modify it */
			vehicle_attitude_setpoint_poll();
		}

		if (!_v_control_mode.flag_control_climb_rate_enabled) {
			/* pass throttle directly if not in altitude stabilized mode */
			_v_att_sp.thrust = _manual_control_sp.z;
			publish_att_sp = true;
		}

		if (!_armed.armed) {
			/* reset yaw setpoint when disarmed */
			_reset_yaw_sp = true;
		}

		/* move yaw setpoint in all modes */
	//	if (_v_att_sp.thrust < 0.3f) {
		if (_v_status.condition_landed){
			// TODO
			//if (_status.condition_landed) {
			/* reset yaw setpoint if on ground */
			_reset_yaw_sp = true;
			//}
		} else {
			/* move yaw setpoint */

			/*move the yaw setpoint directly*/
			yaw_sp_move_rate = _manual_control_sp.r * _params.man_yaw_max;
			yaw_sp_move_rate = limit_acc(yaw_sp_move_rate,yaw_sp_rate_pre,1.0f,dt);
			yaw_sp_rate_pre = yaw_sp_move_rate;
			_v_att_sp.yaw_body = _wrap_pi(_v_att_sp.yaw_body + yaw_sp_move_rate * dt);

			/*limit the yaw offset error*/

			float yaw_offs_max = _params.man_yaw_max;
			float yaw_offs = _wrap_pi(_v_att_sp.yaw_body - _v_att.yaw);
			if (yaw_offs < - yaw_offs_max) {
				_v_att_sp.yaw_body = _wrap_pi(_v_att.yaw - yaw_offs_max);

			} else if (yaw_offs > yaw_offs_max) {
				_v_att_sp.yaw_body = _wrap_pi(_v_att.yaw + yaw_offs_max);
			}
			_v_att_sp.R_valid = false;
			publish_att_sp = true;
		}

		/* reset yaw setpint to current position if needed */
		if (_reset_yaw_sp) {
			_reset_yaw_sp = false;
			yaw_sp_pre = _v_att.yaw;
			yaw_sp_rate_pre = 0.0f;
			_v_att_sp.yaw_body = _v_att.yaw;
			_v_att_sp.R_valid = false;
			publish_att_sp = true;
		}

		if (!_v_control_mode.flag_control_velocity_enabled) {
			/* update attitude setpoint if not in position control mode */
			_v_att_sp.roll_body = _manual_control_sp.y * _params.man_roll_max;
			_v_att_sp.pitch_body = -_manual_control_sp.x * _params.man_pitch_max;
			_v_att_sp.R_valid = false;
			publish_att_sp = true;
		}

	} else {
		/* in non-manual mode use 'vehicle_attitude_setpoint' topic */
		vehicle_attitude_setpoint_poll();
		yaw_sp_move_rate = _v_att_sp.yaw_speed;
		yaw_sp_move_rate = limit_acc(yaw_sp_move_rate,yaw_sp_rate_pre,1.0f,dt);
		yaw_sp_rate_pre = yaw_sp_move_rate;
		/* reset yaw setpoint after non-manual control mode */
		//_reset_yaw_sp = true;
	}

	_thrust_sp = _v_att_sp.thrust;



   float pitch_sp_cur = limit_acc(_v_att_sp.pitch_body,pitch_sp_pre,2.0f,dt);
  // _rates_sp(0) = (pitch_sp_cur - pitch_sp_pre)/dt;
   _rates_sp(0) = 0.0f;
   pitch_sp_pre = pitch_sp_cur;
   _v_att_sp.pitch_body = pitch_sp_cur;

   float roll_sp_cur = limit_acc(_v_att_sp.roll_body,roll_sp_pre,2.0f,dt);
  // _rates_sp(1) = (roll_sp_cur - roll_sp_pre)/dt;
   _rates_sp(1) = 0.0f;
   roll_sp_pre = roll_sp_cur;
   _v_att_sp.roll_body = roll_sp_cur;

   _rates_sp(2) = _params.yaw_ff * yaw_sp_rate_pre;



	/* construct attitude setpoint rotation matrix */
	math::Matrix<3, 3> R_sp;

		/* rotation matrix in _att_sp is not valid, use euler angles instead */
	R_sp.from_euler(_v_att_sp.roll_body, _v_att_sp.pitch_body, _v_att_sp.yaw_body);

	/* copy rotation matrix back to setpoint struct */
	memcpy(&_v_att_sp.R_body[0][0], &R_sp.data[0][0], sizeof(_v_att_sp.R_body));
	_v_att_sp.R_valid = true;


	/* publish the attitude setpoint if needed */
	if (publish_att_sp) {
		_v_att_sp.timestamp = hrt_absolute_time();

		if (_att_sp_pub > 0) {
			orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_v_att_sp);

		} else {
			_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_v_att_sp);
		}
	}

	/* rotation matrix for current state */
	math::Matrix<3, 3> R;
	R.set(_v_att.R);

	/* all input data is ready, run controller itself */

	/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
	math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
	math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

	/* axis and sin(angle) of desired rotation */
	math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);

	/* calculate angle error */
	float e_R_z_sin = e_R.length();
	float e_R_z_cos = R_z * R_sp_z;

	/* calculate weight for yaw control */
	float yaw_w = R_sp(2, 2) * R_sp(2, 2);

	/* calculate rotation matrix after roll/pitch only rotation */
	math::Matrix<3, 3> R_rp;

	if (e_R_z_sin > 0.0f) {
		/* get axis-angle representation */
		float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
		math::Vector<3> e_R_z_axis = e_R / e_R_z_sin;

		e_R = e_R_z_axis * e_R_z_angle;

		/* cross product matrix for e_R_axis */
		math::Matrix<3, 3> e_R_cp;
		e_R_cp.zero();
		e_R_cp(0, 1) = -e_R_z_axis(2);
		e_R_cp(0, 2) = e_R_z_axis(1);
		e_R_cp(1, 0) = e_R_z_axis(2);
		e_R_cp(1, 2) = -e_R_z_axis(0);
		e_R_cp(2, 0) = -e_R_z_axis(1);
		e_R_cp(2, 1) = e_R_z_axis(0);

		/* rotation matrix for roll/pitch only rotation */
		R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));

	} else {
		/* zero roll/pitch rotation */
		R_rp = R;
	}

	/* R_rp and R_sp has the same Z axis, calculate yaw error */
	math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
	math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
	e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

	if (e_R_z_cos < 0.0f) {
		/* for large thrust vector rotations use another rotation method:
		 * calculate angle and axis for R -> R_sp rotation directly */
		math::Quaternion q;
		q.from_dcm(R.transposed() * R_sp);
		math::Vector<3> e_R_d = q.imag();
		e_R_d.normalize();
		e_R_d *= 2.0f * atan2f(e_R_d.length(), q(0));

		/* use fusion of Z axis based rotation and direct rotation */
		float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
		e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
	}

	_angle_error(0) = e_R(0);
	_angle_error(1) = e_R(1);
	_angle_error(2) = _wrap_pi(e_R(2));

}

void MulticopterAttitudeControl::pre_arm_check()
{
	_actuators.control[0] = -1.0f;
	_actuators.control[1] = -1.0f;
	_actuators.control[2] = -1.0f;
	_actuators.control[3] = -1.0f;
#ifdef HEX
	_actuators.control[4] = -1.0f;
	_actuators.control[5] = -1.0f;
#endif

#ifdef OCT
	_actuators.control[4] = -1.0f;
	_actuators.control[5] = -1.0f;
	_actuators.control[6] = -1.0f;
	_actuators.control[7] = -1.0f;
#endif
		if(hrt_absolute_time() - armed_timing>3000*1000 && hrt_absolute_time() - armed_timing <4000*1000 )
		{
#ifdef QUAD
			_actuators.control[0] = -0.7f;
			_actuators.control[1] = -1.0f;
			_actuators.control[2] = -1.0f;
			_actuators.control[3] = -1.0f;
#endif
#ifdef OCT
			_actuators.control[0] = -0.8f;
			_actuators.control[1] = -1.0f;
			_actuators.control[2] = -1.0f;
			_actuators.control[3] = -1.0f;
			_actuators.control[4] = -0.8f;
			_actuators.control[5] = -1.0f;
			_actuators.control[6] = -1.0f;
			_actuators.control[7] = -1.0f;
#endif
			if(_battery.current_a > max_1_current)
			{
				max_1_current = _battery.current_a;
			}
		}

		if(hrt_absolute_time() - armed_timing>5000*1000 && hrt_absolute_time() - armed_timing <6000*1000 )
		{
#ifdef QUAD
			_actuators.control[0] = -1.0f;
			_actuators.control[1] = -0.7f;
			_actuators.control[2] = -1.0f;
			_actuators.control[3] = -1.0f;
#endif
#ifdef OCT
			_actuators.control[0] = -1.0f;
			_actuators.control[1] = -0.8f;
			_actuators.control[2] = -1.0f;
			_actuators.control[3] = -1.0f;
			_actuators.control[4] = -1.0f;
			_actuators.control[5] = -0.8f;
			_actuators.control[6] = -1.0f;
			_actuators.control[7] = -1.0f;
#endif
			if(_battery.current_a > max_2_current)
			{
				max_2_current = _battery.current_a;
			}
		}
		else if(hrt_absolute_time() - armed_timing>7000*1000 && hrt_absolute_time() - armed_timing<8000*1000 )
		{
#ifdef QUAD
			_actuators.control[0] = -1.0f;
			_actuators.control[1] = -1.0f;
			_actuators.control[2] = -0.7f;
			_actuators.control[3] = -1.0f;
#endif
#ifdef OCT
			_actuators.control[0] = -1.0f;
			_actuators.control[1] = -1.0f;
			_actuators.control[2] = -0.8f;
			_actuators.control[3] = -1.0f;
			_actuators.control[4] = -1.0f;
			_actuators.control[5] = -1.0f;
			_actuators.control[6] = -0.8f;
			_actuators.control[7] = -1.0f;
#endif
			if(_battery.current_a > max_3_current)
			{
				max_3_current = _battery.current_a;
			}
		}
		else if(hrt_absolute_time() - armed_timing>9000*1000 && hrt_absolute_time() - armed_timing<10000*1000 )
		{
#ifdef QUAD
			_actuators.control[0] = -1.0f;
			_actuators.control[1] = -1.0f;
			_actuators.control[2] = -1.0f;
			_actuators.control[3] = -0.7f;
#endif
#ifdef OCT
			_actuators.control[0] = -1.0f;
			_actuators.control[1] = -1.0f;
			_actuators.control[2] = -1.0f;
			_actuators.control[3] = -0.8f;
			_actuators.control[4] = -1.0f;
			_actuators.control[5] = -1.0f;
			_actuators.control[6] = -1.0f;
			_actuators.control[7] = -0.8f;
#endif
			if(_battery.current_a > max_4_current)
			{
				max_4_current = _battery.current_a;
			}

		}
		else if(hrt_absolute_time() - armed_timing>11000*1000 && hrt_absolute_time() - armed_timing<11100*1000){
			if(! motor_checked)
			{
				check_motor();
				motor_checked = true;
			}

		}


}


#ifdef QUAD
void MulticopterAttitudeControl::calculate_delta()
{

  math::Vector <4> _omega_square;
  if(_thrust_sp>0.1f){
	  _u(0) = isfinite(_att_control(0))?_att_control(0)*Ix:0.0f;
	  _u(1) = isfinite(_att_control(1))?_att_control(1)*Iy:0.0f;
	  _u(2) = isfinite(_att_control(2))?_att_control(2)*Iz:0.0f;
  }
  else
  {
	  _u(0) = 0.0f;
	  _u(1) = 0.0f;
	  _u(2) = 0.0f;
  }
  _u(3) = isfinite(_thrust_sp)?_thrust_sp*2.0f*Mg:0.0f;

  _omega_square = _u_omega2*_u;

  /*limit the ratating speed of the motor, the rotating speed could not be less than 0.1*/
  if(_omega_square(0)<0)
	  _omega_square(0) = 0;
  if(_omega_square(1)<0)
	  _omega_square(1) = 0;
  if(_omega_square(2)<0)
	  _omega_square(2) = 0;
  if(_omega_square(3)<0)
	  _omega_square(3) = 0;


  _actuators1.control[0] = _u(0);
  _actuators1.control[1] = _u(1);
  _actuators1.control[2] = _u(2);
  _actuators1.control[3] = _thrust_sp;

  _actuators.control[0] = limit_range(((sqrtf(_omega_square(0))-Ct)/Cm)*2-1,-1.0f,1.0f);
  _actuators.control[1] = limit_range(((sqrtf(_omega_square(1))-Ct)/Cm)*2-1,-1.0f,1.0f);
  _actuators.control[2] = limit_range(((sqrtf(_omega_square(2))-Ct)/Cm)*2-1,-1.0f,1.0f);
  _actuators.control[3] = limit_range(((sqrtf(_omega_square(3))-Ct)/Cm)*2-1,-1.0f,1.0f);

}
#endif

#ifdef HEX
void MulticopterAttitudeControl::calculate_delta()
{

	  math::Vector <6> _omega_square;
	  if(_thrust_sp>0.1f){
		  _u(0) = isfinite(_att_control(0))?_att_control(0)*Ix:0.0f;
		  _u(1) = isfinite(_att_control(1))?_att_control(1)*Iy:0.0f;
		  _u(2) = isfinite(_att_control(2))?_att_control(2)*Iz:0.0f;
	  }
	  else
	  {
		  _u(0) = 0.0f;
		  _u(1) = 0.0f;
		  _u(2) = 0.0f;
	  }
	  _u(3) = isfinite(_thrust_sp)?_thrust_sp*2.0f*Mg:0.0f;

	  _actuators1.control[0] = _att_control(0);
	  _actuators1.control[1] = _att_control(1);
	  _actuators1.control[2] = _att_control(2);
	  _actuators1.control[3] = _thrust_sp;

	  _omega_square = _u_omega2*_u;

	  if(_omega_square(0)<0.0f)
		  _omega_square(0) = 0.0f;
	  if(_omega_square(1)<0.0f)
		  _omega_square(1) = 0.0f;
	  if(_omega_square(2)<0.0f)
		  _omega_square(2) = 0.0f;
	  if(_omega_square(3)<0.0f)
		  _omega_square(3) = 0.0f;
	  if(_omega_square(4)<0.0f)
		  _omega_square(4) = 0.0f;
	  if(_omega_square(5)<0.0f)
		  _omega_square(5) = 0.0f;

	  _actuators.control[0] = constrain_ref(((sqrtf(_omega_square(0))-Ct)/Cm)*2-1,1.0f);
	  _actuators.control[1] = constrain_ref(((sqrtf(_omega_square(1))-Ct)/Cm)*2-1,1.0f);
	  _actuators.control[2] = constrain_ref(((sqrtf(_omega_square(2))-Ct)/Cm)*2-1,1.0f);
	  _actuators.control[3] = constrain_ref(((sqrtf(_omega_square(3))-Ct)/Cm)*2-1,1.0f);
	  _actuators.control[4] = constrain_ref(((sqrtf(_omega_square(4))-Ct)/Cm)*2-1,1.0f);
	  _actuators.control[5] = constrain_ref(((sqrtf(_omega_square(5))-Ct)/Cm)*2-1,1.0f);


}
#endif

#ifdef OCT
void MulticopterAttitudeControl::calculate_delta()
{

  math::Vector <8> _omega_square;
  if(_thrust_sp>0.1f){
	  _u(0) = isfinite(_att_control(0))?_att_control(0)*Ix:0.0f;
	  _u(1) = isfinite(_att_control(1))?_att_control(1)*Iy:0.0f;
	  _u(2) = isfinite(_att_control(2))?_att_control(2)*Iz:0.0f;
  }
  else
  {
	  _u(0) = 0.0f;
	  _u(1) = 0.0f;
	  _u(2) = 0.0f;
  }
  _u(3) = isfinite(_thrust_sp)?_thrust_sp*2.0f*Mg:0.0f;





  _omega_square = _u_omega2*_u;


  if(_omega_square(0)<0)
	  _omega_square(0) = 0;
  if(_omega_square(1)<0)
	  _omega_square(1) = 0;
  if(_omega_square(2)<0)
	  _omega_square(2) = 0;
  if(_omega_square(3)<0)
	  _omega_square(3) = 0;
  if(_omega_square(4)<0)
	  _omega_square(4) = 0;
  if(_omega_square(5)<0)
	  _omega_square(5) = 0;
  if(_omega_square(6)<0)
	  _omega_square(6) = 0;
  if(_omega_square(7)<0)
	  _omega_square(7) = 0;



  _actuators1.control[0] = _u(0);
  _actuators1.control[1] = _u(1);
  _actuators1.control[2] = _u(2);
  _actuators1.control[3] = _thrust_sp;

  _actuators.control[0] = limit_range(((sqrtf(_omega_square(0))-Ct)/Cm)*2-1,-1.0f,1.0f);
  _actuators.control[1] = limit_range(((sqrtf(_omega_square(1))-Ct)/Cm)*2-1,-1.0f,1.0f);
  _actuators.control[2] = limit_range(((sqrtf(_omega_square(2))-Ct)/Cm)*2-1,-1.0f,1.0f);
  _actuators.control[3] = limit_range(((sqrtf(_omega_square(3))-Ct)/Cm)*2-1,-1.0f,1.0f);
  _actuators.control[4] = limit_range(((sqrtf(_omega_square(4))-Ct)/Cm)*2-1,-1.0f,1.0f);
  _actuators.control[5] = limit_range(((sqrtf(_omega_square(5))-Ct)/Cm)*2-1,-1.0f,1.0f);
  _actuators.control[6] = limit_range(((sqrtf(_omega_square(6))-Ct)/Cm)*2-1,-1.0f,1.0f);
  _actuators.control[7] = limit_range(((sqrtf(_omega_square(7))-Ct)/Cm)*2-1,-1.0f,1.0f);


}


#endif







bool MulticopterAttitudeControl::check_motor()
{
		ave_current = _params.check_current;
		_motor_status.motor_fault = false;
		_motor_status.motor1 = true;
		_motor_status.motor2 = true;
		_motor_status.motor3 = true;
		_motor_status.motor4 = true;


	if(max_1_current < ave_current)
	{
		_motor_status.motor1 = false;
		_motor_status.motor_fault = true;
	}

	if(max_2_current < ave_current)
	{
		_motor_status.motor2 = false;
		_motor_status.motor_fault = true;
	}

	if(max_3_current < ave_current)
	{
		_motor_status.motor3 = false;
		_motor_status.motor_fault = true;
	}

	if(max_4_current < ave_current)
	{
		_motor_status.motor4 = false;
		_motor_status.motor_fault = true;
	}


	if(_motor_status_pub > 0)
	{
		orb_publish(ORB_ID(motor_status),_motor_status_pub,&_motor_status);
	}
	else
		_motor_status_pub = orb_advertise(ORB_ID(motor_status),&_motor_status);

	return false;
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
MulticopterAttitudeControl::control_attitude_rates(float dt)
{
	/* reset integral if disarmed */
	if (!_armed.armed) {
		_angle_int.zero();
	}

	/* current body angular rates */
	math::Vector<3> rates;
	rates(0) = _v_att.rollspeed;
	rates(1) = _v_att.pitchspeed;
	rates(2) = _v_att.yawspeed;

	/* angular rates error */
	math::Vector<3> rates_err = _rates_sp - rates;

	_rates_d = (rates - _rates_prev)/dt;


	_att_control = _params.rate_p.emult(rates_err) - _params.rate_d.emult(_rates_d) + _angle_int + _params.att_p.emult(_angle_error);
	_rates_prev = rates;
    _rates_d_prev = _rates_d;


	/* update integral only if not saturated on low limit */
	if (_thrust_sp > MIN_TAKEOFF_THRUST) {
		for (int i = 0; i < 3; i++) {
			if (fabsf(_att_control(i)/19.6f) < _thrust_sp) {
				float angle_i = _angle_int(i) + _params.angle_i(i) * _angle_error(i) * dt;

				if (isfinite(angle_i) && angle_i > -ANGLE_I_LIMIT && angle_i < ANGLE_I_LIMIT &&_att_control(i) > -ANGLE_I_LIMIT && _att_control(i) < ANGLE_I_LIMIT) {
					_angle_int(i) = angle_i;
				}
			}
		}
	}
}


void
MulticopterAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	mc_att_control::g_control->task_main();
}

void
MulticopterAttitudeControl::task_main()
{
	warnx("started");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	_home_sub = orb_subscribe(ORB_ID(home_position));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	_battery_sub = orb_subscribe(ORB_ID(battery_status));
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	_v_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	//_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	//_odroid_status_sub=orb_subscribe(ORB_ID(odroid_status));
	_odroid_preflight_status_sub=orb_subscribe(ORB_ID(odroid_preflight_status));
//	_vehicle_serial_command_sub=orb_subscribe(ORB_ID(vehicle_serial_command));

	memset(&_motor_status, 0, sizeof(_motor_status));
	_motor_status.motor1 = true;
	_motor_status.motor2 = true;
	_motor_status.motor3 = true;
	_motor_status.motor4 = true;
	memset(&_battery,0,sizeof(_battery));


	 gps_valid = false;
	 home_valid = false;
	 pitch_sp_pre = 0.0f;
	 roll_sp_pre = 0.0f;
	 yaw_sp_pre = 0.0f;
	 was_armed = false;
	 armed_timing = 0;
#ifdef QUAD
	// int fixed_time_shutter_count = 0;
	// int time_in_sec = 0;
#endif

	  max_1_current = 0.0f;
	  max_2_current = 0.0f;
	  max_3_current = 0.0f;
	  max_4_current = 0.0f;
	  ave_current = 1.0f;
	  motor_checked = false;
	  loop_count = 0;


	Ix  = 0.24f;
	Iy  = 0.26f;
	Iz  = 0.5992f;
	Kt1 = 0.00039915f;
	Kt2 = 0.00044879f;
	Kq1 = 0.000012087f;
	Kq2 = 0.000013324f;
	Lm  = 0.500f;
	Cm  = 440.9687f;  // omega = Cm * delta + Ct
	Ct  = 37.0633f;
	Mg  = 61.5538f; //9.268 Kg  kangli
	air_density_ratio = 1.0f;
	pressure_filtered = 1006.0f;
	temp_filtered = 25.0f;

	bool airdensity_filter_init = false;
	int  baro_count = 0;
	float sum_pressure = 0.0f;
	float sum_temp = 0.0f;

	memset(&_actuators,0,sizeof(_actuators));
	memset(&_actuators1,0,sizeof(_actuators1));

	math::Matrix<4,4> omega_square_to_u;

	/* initialize parameters cache */
	parameters_update();



	/* wakeup source: vehicle attitude */
	struct pollfd fds[1];

	fds[0].fd = _v_att_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);
		loop_count++;

		/* run controller on attitude changes */
		if (fds[0].revents & POLLIN) {
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}


			/* copy attitude topic */
			orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);

			/* check for updates in other topics */
			parameter_update_poll();
			vehicle_control_mode_poll();
			arming_status_poll();
			vehicle_manual_poll();
			home_position_poll();
			local_position_poll();
			gps_poll();
			battery_poll();
			vehicle_status_poll();
			sensor_combined_poll();
//			position_triplet_poll();
			odroid_preflight_status_poll();
			//vehilcle_serial_comand_poll();

			if(loop_count % 10 == 0)
			{
				if(!airdensity_filter_init)
				{
					sum_pressure += _sensor_combined.baro_pres_mbar;
					sum_temp += _sensor_combined.baro_temp_celcius;
					baro_count++;
					if(baro_count>=10)
					{
						pressure_filtered = sum_pressure/baro_count;
						//temp_filtered = sum_temp/baro_count;
						airdensity_filter_init = true;
					}
				}
				else
				{
					pressure_filtered = 0.99f*pressure_filtered + 0.01f*_sensor_combined.baro_pres_mbar;
					//temp_filtered = temp_filtered * 0.99f + 0.0095f * _sensor_combined.baro_temp_celcius;
					temp_filtered = _params.temp;
					air_density_ratio = (pressure_filtered/1006.0f)*((25.0f + 273.15f)/(temp_filtered + 273.15f));
					if(air_density_ratio > 1.3f)
						air_density_ratio = 1.3f;
					else if(air_density_ratio < 0.5f)
						air_density_ratio = 0.5f;
					if(loop_count % 200 == 0)
						update_u_omega2();
				}


			}


			if (_v_control_mode.flag_control_attitude_enabled) {
				control_attitude(dt);

				/* publish attitude rates setpoint */
				_v_rates_sp.roll = _rates_sp(0);
				_v_rates_sp.pitch = _rates_sp(1);
				_v_rates_sp.yaw = _rates_sp(2);
				_v_rates_sp.thrust = _thrust_sp;
				_v_rates_sp.timestamp = hrt_absolute_time();

				if (_v_rates_sp_pub > 0) {
					orb_publish(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_pub, &_v_rates_sp);

				} else {
					_v_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_v_rates_sp);
				}

			} else {
				/* attitude controller disabled, poll rates setpoint topic */
				if (_v_control_mode.flag_control_manual_enabled) {
					/* manual rates control - ACRO mode */
					_rates_sp = math::Vector<3>(_manual_control_sp.y, -_manual_control_sp.x, _manual_control_sp.r).emult(_params.acro_rate_max);
					_thrust_sp = _manual_control_sp.z;

					/* reset yaw setpoint after ACRO */
					_reset_yaw_sp = true;

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub > 0) {
						orb_publish(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_pub, &_v_rates_sp);

					} else {
						_v_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_v_rates_sp);
					}

				} else {
					/* attitude controller disabled, poll rates setpoint topic */
					vehicle_rates_setpoint_poll();
					_rates_sp(0) = _v_rates_sp.roll;
					_rates_sp(1) = _v_rates_sp.pitch;
					_rates_sp(2) = _v_rates_sp.yaw;
					_thrust_sp = _v_rates_sp.thrust;
				}
			}

			if (_v_control_mode.flag_control_rates_enabled) {
				control_attitude_rates(dt); //kangli

				if(_armed.armed)
				{
					if(was_armed == false)
					{
						was_armed = true;
						armed_timing = hrt_absolute_time();
					}


					if(hrt_absolute_time() - armed_timing<11100*1000 && _params.check_on_off > 0.5f)  //kangli modified on 8th Dec
						{
							pre_arm_check();
						}
					else
						{

						    calculate_delta();
						}
				}
				else{
						_actuators.control[0] = -1.0f;
						_actuators.control[1] = -1.0f;
						_actuators.control[2] = -1.0f;
						_actuators.control[3] = -1.0f;
#ifdef QUAD
						_actuators.control[4] = -0.4f;
						_actuators.control[5] =  0.0f;
#endif

#ifdef HEX
						_actuators.control[4] = -1.0f;
						_actuators.control[5] = -1.0f;
#endif

#ifdef OCT
						_actuators.control[4] = -1.0f;
						_actuators.control[5] = -1.0f;
						_actuators.control[6] = -1.0f;
						_actuators.control[7] = -1.0f;

#endif

						_actuators1.control[0] = 0.0f;
						_actuators1.control[1] = 0.0f;
						_actuators1.control[2] = 0.0f;
						_actuators1.control[3] = 0.0f;
					}

/*
#ifdef QUAD
				if(_vehicle_serial_cmd.cmd1==1)
					{
				//	printf("start to act...................\n");
					_actuators.control[5] = 1.0f;

					}

					else
						_actuators.control[5] = -1.0f;
#endif

*/
//				if(!_v_control_mode.flag_control_manual_enabled)
//				 {
//					_actuators1.control[4] = _vehicle_serial_cmd.servo1;
//					_actuators1.control[5] = _vehicle_serial_cmd.servo2;
//					_actuators1.control[6] = _vehicle_serial_cmd.servo3;
//					_actuators1.control[7] = _vehicle_serial_cmd.servo4;
//				 }
//				else
//				{
//					_actuators1.control[4] = _manual_control_sp.aux1;
//					_actuators1.control[5] = _manual_control_sp.aux2;
//					_actuators1.control[6] = _manual_control_sp.aux3;
//					_actuators1.control[7] = _manual_control_sp.aux4;
//
//				}
			     _actuators.timestamp = hrt_absolute_time();

				if (!_actuators_0_circuit_breaker_enabled) {
					if (_actuators_0_pub > 0) {
						orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);
					} else {
						_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
					}
/*mingjie, gimbal_aux will publish this topic*/
					if (_actuators_1_pub > 0) {
						orb_publish(ORB_ID(actuator_controls_1), _actuators_1_pub, &_actuators1);

					} else {
						_actuators_1_pub = orb_advertise(ORB_ID(actuator_controls_1), &_actuators1);
					}

				}
			}
		}

		perf_end(_loop_perf);
	}

	warnx("exit");

	_control_task = -1;
	_exit(0);
}

int
MulticopterAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("mc_att_control",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2000,
				       (main_t)&MulticopterAttitudeControl::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_att_control_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: mc_att_control {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (mc_att_control::g_control != nullptr)
			errx(1, "already running");

		mc_att_control::g_control = new MulticopterAttitudeControl;

		if (mc_att_control::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != mc_att_control::g_control->start()) {
			delete mc_att_control::g_control;
			mc_att_control::g_control = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (mc_att_control::g_control == nullptr)
			errx(1, "not running");

		delete mc_att_control::g_control;
		mc_att_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (mc_att_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
