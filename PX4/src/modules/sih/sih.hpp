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

#include <px4_module.h>
#include <px4_module_params.h>

#include <matrix/matrix/math.hpp> 	// matrix, vectors, dcm, quaterions
#include <conversion/rotation.h> 	// math::radians, 
#include <ecl/geo/geo.h> 			// to get the physical constants
#include <drivers/drv_hrt.h> 		// to get the real time

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/sih.h>

using namespace matrix;

extern "C" __EXPORT int sih_main(int argc, char *argv[]);

class Sih : public ModuleBase<Sih>, public ModuleParams
{
public:
	Sih(int example_param, bool example_flag);

	virtual ~Sih() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Sih *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	static float generate_wgn(); 	// generate white Gaussian noise sample

	// generate white Gaussian noise sample as a 3D vector with specified std
	static Vector3f noiseGauss3f(float stdx, float stdy, float stdz);

	// static int pack_float(char* uart_msg, int index, void *value); 	// pack a float to a IEEE754
private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update_poll(int parameter_update_sub);
	void parameters_updated();

	uint8_t is_HIL_running(int vehicle_status_sub);

	// to publish the simulator states
	struct sih_s 					_sih {};
	orb_advert_t    				_sih_pub{nullptr};
	// to publish the sensor baro
	struct sensor_baro_s 			_sensor_baro {};
	orb_advert_t    				_sensor_baro_pub{nullptr};
	// to publish the sensor mag
	struct sensor_mag_s 			_sensor_mag {};
	orb_advert_t    				_sensor_mag_pub{nullptr};
	// to publish the sensor gyroscope
	struct sensor_gyro_s 			_sensor_gyro {};
	orb_advert_t    				_sensor_gyro_pub{nullptr};
	// to publish the sensor accelerometer
	struct sensor_accel_s 			_sensor_accel {};
	orb_advert_t    				_sensor_accel_pub{nullptr};
	// to publish the gps position
	struct vehicle_gps_position_s 	_vehicle_gps_pos {};
	orb_advert_t 					_vehicle_gps_pos_pub{nullptr};

	// hard constants
	static constexpr uint16_t NB_MOTORS = 4;
	static constexpr float T1_C = 15.0f; 						// ground temperature in celcius
	static constexpr float T1_K = T1_C - CONSTANTS_ABSOLUTE_NULL_CELSIUS;	// ground temperature in Kelvin
	static constexpr float TEMP_GRADIENT  = -6.5f / 1000.0f;	// temperature gradient in degrees per metre
	static constexpr uint32_t BAUDS_RATE = 57600; 			// bauds rate of the serial port

	void init_variables();
	void init_sensors();
	int  init_serial_port();
	void read_motors(const int actuator_out_sub);
	void generate_force_and_torques();
	void equations_of_motion();
	void reconstruct_sensors_signals();
	void send_IMU(hrt_abstime now);
	void send_gps(hrt_abstime now);
	void publish_sih();
	void send_serial_msg(int serial_fd, int64_t t_ms);

	float  		_dt; 			// sampling time [s]

	char _uart_name[12] = "/dev/ttyS5/"; 					// serial port name

	Vector3f 	_T_B; 			// thrust force in body frame [N]
	Vector3f 	_Fa_I; 			// aerodynamic force in inertial frame [N]
	Vector3f 	_Mt_B; 			// thruster moments in the body frame [Nm]
	Vector3f 	_Ma_B; 			// aerodynamic moments in the body frame [Nm]
	Vector3f 	_p_I; 			// inertial position [m]
	Vector3f 	_v_I; 			// inertial velocity [m/s]
	Vector3f 	_v_B; 			// body frame velocity [m/s]
	Vector3f 	_p_I_dot; 		// inertial position differential
	Vector3f 	_v_I_dot; 		// inertial velocity differential
	Quatf 		_q; 			// quaternion attitude
	Dcmf 		_C_IB; 			// body to inertial transformation
	Vector3f 	_w_B; 			// body rates in body frame [rad/s]
	Quatf 		_q_dot;			// quaternion differential
	Vector3f 	_w_B_dot; 		// body rates differential
	float 		_u[NB_MOTORS];	// thruster signals


	// sensors reconstruction
	Vector3f 	_acc;
	Vector3f 	_mag;
	Vector3f 	_gyro;
	Vector3f 	_gps_vel;
	double 		_gps_lat, _gps_lat_noiseless;
	double 		_gps_lon, _gps_lon_noiseless;
	float 		_gps_alt, _gps_alt_noiseless;
	float 		_baro_p_mBar; 	// reconstructed (simulated) pressure in mBar
	float 		_baro_temp_c; 	// reconstructed (simulated) barometer temperature in celcius

	// parameters
	float _MASS, _T_MAX, _Q_MAX, _L_ROLL, _L_PITCH, _KDV, _KDW, _H0;
	double _LAT0, _LON0, _COS_LAT0;
	Vector3f _W_I; 	// weight of the vehicle in inertial frame [N]
	Matrix3f _I; 	// vehicle inertia matrix
	Matrix3f _Im1; 	// inverse of the intertia matrix
	Vector3f _mu_I;	// NED magnetic field in inertial frame [G]

	// parameters defined in sih_params.c
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SIH_MASS>) _sih_mass,
		(ParamFloat<px4::params::SIH_IXX>) _sih_ixx,
		(ParamFloat<px4::params::SIH_IYY>) _sih_iyy,
		(ParamFloat<px4::params::SIH_IZZ>) _sih_izz,
		(ParamFloat<px4::params::SIH_IXY>) _sih_ixy,
		(ParamFloat<px4::params::SIH_IXZ>) _sih_ixz,
		(ParamFloat<px4::params::SIH_IYZ>) _sih_iyz,
		(ParamFloat<px4::params::SIH_T_MAX>) _sih_t_max,
		(ParamFloat<px4::params::SIH_Q_MAX>) _sih_q_max,
		(ParamFloat<px4::params::SIH_L_ROLL>) _sih_l_roll,
		(ParamFloat<px4::params::SIH_L_PITCH>) _sih_l_pitch,
		(ParamFloat<px4::params::SIH_KDV>) _sih_kdv,
		(ParamFloat<px4::params::SIH_KDW>) _sih_kdw,
		(ParamInt<px4::params::SIH_LAT0>) _sih_lat0,
		(ParamInt<px4::params::SIH_LON0>) _sih_lon0,
		(ParamFloat<px4::params::SIH_H0>) _sih_h0,
		(ParamFloat<px4::params::SIH_MU_X>) _sih_mu_x,
		(ParamFloat<px4::params::SIH_MU_Y>) _sih_mu_y,
		(ParamFloat<px4::params::SIH_MU_Z>) _sih_mu_z
	)
};
