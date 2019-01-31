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

/**
 * @file sih.cpp
 * Simulator in Hardware
 * 
 * @author Romain Chiappinelli		<romain.chiap@gmail.com>
 *
 * Coriolis g Corporation - January 2019
 */

#include "sih.hpp"

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>

#include <drivers/drv_pwm_output.h>	// to get PWM flags
// #include <drivers/drv_sensor.h> 	// to get sensors address

#include <unistd.h> 			//
#include <string.h>    			//
#include <fcntl.h> 				//
#include <termios.h> 			//

using namespace math;

int Sih::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Simulator in Hardware
This module provide a simulator for quadrotors running fully 
inside the hardware autopilot.

This simulator subscribes to "actuator_outputs" which are the actuator pwm
signals given by the mixer.

This simulator publishes the sensors signals corrupted with realistic noise
in order to incorporate the state estimator in the loop.

### Implementation
The simulator implements the equations of motion using matrix algebra. 
Quaternion representation is used for the attitude.
Forward Euler is used for integration.
Most of local variables are declared global in the .hpp to avoid stack overflow.


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sih", "sih");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 4096, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int Sih::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int Sih::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int Sih::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("sih",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      4096,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Sih *Sih::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	Sih *instance = new Sih(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

Sih::Sih(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void Sih::run()
{
	
	// to subscribe to (read) the actuators_out pwm
	int actuator_out_sub = orb_subscribe(ORB_ID(actuator_outputs));

	// initialize parameters
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	parameters_update_poll(parameter_update_sub);


	init_variables();
 	init_sensors();	
	// "/dev/ttyS2/" is TELEM2 UART3 --- "/dev/ttyS5/" is Debug UART7 --- "/dev/ttyS4/" is OSD UART8
	int serial_fd=init_serial_port("/dev/ttyS5/", 57600); 	 	// init and open the serial port

	const hrt_abstime task_start = hrt_absolute_time();
	hrt_abstime last_run = task_start;
	hrt_abstime gps_time = task_start;
	hrt_abstime serial_time = task_start;
	hrt_abstime now;

	while (!should_exit()) {

		now = hrt_absolute_time();
		dt = (now - last_run) * 1e-6f;
		last_run = now;
		
		read_motors(actuator_out_sub);

		generate_force_and_torques();

		equations_of_motion();

		reconstruct_sensors_signals();

		send_IMU(now);

		if (now - gps_time > 50000) 	// gps published at 20Hz
		{
			gps_time=now;
			send_gps(gps_time);				
		}		
		
		// send uart message every 40 ms
		if (now - serial_time > 40000)
		{
			serial_time=now;

			publish_sih(); 	// publish sih message for debug purpose

			send_serial_msg(serial_fd, (int64_t)(now - task_start)/1000); 	

			parameters_update_poll(parameter_update_sub); 	// update the parameters if needed
		}
		// else if (loop_count==5)
		// {
		// 	tcflush(serial_fd, TCOFLUSH); 	// flush output data
		// 	tcdrain(serial_fd);
		// }

		usleep(1000); 	// sleeping time us

	}

	orb_unsubscribe(actuator_out_sub);
	orb_unsubscribe(parameter_update_sub);
	close(serial_fd);
	
}

void Sih::parameters_update_poll(int parameter_update_sub)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(parameter_update_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
		updateParams();
		parameters_updated();		
	}
}

// store the parameters in a more convenient form
void Sih::parameters_updated()
{

	T_MAX = _sih_t_max.get();
	Q_MAX = _sih_q_max.get();
	L = _sih_arm_length.get();
	KDV = _sih_kdv.get();
	KDW = _sih_kdw.get();
	H0 = _sih_h0.get();

	LAT0 = (double)_sih_lat0.get()*1.0e-7;
	LON0 = (double)_sih_lon0.get()*1.0e-7;
	COS_LAT0=cosl(radians(LAT0)); 

	MASS=_sih_mass.get();

	_W_I=Vector3f(0.0f,0.0f,MASS*CONSTANTS_ONE_G);
	
	_I=diag(Vector3f(_sih_ixx.get(),_sih_iyy.get(),_sih_izz.get()));
	_I(0,1)=_I(1,0)=_sih_ixy.get();
	_I(0,2)=_I(2,0)=_sih_ixz.get();
	_I(1,2)=_I(2,1)=_sih_iyz.get();

	_Im1=inv(_I);

	_mu_I=Vector3f(_sih_mu_x.get(), _sih_mu_y.get(), _sih_mu_z.get());

}

// initialization of the variables for the simulator
void Sih::init_variables()
{
	srand(1234); 	// initialize the random seed once before calling generate_wgn()

	p_I=Vector3f(0.0f,0.0f,0.0f);
	v_I=Vector3f(0.0f,0.0f,0.0f);
	q=Quatf(1.0f,0.0f,0.0f,0.0f);
	w_B=Vector3f(0.0f,0.0f,0.0f);

	u[0]=u[1]=u[2]=u[3]=0.0f;

}

void Sih::init_sensors()
{

	sensor_accel.device_id=1;
	sensor_accel.error_count=0;		
	sensor_accel.integral_dt=0;
	sensor_accel.temperature=15.0f;
	sensor_accel.scaling=1.0f/1024.0f;

	sensor_gyro.device_id=1;
	sensor_gyro.error_count=0;		
	sensor_gyro.integral_dt=0;
	sensor_gyro.temperature=15.0f;
	sensor_gyro.scaling=1.0f/1024.0f;

	sensor_mag.device_id=1;
	sensor_mag.error_count=0;		
	sensor_mag.temperature=15.0f;
	sensor_mag.scaling=1.0f/1024.0f;		
	sensor_mag.is_external=false;

	sensor_baro.error_count=0;
	sensor_baro.device_id=1;

	vehicle_gps_pos.fix_type=3; 	// 3D fix
	vehicle_gps_pos.satellites_used=8;
	vehicle_gps_pos.heading=NAN;
	vehicle_gps_pos.heading_offset=NAN;
	vehicle_gps_pos.s_variance_m_s = 0.5f;
	vehicle_gps_pos.c_variance_rad = 0.1f;
	vehicle_gps_pos.eph = 1.2f; //0.8f;
	vehicle_gps_pos.epv = 1.5f; //1.2f;
	vehicle_gps_pos.hdop = 0.7f;
	vehicle_gps_pos.vdop = 1.1f;	
}

int Sih::init_serial_port(const char uart_name[20], uint32_t speed)
{
	struct termios uart_config;
	int serial_fd = open(uart_name, O_WRONLY | O_NONBLOCK | O_NOCTTY);
	if (serial_fd < 0) {
		PX4_ERR("failed to open port: %s", uart_name);
	}

	tcgetattr(serial_fd, &uart_config); // read configuration

	uart_config.c_oflag |= ONLCR;
	// try to set Bauds rate
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_WARN("ERR SET BAUD %s\n", uart_name);
		close(serial_fd);
	}	

	tcsetattr(serial_fd, TCSANOW, &uart_config); 	// set config

	return serial_fd;
}

// read the motor signals outputted from the mixer
void Sih::read_motors(const int actuator_out_sub)
{
	struct actuator_outputs_s actuators_out {};

	// read the actuator outputs
	bool updated;
	orb_check(actuator_out_sub, &updated);

	if (updated) {		
		orb_copy(ORB_ID(actuator_outputs), actuator_out_sub, &actuators_out);
		for (int i=0; i<NB_MOTORS; i++) 	// saturate the motor signals
			u[i]=constrain((actuators_out.output[i]-PWM_DEFAULT_MIN)/(PWM_DEFAULT_MAX-PWM_DEFAULT_MIN),0.0f, 1.0f);
	}
}

// generate the motors thrust and torque in the body frame
void Sih::generate_force_and_torques()
{
	T_B=Vector3f(0.0f,0.0f, -T_MAX * (+u[0]+u[1]+u[2]+u[3]));
	Mt_B=Vector3f(	L*SQRT_2_O_2*T_MAX*(-u[0]+u[1]+u[2]-u[3]),
							L*SQRT_2_O_2*T_MAX*(+u[0]-u[1]+u[2]-u[3]),
									   Q_MAX * (+u[0]+u[1]-u[2]-u[3]));

	Fa_I=-KDV*v_I; 		// first order drag to slow down the aircraft
	Mt_B=Mt_B-KDW*w_B; 	// angular damper
}

// apply the equations of motion of a rigid body and integrate one step
void Sih::equations_of_motion()
{
	C_IB=q.to_dcm(); 	// body to inertial transformation 

	// Equations of motion of a rigid body
	p_I_dot=v_I; 							// position differential
	v_I_dot=(_W_I+Fa_I+C_IB*T_B)/MASS; 			// conservation of linear momentum
	q_dot=q.derivative1(w_B); 				// attitude differential
	w_B_dot=_Im1*(Mt_B-w_B.cross(_I*w_B));	// conservation of angular momentum

	// fake ground, avoid free fall
	if(p_I(2)>0.0f && v_I_dot(2)>0.0f)
	{
		v_I.setZero();
		w_B.setZero(); 
		v_I_dot.setZero();
	}
	else
	{
		// integration: Euler forward
		p_I = p_I + p_I_dot*dt;
		v_I = v_I + v_I_dot*dt;
		q = q+q_dot*dt; q.normalize(); 	// as given in attitude_estimator_q_main.cpp
		w_B = w_B + w_B_dot*dt;

	}
}

// reconstruct the noisy sensor signals
void Sih::reconstruct_sensors_signals()
{
	// IMU
	acc=C_IB.transpose()*(v_I_dot-Vector3f(0.0f,0.0f,CONSTANTS_ONE_G))+noiseGauss3f(0.5f,1.5f,1.5f);
	mag=C_IB.transpose()*_mu_I+noiseGauss3f(0.02f,0.02f,0.1f);
	gyro=w_B+noiseGauss3f(0.1f,0.1f,0.03f);

	// barometer
	float altitude=(H0-p_I(2))+generate_wgn()*0.15f; 	// altitude with noise
	baro_p_mBar=CONSTANTS_STD_PRESSURE_MBAR*powf((1.0f+altitude*a/T1),-CONSTANTS_ONE_G/(a*CONSTANTS_AIR_GAS_CONST)); 	// pressure in mBar
	baro_temp_c=T1+CONSTANTS_ABSOLUTE_NULL_CELSIUS+a*altitude+generate_wgn()*0.2f; 	// reconstructed temperture in celcius

	// GPS
	gps_lat=LAT0+degrees((double)p_I(0)/CONSTANTS_RADIUS_OF_EARTH)+(double)(generate_wgn()*1e-5f);  			// latitude in degrees
	gps_lon=LON0+degrees((double)p_I(1)/CONSTANTS_RADIUS_OF_EARTH)/COS_LAT0+(double)(generate_wgn()*1e-5f); 	// longitude in degrees
	gps_alt=H0-p_I(2)+generate_wgn()*1.78f;
	gps_vel=v_I+noiseGauss3f(0.07f,0.07f,0.16f);
}

void Sih::send_IMU(hrt_abstime now)
{
	sensor_accel.timestamp=now;
	sensor_accel.x=acc(0);
	sensor_accel.y=acc(1);	
	sensor_accel.z=acc(2);
	if (sensor_accel_pub != nullptr) {
		orb_publish(ORB_ID(sensor_accel), sensor_accel_pub, &sensor_accel);
	} else {
		sensor_accel_pub = orb_advertise(ORB_ID(sensor_accel), &sensor_accel);
	}

	sensor_gyro.timestamp=now;
	sensor_gyro.x=gyro(0);
	sensor_gyro.y=gyro(1);	
	sensor_gyro.z=gyro(2);
	if (sensor_gyro_pub != nullptr) {
		orb_publish(ORB_ID(sensor_gyro), sensor_gyro_pub, &sensor_gyro);
	} else {
		sensor_gyro_pub = orb_advertise(ORB_ID(sensor_gyro), &sensor_gyro);
	}

	sensor_mag.timestamp=now;
	sensor_mag.x=mag(0);
	sensor_mag.y=mag(1);
	sensor_mag.z=mag(2);
	if (sensor_mag_pub != nullptr) {
		orb_publish(ORB_ID(sensor_mag), sensor_mag_pub, &sensor_mag);
	} else {
		sensor_mag_pub = orb_advertise(ORB_ID(sensor_mag), &sensor_mag);
	}

	sensor_baro.timestamp=now;
	sensor_baro.pressure=baro_p_mBar;
	sensor_baro.temperature=baro_temp_c;
	if (sensor_baro_pub != nullptr) {
		orb_publish(ORB_ID(sensor_baro), sensor_baro_pub, &sensor_baro);
	} else {
		sensor_baro_pub = orb_advertise(ORB_ID(sensor_baro), &sensor_baro);
	}
}

void Sih::send_gps(hrt_abstime now)
{
	vehicle_gps_pos.timestamp=now; 			
	vehicle_gps_pos.lat=(int32_t)(gps_lat*1e7); 			// Latitude in 1E-7 degrees	
	vehicle_gps_pos.lon=(int32_t)(gps_lon*1e7);	// Longitude in 1E-7 degrees
	vehicle_gps_pos.alt=(int32_t)(gps_alt*1000.0f); 	// Altitude in 1E-3 meters above MSL, (millimetres)
	vehicle_gps_pos.alt_ellipsoid = (int32_t)((H0-p_I(2))*1000); 	// Altitude in 1E-3 meters bove Ellipsoid, (millimetres)
	vehicle_gps_pos.vel_ned_valid=true;				// True if NED velocity is valid
	vehicle_gps_pos.vel_m_s=sqrtf(gps_vel(0)*gps_vel(0)+gps_vel(1)*gps_vel(1));	// GPS ground speed, (metres/sec)
	vehicle_gps_pos.vel_n_m_s=gps_vel(0);				// GPS North velocity, (metres/sec)
	vehicle_gps_pos.vel_e_m_s=gps_vel(1);				// GPS East velocity, (metres/sec)
	vehicle_gps_pos.vel_d_m_s=gps_vel(2);				// GPS Down velocity, (metres/sec)
	vehicle_gps_pos.cog_rad=atan2(gps_vel(1),gps_vel(0));	// Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)
	if (vehicle_gps_pos_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_gps_position), vehicle_gps_pos_pub, &vehicle_gps_pos);
	} else {
		vehicle_gps_pos_pub = orb_advertise(ORB_ID(vehicle_gps_position), &vehicle_gps_pos);
	}
}

void Sih::publish_sih()
{

	Eulerf Euler(q);
	sih.timestamp=hrt_absolute_time();
	sih.dt_us=(uint32_t)(dt*1e6f);
	sih.euler_rpy[0]=degrees(Euler(0));
	sih.euler_rpy[1]=degrees(Euler(1));
	sih.euler_rpy[2]=degrees(Euler(2));
	sih.omega_b[0]=w_B(0); 	// wing body rates in body frame
	sih.omega_b[1]=w_B(1);
	sih.omega_b[2]=w_B(2);
	sih.p_i_local[0]=p_I(0); 	// local inertial position
	sih.p_i_local[1]=p_I(1);
	sih.p_i_local[2]=p_I(2);
	sih.v_i[0]=v_I(0); 	// inertial velocity
	sih.v_i[1]=v_I(1);
	sih.v_i[2]=v_I(2);
	sih.u[0]=u[0];
	sih.u[1]=u[1];
	sih.u[2]=u[2];
	sih.u[3]=u[3];
	if (sih_pub != nullptr) {
		orb_publish(ORB_ID(sih), sih_pub, &sih);
	} else {
		sih_pub = orb_advertise(ORB_ID(sih), &sih);
	}
} 

void Sih::send_serial_msg(int serial_fd, int64_t t_ms)
{

	char uart_msg[100];
	uint8_t n;
	int32_t EA_deg[4]; 		// Euler angles in degrees integers to send to serial
	int32_t p_I_cm[3];		// inertial position in cm to send to serial
	int32_t deflections[2]; 	// control surface deflection [-100;+100] to send to serial
	int32_t throttles[4]; 	// throttles from 0 to 99

	Eulerf Euler(q);
	EA_deg[0]=(int32_t)degrees(Euler(0)*10.0f); 	// decidegrees
	EA_deg[1]=(int32_t)degrees(Euler(1)*10.0f);
	EA_deg[2]=(int32_t)degrees(Euler(2)*10.0f);
	EA_deg[3]=(int32_t)0;
	p_I_cm[0]=(int32_t)(p_I(0)*100.0f); 			// centimeters cm
	p_I_cm[1]=(int32_t)(p_I(1)*100.0f);
	p_I_cm[2]=(int32_t)(p_I(2)*100.0f);
	deflections[0]=(int32_t)0; 	// [-100;100]
	deflections[1]=(int32_t)0;
	throttles[0]=(int32_t)(u[0]*99.0f);
	throttles[1]=(int32_t)(u[1]*99.0f);
	throttles[2]=(int32_t)(u[2]*99.0f);
	throttles[3]=(int32_t)(u[3]*99.0f);

	n = sprintf(uart_msg, "T%07lld,P%+07d%+07d%+07d,A%+05d%+05d%+05d%+05d,D%+04d%+04d,U%+03d%+03d%+03d%+03d\n", 
		t_ms,p_I_cm[0],p_I_cm[1],p_I_cm[2],
		EA_deg[0],EA_deg[1],EA_deg[2],EA_deg[3],
		deflections[0],deflections[1],throttles[0],throttles[1],throttles[2],throttles[3]);
	write(serial_fd, uart_msg, n);
}

float Sih::generate_wgn() 	// generate white Gaussian noise sample with std=1
{
	float temp=((float)(rand()+1))/(((float)RAND_MAX+1.0f));
 
	return sqrtf(-2.0f*logf(temp))*cosf(2.0f*M_PI_F*rand()/RAND_MAX);	
}

Vector3f Sih::noiseGauss3f(float stdx,float stdy, float stdz) 	// generate white Gaussian noise sample with specified std
{
	return Vector3f(generate_wgn()*stdx,generate_wgn()*stdy,generate_wgn()*stdz);	
} // there is another wgn algorithm in BlockRandGauss.hpp

int sih_main(int argc, char *argv[])
{
	return Sih::main(argc, argv);
}
