#include <ros/ros.h>
#include <std_srvs/Empty.h>

// msgs
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
// crazyflie
#include <crazyflie_controller/CrazyflieState.h>
#include <crazyflie_controller/PropellerSpeeds.h>
#include <crazyflie_controller/CrazyflieStateStamped.h>
#include <crazyflie_controller/PropellerSpeedsStamped.h>
#include <crazyflie_controller/CrazyflieOpenloopTraj.h>
#include <crazyflie_controller/GenericLogData.h>

// Dynamic reconfirgure
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>
#include "boost/thread/mutex.hpp"
// crazyflie
#include <crazyflie_controller/crazyflie_paramsConfig.h>

// Matrices and vectors
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// standard
#include <iostream>
#include <sstream>
#include <fstream>
#include <ios>

// acados
#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

// crazyflie specific
#include "crazyflie_model/crazyflie_model.h"
#include "acados_solver_crazyflie.h"

// global data
ocp_nlp_in * nlp_in;
ocp_nlp_out * nlp_out;
ocp_nlp_solver * nlp_solver;
void * nlp_opts;
ocp_nlp_plan * nlp_solver_plan;
ocp_nlp_config * nlp_config;
ocp_nlp_dims * nlp_dims;

external_function_casadi * forw_vde_casadi;

using namespace Eigen;
using std::ofstream;
using std::cout;
using std::endl;
using std::fixed;
using std::showpos;

// acados dims

// Number of intervals in the horizon
#define N 50
// Number of differential state variables
#define NX 13
// Number of control inputs
#define NU 4
// Number of measurements/references on nodes 0..N-1
#define NY 17
// Number of measurements/references on node N
#define NYN 13
// Constants
#define pi  3.14159265358979323846
#define g0 9.80665

#define CONTROLLER 1
#define FIXED_U0 0
#define READ_CASADI_TRAJ 0
#define WRITE_OPENLOOP_TRAJ 1
#define WRITE_FULL_LOG 0

class NMPC
	{
	enum systemStates{
		xq = 0,
		yq = 1,
		zq = 2,
		q1 = 3,
		q2 = 4,
		q3 = 5,
		q4 = 6,
		vbx = 7,
		vby = 8,
		vbz = 9,
		wx = 10,
		wy = 11,
		wz = 12
	};

	enum controlInputs{
		w1 = 0,
		w2 = 1,
		w3 = 2,
		w4 = 3
	};

	enum reference_mode{
		Regulation = 0,
		Tracking = 1,
		Position_Hold = 2
	};

	struct euler{
		double phi;
		double theta;
		double psi;
	};

	struct solver_output{
		double status, KKT_res, cpu_time;
		double u0[NU];
		double u1[NU];
		double x1[NX];
		double x2[NX];
		double xi[NU];
		double ui[NU];
	};

	struct solver_input{
		double x0[NX];
		double yref[(NY*N)];
		double yref_e[NYN];
		double W[NY*NY];
		double WN[NX*NX];
	};

	ros::Publisher p_motvel;
	ros::Publisher p_bodytwist;
	// trajectory
	ros::Publisher p_ol_traj;

	ros::Subscriber s_estimator;

	ros::Subscriber s_imu_sub;
	ros::Subscriber s_eRaptor_sub;
	ros::Subscriber s_euler_sub;
	ros::Subscriber s_motors;

	// Variables for joy callback
	double joy_roll,joy_pitch,joy_yaw;
	double joy_thrust;


	unsigned int k,i,j,ii;

	float uss,Ct,mq;

	// Variables of the nmpc control process
	double x0_sign[NX];
	double yref_sign[(NY*N)+NY];

	// Variables for dynamic reconfigure
	double xq_des, yq_des, zq_des;

	// acados struct
	solver_input acados_in;
	solver_output acados_out;
	int acados_status;

	reference_mode policy;

	// Variable for storing he optimal trajectory
	// Variable for storing he optimal trajectory
	std::vector<std::vector<double>> precomputed_traj;
	int N_STEPS,iter;

public:

	NMPC(ros::NodeHandle& n, const std::string& ref_traj)
		{

		int status = 0;
		status = acados_create();

		if (status){
			ROS_INFO_STREAM("acados_create() returned status " << status << ". Exiting." << endl);
			exit(1);
		}

		// publisher for the real robot inputs (thrust, roll, pitch, yawrate)
		p_bodytwist = n.advertise<geometry_msgs::Twist>("/crazyflie/cmd_vel", 1);

		// publisher for the control inputs of acados (motor speeds to be applied)
		p_motvel = n.advertise<crazyflie_controller::PropellerSpeedsStamped>("/crazyflie/acados_motvel", 1);

		// solution
		p_ol_traj = n.advertise<crazyflie_controller::CrazyflieOpenloopTraj>("/cf_mpc/openloop_traj",1);

		// subscriber of estimator state
		s_estimator = n.subscribe("/cf_estimator/state_estimate", 5, &NMPC::iteration, this);

		// Initializing control inputs
		for(unsigned int i=0; i < NU; i++) acados_out.u1[i] = 0.0;

		// Steady-state control input value
		// (Kg)
		mq = 33e-3;
		// (N/kRPM**2)
		Ct = 3.25e-4;
		// steady state prop speed (kRPM)
		uss = sqrt((mq*g0)/(4*Ct));

		const char * c = ref_traj.c_str();
		// Pre-load the trajectory
		N_STEPS = readDataFromFile(c, precomputed_traj);
		if (N_STEPS == 0){
			ROS_WARN("Cannot load CasADi optimal trajectory!");
		}
		else{
			ROS_INFO_STREAM("Number of steps: " << N_STEPS << endl);
		}
		//

		// Set number of trajectory iterations to zero initially
		iter = 0;
		}

	void run()
		{
		ROS_DEBUG("Setting up the dynamic reconfigure panel and server");
		dynamic_reconfigure::Server<crazyflie_controller::crazyflie_paramsConfig> server;
		dynamic_reconfigure::Server<crazyflie_controller::crazyflie_paramsConfig>::CallbackType f;
		f = boost::bind(&NMPC::callback_dynamic_reconfigure, this, _1, _2);
		server.setCallback(f);

		ros::spin();
		}

	void callback_dynamic_reconfigure(crazyflie_controller::crazyflie_paramsConfig &config, uint32_t level)
		{
		if (level)
			{
			if(config.enable_traj_tracking)
				{
				ROS_INFO_STREAM("Tracking trajectory");
				config.enable_regulation = false;
				policy = Tracking;
				}
			if(config.enable_regulation)
				{
				config.enable_traj_tracking = false;
				xq_des = config.xq_des;
				yq_des = config.yq_des;
				zq_des = config.zq_des;
				policy = Regulation;
				}
			}

		ROS_INFO_STREAM(
			fixed << showpos << "Quad status" << endl
			<< "NMPC for regulation: " <<  (config.enable_regulation?"ON":"off") << endl
			<< "NMPC trajectory tracker: " <<  (config.enable_traj_tracking?"ON":"off") << endl
			<< "Current regulation point: " << xq_des << ", " << yq_des << ", " << zq_des << endl
		);
		}

	int readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data)
		{
		std::ifstream file(fileName);
		std::string line;
		int num_of_steps = 0;

		if (file.is_open())
			{
			while(getline(file, line)){
				++num_of_steps;
				std::istringstream linestream( line );
				std::vector<double> linedata;
				double number;

				while( linestream >> number ){
					linedata.push_back( number );
				}
				data.push_back( linedata );
			}

			file.close();
			cout << num_of_steps << endl;
			}
		else
			{
			return 0;
			}

		return num_of_steps;
		}

	euler quatern2euler(Quaterniond* q)
		{

		euler angle;

		double R11 = q->w()*q->w()+q->x()*q->x()-q->y()*q->y()-q->z()*q->z();
		double R21 = 2*(q->x()*q->y()+q->w()*q->z());
		double R31 = 2*(q->x()*q->z()-q->w()*q->y());
		double R32 = 2*(q->y()*q->z()+q->w()*q->x());
		double R33 = q->w()*q->w()-q->x()*q->x()-q->y()*q->y()+q->z()*q->z();

		double phi	 = atan2(R32, R33);
		double theta = asin(-R31);
		double psi	 = atan2(R21, R11);

		angle.phi	= phi;
		angle.theta = theta;
		angle.psi	= psi;

		return angle;
		}

	Quaterniond euler2quatern(euler angle)
		{

		Quaterniond q;

		double cosPhi = cos(angle.phi*0.5);
		double sinPhi = sin(angle.phi*0.5);

		double cosTheta = cos(angle.theta*0.5);
		double sinTheta = sin(angle.theta*0.5);

		double cosPsi = cos(angle.psi*0.5);
		double sinPsi = sin(angle.psi*0.5);

		// Convention according the firmware of the crazyflie
		q.w() = cosPsi*cosTheta*cosPhi + sinPsi*sinTheta*sinPhi;
		q.x() = -(cosPsi*cosTheta*sinPhi - sinPsi*sinTheta*cosPhi);
		q.y() = -(cosPsi*sinTheta*cosPhi + sinPsi*cosTheta*sinPhi);
		q.z() = -(sinPsi*cosTheta*cosPhi - cosPsi*sinTheta*sinPhi);

		if(q.w() < 0){
			q.w() = -q.w();
			q.vec() = -q.vec();
		}

		return q;
		}

	double deg2Rad(double deg)
		{
		return deg / 180.0 * pi;
		}

	double rad2Deg(double rad)
		{
		return rad * 180.0 / pi;
		}

	void nmpcReset()
		{
		acados_free();
		}

	int krpm2pwm(double Krpm)
		{
		int pwm = ((Krpm*1000)-4070.3)/0.2685;
		return pwm;
		}

	void iteration(const crazyflie_controller::CrazyflieStateStampedPtr& msg)
		{
		ROS_INFO_STREAM("mpc iteration");
		ROS_WARN("mpc iteration");
		try
			{

			if(iter > N_STEPS)
				{
				ROS_INFO_STREAM("Trajectory is over");
				// ros::shutdown();
				}

			# if 0
			if(iter >= N_STEPS-N)
				{
				for (k = 0; k < N_STEPS-iter; k++)
					{
					acados_in.yref[k * NY + 0]  = precomputed_traj[iter + k][xq];
					acados_in.yref[k * NY + 1]  = precomputed_traj[iter + k][yq];
					acados_in.yref[k * NY + 2]  = precomputed_traj[iter + k][zq];
					acados_in.yref[k * NY + 3]  = precomputed_traj[iter + k][q1];
					acados_in.yref[k * NY + 4]  = precomputed_traj[iter + k][q2];
					acados_in.yref[k * NY + 5]  = precomputed_traj[iter + k][q3];
					acados_in.yref[k * NY + 6]  = precomputed_traj[iter + k][q4];
					acados_in.yref[k * NY + 7]  = precomputed_traj[iter + k][vbx];
					acados_in.yref[k * NY + 8]  = precomputed_traj[iter + k][vby];
					acados_in.yref[k * NY + 9]  = precomputed_traj[iter + k][vbz];
					acados_in.yref[k * NY + 10] = precomputed_traj[iter + k][wx];
					acados_in.yref[k * NY + 11] = precomputed_traj[iter + k][wy];
					acados_in.yref[k * NY + 12] = precomputed_traj[iter + k][wz];
					acados_in.yref[k * NY + 13] = precomputed_traj[iter + k][13];
					acados_in.yref[k * NY + 14] = precomputed_traj[iter + k][14];
					acados_in.yref[k * NY + 15] = precomputed_traj[iter + k][15];
					acados_in.yref[k * NY + 16] = precomputed_traj[iter + k][16];
					}
				for (k=N_STEPS-iter; k < N+1; k++)
					{
					acados_in.yref[k * NY + 0]  = precomputed_traj[N_STEPS][xq];
					acados_in.yref[k * NY + 1]  = precomputed_traj[N_STEPS][yq];
					acados_in.yref[k * NY + 2]  = precomputed_traj[N_STEPS][zq];
					acados_in.yref[k * NY + 3]  = precomputed_traj[N_STEPS][q1];
					acados_in.yref[k * NY + 4]  = precomputed_traj[N_STEPS][q2];
					acados_in.yref[k * NY + 5]  = precomputed_traj[N_STEPS][q3];
					acados_in.yref[k * NY + 6]  = precomputed_traj[N_STEPS][q4];
					acados_in.yref[k * NY + 7]  = precomputed_traj[N_STEPS][vbx];
					acados_in.yref[k * NY + 8]  = precomputed_traj[N_STEPS][vby];
					acados_in.yref[k * NY + 9]  = precomputed_traj[N_STEPS][vbz];
					acados_in.yref[k * NY + 10] = precomputed_traj[N_STEPS][wx];
					acados_in.yref[k * NY + 11] = precomputed_traj[N_STEPS][wy];
					acados_in.yref[k * NY + 12] = precomputed_traj[N_STEPS][wz];
					acados_in.yref[k * NY + 13] = precomputed_traj[N_STEPS][13];
					acados_in.yref[k * NY + 14] = precomputed_traj[N_STEPS][14];
					acados_in.yref[k * NY + 15] = precomputed_traj[N_STEPS][15];
					acados_in.yref[k * NY + 16] = precomputed_traj[N_STEPS][16];
					}
				}

			for (k = 0; k < N+1; k++)
				{
				acados_in.yref[k * NY + 0]  = precomputed_traj[iter + k][xq];
				acados_in.yref[k * NY + 1]  = precomputed_traj[iter + k][yq];
				acados_in.yref[k * NY + 2]  = precomputed_traj[iter + k][zq];
				acados_in.yref[k * NY + 3]  = precomputed_traj[iter + k][q1];
				acados_in.yref[k * NY + 4]  = precomputed_traj[iter + k][q2];
				acados_in.yref[k * NY + 5]  = precomputed_traj[iter + k][q3];
				acados_in.yref[k * NY + 6]  = precomputed_traj[iter + k][q4];
				acados_in.yref[k * NY + 7]  = precomputed_traj[iter + k][vbx];
				acados_in.yref[k * NY + 8]  = precomputed_traj[iter + k][vby];
				acados_in.yref[k * NY + 9]  = precomputed_traj[iter + k][vbz];
				acados_in.yref[k * NY + 10] = precomputed_traj[iter + k][wx];
				acados_in.yref[k * NY + 11] = precomputed_traj[iter + k][wy];
				acados_in.yref[k * NY + 12] = precomputed_traj[iter + k][wz];
				acados_in.yref[k * NY + 13] = precomputed_traj[iter + k][13];
				acados_in.yref[k * NY + 14] = precomputed_traj[iter + k][14];
				acados_in.yref[k * NY + 15] = precomputed_traj[iter + k][15];
				acados_in.yref[k * NY + 16] = precomputed_traj[iter + k][16];
				}
			++iter;
			#endif

			for (k = 0; k < N; k++){
				acados_in.yref[k * NY + 0] = xq_des;	// xq
				acados_in.yref[k * NY + 1] = yq_des;	// yq
				acados_in.yref[k * NY + 2] = zq_des;	// zq
				acados_in.yref[k * NY + 3] = 1.00;		// q1
				acados_in.yref[k * NY + 4] = 0.00;		// q2
				acados_in.yref[k * NY + 5] = 0.00;		// q3
				acados_in.yref[k * NY + 6] = 0.00;		// q4
				acados_in.yref[k * NY + 7] = 0.00;		// vbx
				acados_in.yref[k * NY + 8] = 0.00;		// vby
				acados_in.yref[k * NY + 9] = 0.00;		// vbz
				acados_in.yref[k * NY + 10] = 0.00;	// wx
				acados_in.yref[k * NY + 11] = 0.00;	// wy
				acados_in.yref[k * NY + 12] = 0.00;	// wz
				acados_in.yref[k * NY + 13] = uss;		// w1
				acados_in.yref[k * NY + 14] = uss;		// w2
				acados_in.yref[k * NY + 15] = uss;		// w3
				acados_in.yref[k * NY + 16] = uss;		// w4
			}

			acados_in.yref_e[k * NY + 0]  = xq_des;	// xq
			acados_in.yref_e[k * NY + 1]  = yq_des;	// yq
			acados_in.yref_e[k * NY + 2]  = zq_des;	// zq
			acados_in.yref_e[k * NY + 3]  = 1.00;	// q1
			acados_in.yref_e[k * NY + 4]  = 0.00;	// q2
			acados_in.yref_e[k * NY + 5]  = 0.00;	// q3
			acados_in.yref_e[k * NY + 6]  = 0.00;	// q4
			acados_in.yref_e[k * NY + 7]  = 0.00;	// vbx
			acados_in.yref_e[k * NY + 8]  = 0.00;	// vby
			acados_in.yref_e[k * NY + 9]  = 0.00;	// vbz
			acados_in.yref_e[k * NY + 10] = 0.00;	// wx
			acados_in.yref_e[k * NY + 11] = 0.00;	// wy
			acados_in.yref_e[k * NY + 12] = 0.00;	// wz

			// read msg
			// position
			acados_in.x0[xq] = msg->pos.x;
			acados_in.x0[yq] = msg->pos.y;
			acados_in.x0[zq] = msg->pos.z;

			// quaternion
			acados_in.x0[q1] = msg->quat.w;
			acados_in.x0[q2] = msg->quat.x;
			acados_in.x0[q3] = msg->quat.y;
			acados_in.x0[q4] = msg->quat.z;

			// body velocities
			acados_in.x0[vbx] = msg->vel.x;
			acados_in.x0[vby] = msg->vel.y;
			acados_in.x0[vbz] = msg->vel.z;

			// rates
			acados_in.x0[wx] = msg->rates.x;
			acados_in.x0[wy] = msg->rates.y;
			acados_in.x0[wz] = msg->rates.z;

			//---------------------------------------
			// acados NMPC
			//---------------------------------------

			ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", acados_in.x0);
			ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", acados_in.x0);

			// for (i = 0; i < N; i++) {
				// for (j = 0; j < NY; ++j) {
					// acados_in.yref[i*NY + j] = yref_sign[i*NY + j];
				// }
			// }
			// for (i = 0; i < NYN; i++) {
				// acados_in.yref_e[i] = yref_sign[N*NY + i];
			// }

			for (ii = 0; ii < N; ii++)
				{
				ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "yref", acados_in.yref + ii*NY);
				}
			ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", acados_in.yref_e);

			// set constraints
			if (FIXED_U0 == 1) {
				ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbu", acados_out.u1);
				ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubu", acados_out.u1);
			}

			// call solver
			acados_status = acados_solve();

			// assign output signals
			acados_out.status = acados_status;
			acados_out.KKT_res = (double)nlp_out->inf_norm_res;
			acados_out.cpu_time = (double)nlp_out->total_time;

			// get solution
			ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", (void *)acados_out.u0);

			// get solution at stage 1
			ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "u", (void *)acados_out.u1);

			// get next stage
			ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", (void *)acados_out.x1);

			// get stage 2 which compensates 15 ms for the delay
			ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 2, "x", (void *)acados_out.x2);

			// publish acados output
			crazyflie_controller::PropellerSpeedsStamped propellerspeeds;
			propellerspeeds.header.stamp = ros::Time::now();

			if (FIXED_U0 == 1) {
				propellerspeeds.w1 =  acados_out.u1[w1];
				propellerspeeds.w2 =  acados_out.u1[w2];
				propellerspeeds.w3 =  acados_out.u1[w3];
				propellerspeeds.w4 =  acados_out.u1[w4];
			} else {
				propellerspeeds.w1 =  acados_out.u0[w1];
				propellerspeeds.w2 =  acados_out.u0[w2];
				propellerspeeds.w3 =  acados_out.u0[w3];
				propellerspeeds.w4 =  acados_out.u0[w4];
			}
			p_motvel.publish(propellerspeeds);

			// Select the set of optimal states to calculate the real cf control inputs
			Quaterniond q_acados_out;
			q_acados_out.w() = acados_out.x2[q1];
			q_acados_out.x() = acados_out.x2[q2];
			q_acados_out.y() = acados_out.x2[q3];
			q_acados_out.z() = acados_out.x2[q4];
			q_acados_out.normalize();

			// Convert acados output quaternion to desired euler angles
			euler eu_imu;
			eu_imu = quatern2euler(&q_acados_out);

			// Publish real control inputs
			geometry_msgs::Twist bodytwist;

			// linear_x -> pitch
			bodytwist.linear.x  = -rad2Deg(eu_imu.theta);
			// linear_y -> roll
			bodytwist.linear.y  = rad2Deg(eu_imu.phi);
			bodytwist.linear.z  = krpm2pwm(
				(acados_out.u1[w1]+acados_out.u1[w2]+acados_out.u1[w3]+acados_out.u1[w4])/4
			);
			bodytwist.angular.z = rad2Deg(acados_out.x1[wz]);
			if  ( bodytwist.linear.z > 64000 ) bodytwist.linear.z = 64000;

			p_bodytwist.publish(bodytwist);

			ROS_INFO_STREAM("Iteration: " << iter << endl);

			#if WRITE_OPENLOOP_TRAJ

			crazyflie_controller::CrazyflieOpenloopTraj traj_msg;
			traj_msg.header.stamp = ros::Time::now();
			traj_msg.cpu_time = acados_out.cpu_time;

			for(ii=0; ii< N; ii++)
				{
				ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", (void *)(acados_out.xi));
				ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", (void *)(acados_out.ui));

				crazyflie_controller::CrazyflieState crazyflie_state;
				crazyflie_controller::PropellerSpeeds crazyflie_control;

				crazyflie_state.pos.x    = acados_out.xi[xq];
				crazyflie_state.pos.y    = acados_out.xi[yq];
				crazyflie_state.pos.z    = acados_out.xi[zq];
				crazyflie_state.vel.x    = acados_out.xi[vbx];
				crazyflie_state.vel.y    = acados_out.xi[vby];
				crazyflie_state.vel.z    = acados_out.xi[vbz];
				crazyflie_state.quat.x   = acados_out.xi[q2];
				crazyflie_state.quat.y   = acados_out.xi[q3];
				crazyflie_state.quat.z   = acados_out.xi[q4];
				crazyflie_state.quat.w   = acados_out.xi[q1];
				crazyflie_state.rates.x  = acados_out.xi[wx];
				crazyflie_state.rates.y  = acados_out.xi[wy];
				crazyflie_state.rates.z  = acados_out.xi[wz];

				crazyflie_control.w1 = acados_out.ui[0];
				crazyflie_control.w2 = acados_out.ui[1];
				crazyflie_control.w3 = acados_out.ui[2];
				crazyflie_control.w4 = acados_out.ui[3];

				traj_msg.states.push_back(crazyflie_state);
				traj_msg.controls.push_back(crazyflie_control);
				}

			p_ol_traj.publish(traj_msg);
			#endif

			#if WRITE_FULL_LOG
			// Log current state x0 and acados output x1 and x2
			ofstream motorsLog("full_log.txt", std::ios_base::app | std::ios_base::out);
			if (motorsLog.is_open())
				{
				//	motorsLog << xq_des << " ";
				//	motorsLog << yq_des << " ";
				//	motorsLog << zq_des << " ";
				//	motorsLog << actual_roll << " ";
				//	motorsLog << -actual_pitch << " ";
				//	motorsLog << actual_yaw  << " ";
				//	motorsLog << msg.linear.y << " ";
				//	motorsLog << msg.linear.x << " ";
				//	motorsLog << msg.angular.z << " ";
				motorsLog << actual_m1 << " ";
				motorsLog << actual_m2 << " ";
				motorsLog << actual_m3 << " ";
				motorsLog << actual_m4 << " ";
				motorsLog << actual_x << " ";
				motorsLog << actual_y << " ";
				motorsLog << actual_z << " ";
				motorsLog << x0_sign[q1] << " ";
				motorsLog << x0_sign[q2] << " ";
				motorsLog << x0_sign[q3] << " ";
				motorsLog << x0_sign[q4] << " ";
				motorsLog << x0_sign[vbx] << " ";
				motorsLog << x0_sign[vby] << " ";
				motorsLog << x0_sign[vbz] << " ";
				motorsLog << x0_sign[wx] << " ";
				motorsLog << x0_sign[wy] << " ";
				motorsLog << x0_sign[wz] << " ";
				motorsLog << acados_out.x2[xq] << " ";
				motorsLog << acados_out.x2[yq] << " ";
				motorsLog << acados_out.x2[zq] << " ";
				motorsLog << acados_out.x2[q1] << " ";
				motorsLog << acados_out.x2[q2] << " ";
				motorsLog << acados_out.x2[q3] << " ";
				motorsLog << acados_out.x2[q4] << " ";
				motorsLog << acados_out.x2[vbx] << " ";
				motorsLog << acados_out.x2[vby] << " ";
				motorsLog << acados_out.x2[vbz] << " ";
				motorsLog << acados_out.x2[wx] << " ";
				motorsLog << acados_out.x2[wy] << " ";
				motorsLog << acados_out.x2[wz] << " ";
				motorsLog << acados_out.u1[w1] << " ";
				motorsLog << acados_out.u1[w2] << " ";
				motorsLog << acados_out.u1[w3] << " ";
				motorsLog << acados_out.u1[w4] << " ";
				motorsLog << msg.linear.z<< " ";
				// motorsLog << casadi_optimal_traj[iter][xq] << " ";
				// motorsLog << casadi_optimal_traj[iter][yq] << " ";
				// motorsLog << casadi_optimal_traj[iter][zq] << " ";
				// motorsLog << casadi_optimal_traj[iter][q1] << " ";
				// motorsLog << casadi_optimal_traj[iter][q2] << " ";
				// motorsLog << casadi_optimal_traj[iter][q3] << " ";
				// motorsLog << casadi_optimal_traj[iter][q4] << " ";
				// motorsLog << casadi_optimal_traj[iter][vbx] << " ";
				// motorsLog << casadi_optimal_traj[iter][vby] << " ";
				// motorsLog << casadi_optimal_traj[iter][vbz] << " ";
				// motorsLog << casadi_optimal_traj[iter][wx] << " ";
				// motorsLog << casadi_optimal_traj[iter][wy] << " ";
				// motorsLog << casadi_optimal_traj[iter][wz] << " ";
				// motorsLog << casadi_optimal_traj[iter][13] << " ";
				// motorsLog << casadi_optimal_traj[iter][14] << " ";
				// motorsLog << casadi_optimal_traj[iter][15] << " ";
				// motorsLog << casadi_optimal_traj[iter][16] << " ";
				motorsLog << endl;
				motorsLog.close();
				}
			#endif
			}

		catch (int acados_status)
			{
			ROS_INFO_STREAM("An exception occurred. Exception Nr. ");
			}
		}
	};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cf_nmpc");

	ros::NodeHandle n("~");

	std::string ref_traj;
	n.getParam("ref_traj", ref_traj);

	NMPC nmpc(n,ref_traj);
	nmpc.run();

	return 0;
}
