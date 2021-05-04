#ifndef DRONE_CONTROLLER_AEC_CONTROLLER_H_
#define DRONE_CONTROLLER_AEC_CONTROLLER_H_

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
// #include <geometric_msgs/Pose.h>

#include "drone_controller/common_operations.h"

namespace drone_controller{
	class AecController{
	public:
		AecController();
		~AecController();
		void initParams();
		void getRPMs(Eigen::VectorXd* rpms) ;
		void getAttThrust(Eigen::VectorXd* att_thrust);
		void setTraj(const mav_msgs::EigenTrajectoryPoint& com_traj);
		// void setOdometryFromPose(const geometric_msgs::Pose& pose);
		void setOdometry(const EigenOdometry& odom);

		Eigen::Vector3d rho_0a_p_;
		Eigen::Vector3d rho_0b_p_;
		Eigen::Vector3d rho_ssa_p_;
		Eigen::Vector3d rho_ssb_p_;
		double alpha_a_p_;
		double alpha_b_p_;
		Eigen::Vector3d lam_p_;
		Eigen::Vector3d theta_p_;
		Eigen::Vector3d hatK1a_p_;
		Eigen::Vector3d hatK2a_p_;
		Eigen::Vector3d hatK3a_p_;
		Eigen::Vector3d hatK1b_p_;
		Eigen::Vector3d hatK2b_p_;
		Eigen::Vector3d hatK3b_p_;
		Eigen::Vector3d hatK4b_p_;
		Eigen::Vector3d alphaK1a_p_;
		Eigen::Vector3d alphaK2a_p_;
		Eigen::Vector3d alphaK3a_p_;
		Eigen::Vector3d alphaK1b_p_;
		Eigen::Vector3d alphaK2b_p_;
		Eigen::Vector3d alphaK3b_p_;
		Eigen::Vector3d alphaK4b_p_;
		
		double var_pi_p_;
		Eigen::Vector3d Kp_q_;
		Eigen::Vector3d Kv_q_;
		Eigen::Vector4d mass_inertia_;
		Eigen::Matrix4Xd allocation_matrix_;
		RotorConfiguration rotor_config_;
		double max_thrust_;
		double gravity_;

		bool controller_active_;

	private:
		bool initialized_params_;

		Eigen::Vector3d normalized_attitude_gain_;
		Eigen::Vector3d normalized_angular_rate_gain_;
		Eigen::MatrixX4d ang_acc_rpms_;

		mav_msgs::EigenTrajectoryPoint com_traj_;
		EigenOdometry odometry_;


		void ComputeDesiredAngularAcc(const Eigen::Vector3d& forces,
		                            Eigen::Vector3d* angular_acceleration);
		void ComputeDesiredForces(Eigen::Vector3d* forces) ;

	};




}

#endif