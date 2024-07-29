#ifndef DRONE_CONTROLLER_ASM_CONTROLLER_H_
#define DRONE_CONTROLLER_ASM_CONTROLLER_H_

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
// #include <geometric_msgs/Pose.h>

#include "drone_controller/common_operations.h"

namespace drone_controller{
	class AsmController{
	public:
		AsmController();
		~AsmController();
		void initParams();
		void getRPMs(Eigen::VectorXd* rpms);
		void getAttThrust(Eigen::VectorXd* att_thrust);
		void setTraj(const mav_msgs::EigenTrajectoryPoint& com_traj);
		// void setOdometryFromPose(const geometric_msgs::Pose& pose);
		void setOdometry(const EigenOdometry& odom);

		Eigen::Vector3d hatKp0_;
		Eigen::Vector3d hatKp1_;
		Eigen::Vector3d hatKq0_;
		Eigen::Vector3d hatKq1_;
		Eigen::Vector3d hatKq2_;

		Eigen::Vector3d alpha_p0_;
		Eigen::Vector3d alpha_p1_;
		Eigen::Vector3d alpha_q0_;
		Eigen::Vector3d alpha_q1_;
		Eigen::Vector3d alpha_q2_;

		Eigen::Vector3d lam_p_;
		Eigen::Vector3d lam_q_;

		Eigen::Vector3d theta_p_;
		Eigen::Vector3d theta_q_;

		double hatM_;
		double max_thrust_;
		double alpha_m_;
		double var_pi_p_;
		double var_pi_q_;

		Eigen::Matrix4Xd allocation_matrix_;
		RotorConfiguration rotor_config_;
		double gravity_;

		bool controller_active_;

	private:
		bool initialized_params_;

		//double test_mass; //Added by Viswa

		Eigen::MatrixX4d rotor_vel_coef_;
		mav_msgs::EigenTrajectoryPoint com_traj_;
		EigenOdometry odometry_;


		void ComputeDesiredMoments(const Eigen::Vector3d& forces,
		                            Eigen::Vector3d* angular_acceleration);
		void ComputeDesiredForces(Eigen::Vector3d* forces) ;

	};




}

#endif