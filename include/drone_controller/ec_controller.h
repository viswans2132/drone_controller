#ifndef DRONE_CONTROLLER_EC_CONTROLLER_H_
#define DRONE_CONTROLLER_EC_CONTROLLER_H_

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
// #include <geometric_msgs/Pose.h>

#include "drone_controller/common_operations.h"

namespace drone_controller{
	class EcController{
	public:
		EcController();
		~EcController();
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
		Eigen::Vector3d lam_p_;
		Eigen::Vector3d eHat_p_;
		Eigen::Vector3d eeta_p_;
		double alpha_a_p_;
		double alpha_b_p_;
		double var_pi_p_;
		Eigen::Vector3d rho_0a_q_;
		Eigen::Vector3d rho_0b_q_;
		Eigen::Vector3d rho_ssa_q_;
		Eigen::Vector3d rho_ssb_q_;
		Eigen::Vector3d lam_q_;
		Eigen::Vector3d eHat_q_;
		Eigen::Vector3d cHat_q_;
		Eigen::Vector3d eeta_q_;
		double alpha_a_q_;
		double alpha_b_q_;
		double var_pi_q_;
		Eigen::Vector4d mass_inertia_;
		Eigen::Matrix4Xd allocation_matrix_;
		RotorConfiguration rotor_config_;
		double max_thrust_;
		double gravity_;

		bool controller_active_;

	private:
		bool initialized_params_;
		Eigen::MatrixX4d thrust_moments_rpms_;

		mav_msgs::EigenTrajectoryPoint com_traj_;
		EigenOdometry odometry_;


		void ComputeDesiredMoments(const Eigen::Vector3d& forces,
		                            Eigen::Vector3d* moments);
		void ComputeDesiredForces(Eigen::Vector3d* forces) ;

	};




}

#endif