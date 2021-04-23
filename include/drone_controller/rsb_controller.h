#ifndef DRONE_CONTROLLER_RSB_CONTROLLER_H_
#define DRONE_CONTROLLER_RSB_CONTROLLER_H_

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
// #include <geometric_msgs/Pose.h>

#include "drone_controller/common_operations.h"

namespace drone_controller{
	class RsbController{
	public:
		RsbController();
		~RsbController();
		void initParams();
		void getRPMs(Eigen::VectorXd* rpms);
		void getAttThrust(Eigen::VectorXd* att_thrust);
		void setTraj(const mav_msgs::EigenTrajectoryPoint& com_traj);
		// void setOdometryFromPose(const geometric_msgs::Pose& pose);
		void setOdometry(const EigenOdometry& odom);

		Eigen::Vector3d kbp_;
		Eigen::Vector3d K1p_;
		Eigen::Vector3d K2p_;
		Eigen::Vector3d E_hat_p_;
		Eigen::Vector3d c1_p_;
		Eigen::Vector3d c2_p_;
		Eigen::Vector3d eeta_p_;
		Eigen::Vector3d sigma_p_;

		Eigen::Vector3d kbq_;
		Eigen::Vector3d K1q_;
		Eigen::Vector3d K2q_;
		Eigen::Vector3d E_hat_q_;
		Eigen::Vector3d c1_q_;
		Eigen::Vector3d c2_q_;
		Eigen::Vector3d eeta_q_;
		Eigen::Vector3d sigma_q_;

		Eigen::Vector4d mass_inertia_;
		Eigen::Matrix4Xd allocation_matrix_;
		RotorConfiguration rotor_config_;
		double gravity_;
		double max_thrust_;

		bool controller_active_;


	private:
		bool initialized_params_;

		//double test_mass; //Added by Viswa

		Eigen::Vector3d J_inv_;
		Eigen::MatrixX4d rotor_vel_coef_;
		mav_msgs::EigenTrajectoryPoint com_traj_;
		EigenOdometry odometry_;


		void ComputeDesiredMoments(const Eigen::Vector3d& forces,
		                            Eigen::Vector3d* angular_acceleration);
		void ComputeDesiredForces(Eigen::Vector3d* forces) ;

	};




}

#endif