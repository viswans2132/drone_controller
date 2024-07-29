#ifndef DRONE_CONTROLLER_MULTI_ECBF_CONTROLLER_H_
#define DRONE_CONTROLLER_MULTI_ECBF_CONTROLLER_H_

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
// #include <geometric_msgs/Pose.h>

#include "drone_controller/common_operations.h"

namespace drone_controller{
	class MultiEcbfController{
	public:
		MultiEcbfController();
		~MultiEcbfController();
		void initParams();
		// void setLanding_01();
		// void setLanding_02();
		void getRPMs_01(Eigen::VectorXd* rpms) const;
		void getRPMs_02(Eigen::VectorXd* rpms) const;
		void getAttThrust(Eigen::VectorXd* att_thrust) const;
		void setTraj_01(const mav_msgs::EigenTrajectoryPoint& com_traj);
		void setTraj_02(const mav_msgs::EigenTrajectoryPoint& com_traj);
		// void setOdometryFromPose(const geometric_msgs::Pose& pose);
		void setOdometry_01(const EigenOdometry& odom);
		void setOdometry_02(const EigenOdometry& odom);
		void setComOdom_01(const EigenOdometry& odom);
		void setComOdom_02(const EigenOdometry& odom);

		Eigen::Vector3d Kp_p_;
		Eigen::Vector3d Kv_p_;
		Eigen::Vector3d Ki_p_;
		Eigen::Vector3d Kp_q_;
		Eigen::Vector3d Kv_q_;
		Eigen::Vector4d mass_inertia_;
		Eigen::Matrix4Xd allocation_matrix_;
		RotorConfiguration rotor_config_;
		double gravity_;
		double max_thrust_;
		double max_velocity_;
		double Kh_sph_;
		double Kh_ell_;
		double Kh_ae_;
		double Kh_be_;
		double Kh_scale_;
		double Kh_offset_;
		double Khl_b_;
		double Khs_b_;
		double Khe_b_;

		bool controller_active_;
		// bool landing_01;
		// bool landing_02;

	private:
		bool initialized_params_;

		Eigen::Vector3d normalized_attitude_gain_;
		Eigen::Vector3d normalized_angular_rate_gain_;
		Eigen::MatrixX4d ang_acc_rpms_;

		//double test_mass; //Added by Viswa

		mav_msgs::EigenTrajectoryPoint com_traj_01, com_traj_02;
		geometry_msgs::PoseStamped com_pose_01, com_pose_02;
		EigenOdometry odometry_01, odometry_02;
		EigenOdometry com_odom_01, com_odom_02;


		void ComputeDesiredAngularAcc_01(const Eigen::Vector3d& forces,
		                            Eigen::Vector3d* angular_acceleration) const;
		void ComputeDesiredAngularAcc_02(const Eigen::Vector3d& forces,
		                            Eigen::Vector3d* angular_acceleration) const;
		void ComputeDesiredForces(Eigen::VectorXd* forces) const;
	};




}

#endif