#include "drone_controller/asm_controller.h"


namespace drone_controller{
	AsmController::AsmController()
		: initialized_params_(false),
		  controller_active_(false)	{
	  	initParams();
	}

	AsmController::~AsmController() {}

	void AsmController::initParams(){
		calculateAllocationMatrix(rotor_config_, &(allocation_matrix_));


		rotor_vel_coef_.resize(rotor_config_.rotors.size(), 4);
		// Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
		// A^{ \dagger} = A^T*(A*A^T)^{-1}
		rotor_vel_coef_ = allocation_matrix_.transpose()
							* (allocation_matrix_ * allocation_matrix_.transpose()).inverse();

		gravity_ = 9.81;
		initialized_params_ = true;
	}

	void AsmController::getRPMs(Eigen::VectorXd* rpms){
		assert(rpms);
		assert(initialized_params_);

		rpms->resize(rotor_config_.rotors.size());

		if(!controller_active_){
			*rpms = Eigen::VectorXd::Zero(rpms->rows());
			return;
		}


		Eigen::Vector3d forces;
		ComputeDesiredForces(&forces);

		Eigen::Vector3d moments;
		ComputeDesiredMoments(forces, &moments);

		// ROS_INFO_STREAM("Forces: " << forces);
		// Project thrust onto body z axis.
		double thrust = forces.dot(odometry_.orientation.toRotationMatrix().col(2));
		// ROS_INFO_STREAM("Thrust: " << thrust);

		rpmConversion(rpms, &rotor_vel_coef_, moments, thrust);
	}

	void AsmController::ComputeDesiredForces(Eigen::Vector3d* forces) {
		assert(forces);

		Eigen::Vector3d position_error;
		position_error = odometry_.position - com_traj_.position_W;

		// Transform velocity to world frame.
		const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
		Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;
		Eigen::Vector3d velocity_error;
		velocity_error = velocity_W - com_traj_.velocity_W;

		Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());
		static ros::Time last_time = ros::Time::now();
		ros::Time current_time;

		Eigen::Vector3d sp;
		sp = velocity_error + theta_p_.cwiseProduct(position_error);

		// static Eigen::Vector3d hatKp = controller_parameters_.hatKp_;
		// static double hatM = controller_parameters_.hatM_;

		Eigen::Vector3d dot_hatKp0, dot_hatKp1;
		double dot_hatM;
		Eigen::Vector3d xi_p = position_error.cwiseAbs();

		dot_hatKp0 = sp.cwiseAbs() - alpha_p0_.cwiseProduct(hatKp0_);
		dot_hatKp1 = sp.cwiseAbs().cwiseProduct(xi_p) - alpha_p1_.cwiseProduct(hatKp1_);
		// dot_hatM = vehicle_parameters_.gravity_*abs(sp[2]) - controller_parameters_.alpha_m_*(hatM);
		dot_hatM = -gravity_*sp[2] - alpha_m_*(hatM_);


		current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();

		hatKp0_ += dot_hatKp0*dt;
		hatKp1_ += dot_hatKp1*dt;
		hatM_ += dot_hatM*dt;

		hatM_ = std::max(0.0, hatM_);
		hatKp0_.cwiseMax(Eigen::Vector3d::Zero());
		// hatKp1_.cwiseMax(Eigen::Vector3d::Zero());
		hatKp1_ << 0,0,0;

		last_time = current_time;

		// Eigen::Matrix3d lam_p = controller_parameters_.lam_p_.asDiagonal();
		Eigen::Vector3d rho_p = hatKp0_ + hatKp1_.cwiseProduct(xi_p);

		Eigen::Vector3d delTau_p;
		delTau_p << rho_p[0]*signumFn(sp[0], var_pi_p_), 
					  rho_p[1]*signumFn(sp[1], var_pi_p_), 
					  rho_p[2]*signumFn(sp[2], var_pi_p_);

		*forces =  -lam_p_.cwiseProduct(sp) - delTau_p + hatM_*gravity_*e_3;
	}

	void AsmController::ComputeDesiredMoments(const Eigen::Vector3d& forces,
	                                                     Eigen::Vector3d* moments) {
		assert(moments);

		Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();
		Eigen::Matrix3d R_des;
		desAttFromForces(forces, com_traj_.getYaw(), &R_des);


		// Angle error according to asm et al.
		Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
		Eigen::Vector3d angle_error;
		vectorFromSkewMatrix(angle_error_matrix, &angle_error);

		// TODO(burrimi) include angular rate references at some point.
		Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
		angular_rate_des[2] = com_traj_.getYawRate();

		Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;
		static ros::Time last_time = ros::Time::now();
		ros::Time current_time;

		Eigen::Vector3d sq;
		sq = angular_rate_error + theta_q_.cwiseProduct(angle_error);

		Eigen::Vector3d dot_hatKq0, dot_hatKq1, dot_hatKq2;
		Eigen::Vector3d xq_norm;
		xq_norm = angle_error.cwiseAbs();

		Eigen::Vector3d sq_norm = sq.cwiseAbs();

		Eigen::Vector3d xqx_norm, xqy_norm, xqz_norm;
		xqx_norm << 1, xq_norm[0], pow(xq_norm[0],2);
		xqy_norm << 1, xq_norm[1], pow(xq_norm[1],2);
		xqz_norm << 1, xq_norm[2], pow(xq_norm[2],2);;

		dot_hatKq0 = sq_norm[0]*xqx_norm - alpha_q0_.cwiseProduct(hatKq0_);
		dot_hatKq1 = sq_norm[1]*xqy_norm - alpha_q1_.cwiseProduct(hatKq1_);
		dot_hatKq2 = sq_norm[2]*xqz_norm - alpha_q2_.cwiseProduct(hatKq2_);


		current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();

		hatKq0_ += dot_hatKq0*dt;
		hatKq1_ += dot_hatKq1*dt;
		hatKq2_ += dot_hatKq2*dt;

		hatKq0_.cwiseMax(Eigen::Vector3d::Zero());
		hatKq1_.cwiseMax(Eigen::Vector3d::Zero());
		hatKq2_.cwiseMax(Eigen::Vector3d::Zero());


		last_time = current_time;

		Eigen::Vector3d rho_q;
		rho_q[0] = hatKq0_.dot(xqx_norm);
		rho_q[1] = hatKq1_.dot(xqy_norm);
		rho_q[2] = hatKq2_.dot(xqz_norm);


		Eigen::Vector3d delTau_q;

		delTau_q << rho_q[0]*signumFn(sq[0], var_pi_q_), 
				  	rho_q[1]*signumFn(sq[1], var_pi_q_), 
				  	rho_q[2]*signumFn(sq[2], var_pi_q_);

		*moments = -lam_q_.cwiseProduct(sq) - delTau_q;
	}

	void AsmController::getAttThrust(Eigen::VectorXd* att_thrust){
		assert(att_thrust);

		Eigen::Vector3d forces;
		ComputeDesiredForces(&forces);
		double thrust = forces.dot(odometry_.orientation.toRotationMatrix().col(2));
		double throttle = std::min(std::max(0.0, thrust/max_thrust_), 1.0);

		Eigen::Matrix3d R_des;
		desAttFromForces(forces, com_traj_.getYaw(), &R_des);

		Eigen::Quaterniond des_quat(R_des);
		(*att_thrust)[0] = throttle;
		(*att_thrust)[1] = des_quat.x();
		(*att_thrust)[2] = des_quat.y();
		(*att_thrust)[3] = des_quat.z();
		(*att_thrust)[4] = des_quat.w();
	}

	void AsmController::setOdometry(const EigenOdometry& odometry){
		odometry_ = odometry;
	}

	void AsmController::setTraj(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
		com_traj_ = command_trajectory;
		controller_active_ = true;
	}


}