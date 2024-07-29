#include "drone_controller/aec_controller.h"


namespace drone_controller{
	AecController::AecController()
		: initialized_params_(false),
		  controller_active_(false)	{
	  	initParams();
	}

	AecController::~AecController() {}

	void AecController::initParams(){
		calculateAllocationMatrix(rotor_config_, &(allocation_matrix_));

		Eigen::Matrix3d inertia = mass_inertia_.tail(3).asDiagonal();

		// To make the tuning independent of the inertia matrix we divide here.
		normalized_attitude_gain_ = Kp_q_.transpose()
		  * inertia.inverse();
		// To make the tuning independent of the inertia matrix we divide here.
		normalized_angular_rate_gain_ = Kv_q_.transpose()
		  * inertia.inverse();


		Eigen::Matrix4d I;
		I.setIdentity();
		I.block<3, 3>(0, 0) = inertia;
		I(3, 3) = 1;

		ang_acc_rpms_.resize(rotor_config_.rotors.size(), 4);
		// Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
		// A^{ \dagger} = A^T*(A*A^T)^{-1}
		ang_acc_rpms_ = allocation_matrix_.transpose()
		  * (allocation_matrix_* allocation_matrix_.transpose()).inverse() * I;

		gravity_ = 9.81;
		initialized_params_ = true;
	}

	void AecController::getRPMs(Eigen::VectorXd* rpms){
		assert(rpms);
		assert(initialized_params_);

		rpms->resize(rotor_config_.rotors.size());

		if(!controller_active_){
			*rpms = Eigen::VectorXd::Zero(rpms->rows());
			return;
		}


		Eigen::Vector3d forces;
		ComputeDesiredForces(&forces);

		Eigen::Vector3d angular_acceleration;
		ComputeDesiredAngularAcc(forces, &angular_acceleration);

		// Project thrust onto body z axis.
		double thrust = forces.dot(odometry_.orientation.toRotationMatrix().col(2));

		rpmConversion(rpms, &ang_acc_rpms_, angular_acceleration, thrust);
	}

	void AecController::ComputeDesiredForces(Eigen::Vector3d* forces) {
		assert(forces);

		Eigen::Vector3d position_error;
		position_error = odometry_.position - com_traj_.position_W;

		// Transform velocity to world frame.
		const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
		Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;
		Eigen::Vector3d velocity_error;
		velocity_error = velocity_W - com_traj_.velocity_W;

		Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());
		static ros::Time start_time = ros::Time::now();
		static ros::Time last_time = ros::Time::now();
		ros::Time current_time;


		Eigen::Vector3d sp = velocity_error + theta_p_.cwiseProduct(position_error);

		current_time = ros::Time::now();
		double time_from_start = (current_time - start_time).toSec();
		double dt = (current_time - last_time).toSec();
		last_time = current_time;

		Eigen::Vector3d kap = (rho_0a_p_ - rho_ssa_p_)*std::exp(-alpha_a_p_*time_from_start)
								+ rho_ssa_p_;
		Eigen::Vector3d kbp = (rho_0b_p_ - rho_ssb_p_)*std::exp(-alpha_b_p_*time_from_start)
								+ rho_ssb_p_;

		Eigen::Vector3d dot_kap = -alpha_a_p_*(rho_0a_p_ - rho_ssa_p_)*std::exp(-alpha_a_p_*time_from_start);
		Eigen::Vector3d dot_kbp = -alpha_b_p_*(rho_0b_p_ - rho_ssb_p_)*std::exp(-alpha_b_p_*time_from_start);

		Eigen::Vector3d position_error_sq = position_error.array().square();
		Eigen::Vector3d velocity_error_sq = velocity_error.array().square();
		Eigen::Vector3d kap_sq = kap.array().square();
		Eigen::Vector3d kbp_sq = kbp.array().square();

		Eigen::Vector3d zap = (kap_sq - position_error_sq).array().inverse();
		Eigen::Vector3d zbp = (kbp_sq - velocity_error_sq).array().inverse();
		Eigen::Vector3d zap_sq = zap.array().square();
		Eigen::Vector3d zbp_sq = zbp.array().square();

		Eigen::Vector3d xi_p_sq = position_error_sq + velocity_error_sq;
		Eigen::Vector3d xi_p = xi_p_sq.cwiseSqrt();
		Eigen::Vector3d xi_p_cu = xi_p_sq.cwiseProduct(xi_p);

		Eigen::Vector3d sp_abs = sp.cwiseAbs();

		hatK1a_p_ += (sp_abs.cwiseProduct(zap.cwiseProduct(xi_p)) - alphaK1a_p_.cwiseProduct(hatK1a_p_))*dt;
		hatK1a_p_ += (sp_abs.cwiseProduct(zap_sq.cwiseProduct(xi_p)) - alphaK2a_p_.cwiseProduct(hatK2a_p_))*dt;
		hatK1a_p_ += (sp_abs.cwiseProduct(zap_sq.cwiseProduct(xi_p_cu)) - alphaK3a_p_.cwiseProduct(hatK3a_p_))*dt;
		hatK1a_p_ += (sp_abs.cwiseProduct(zbp) - alphaK1b_p_.cwiseProduct(hatK1b_p_))*dt;
		hatK1a_p_ += (sp_abs.cwiseProduct(zbp_sq.cwiseProduct(xi_p)) - alphaK2b_p_.cwiseProduct(hatK2b_p_))*dt;
		hatK1a_p_ += (sp_abs.cwiseProduct(zbp_sq.cwiseProduct(xi_p_sq)) - alphaK3b_p_.cwiseProduct(hatK3b_p_))*dt;
		hatK1a_p_ += (sp_abs.cwiseProduct(zbp_sq.cwiseProduct(xi_p_cu)) - alphaK4b_p_.cwiseProduct(hatK4b_p_))*dt;
  
		Eigen::Vector3d zeta_p(0,0,0);
		zeta_p << hatK1a_p_.cwiseProduct(zap.cwiseProduct(xi_p)) 
					+ hatK2a_p_.cwiseProduct(zap_sq.cwiseProduct(xi_p))
					+ hatK3a_p_.cwiseProduct(zap_sq.cwiseProduct(xi_p_cu))
					+ hatK1b_p_.cwiseProduct(zbp)
					+ hatK2b_p_.cwiseProduct(zbp_sq.cwiseProduct(xi_p))
					+ hatK3b_p_.cwiseProduct(zbp_sq.cwiseProduct(xi_p_sq))
					+ hatK4b_p_.cwiseProduct(zbp_sq.cwiseProduct(xi_p_cu));

		Eigen::Vector3d delTau_p(0,0,0);
		delTau_p << zeta_p[0]*signumFn(sp[0], var_pi_p_), 
		          	zeta_p[1]*signumFn(sp[1], var_pi_p_),
		            zeta_p[2]*signumFn(sp[2], var_pi_p_); 
		
		*forces = -lam_p_.cwiseProduct(sp) - delTau_p;

		// *forces << 0, 0, 0;

		// last_time = current_time;
	}

	void AecController::ComputeDesiredAngularAcc(const Eigen::Vector3d& forces,
	                                                     Eigen::Vector3d* angular_acceleration) {
		assert(angular_acceleration);

		Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();
		Eigen::Matrix3d R_des;
		desAttFromForces(forces, com_traj_.getYaw(), &R_des);


		// Angle error according to pid et al.
		Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
		Eigen::Vector3d angle_error;
		vectorFromSkewMatrix(angle_error_matrix, &angle_error);

		// TODO(burrimi) include angular rate references at some point.
		Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
		angular_rate_des[2] = com_traj_.getYawRate();

		Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;



		*angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
		                       - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
	                       + odometry_.angular_velocity.cross(odometry_.angular_velocity);
	}

	void AecController::getAttThrust(Eigen::VectorXd* att_thrust){
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

	void AecController::setOdometry(const EigenOdometry& odometry){
		odometry_ = odometry;
	}

	void AecController::setTraj(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
		com_traj_ = command_trajectory;
		controller_active_ = true;
	}


}