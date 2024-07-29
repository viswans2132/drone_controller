#include "drone_controller/rsb_controller.h"


namespace drone_controller{
	RsbController::RsbController()
		: initialized_params_(false),
		  controller_active_(false)	{
	  	initParams();
	}

	RsbController::~RsbController() {}

	void RsbController::initParams(){
		calculateAllocationMatrix(rotor_config_, &(allocation_matrix_));


		rotor_vel_coef_.resize(rotor_config_.rotors.size(), 4);
		// Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
		// A^{ \dagger} = A^T*(A*A^T)^{-1}
		rotor_vel_coef_ = allocation_matrix_.transpose()
							* (allocation_matrix_ * allocation_matrix_.transpose()).inverse();

		Eigen::Matrix3d inertia = mass_inertia_.tail(3).asDiagonal();
		J_inv_ = inertia.inverse().diagonal();
		gravity_ = 9.81;
		initialized_params_ = true;
	}

	void RsbController::getRPMs(Eigen::VectorXd* rpms){
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

		// Project thrust onto body z axis.
		double thrust = forces.dot(odometry_.orientation.toRotationMatrix().col(2));

		rpmConversion(rpms, &rotor_vel_coef_, moments, thrust);
	}

	void RsbController::ComputeDesiredForces(Eigen::Vector3d* forces) {
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

  
		Eigen::Vector3d sqr_term_p = kbp_.cwiseProduct(kbp_) - position_error.cwiseProduct(position_error);
		// static Eigen::Vector3d previous_alpha(0,0,0);

		Eigen::Vector3d alpha_p = -sqr_term_p.cwiseProduct(K1p_.cwiseProduct(position_error)) + com_traj_.velocity_W;

		Eigen::Vector3d z2_p = velocity_W - alpha_p;
		Eigen::Vector3d dot_alpha_p(0,0,0);

		dot_alpha_p = (3*position_error.cwiseProduct(position_error) 
		            - kbp_.cwiseProduct(kbp_)).cwiseProduct(
		              K1p_.cwiseProduct(velocity_error)) + com_traj_.acceleration_W;

		Eigen::Vector3d term_3_p(0,0,0);
		term_3_p << position_error[0]/sqr_term_p[0], position_error[1]/sqr_term_p[1], position_error[2]/sqr_term_p[2];

		Eigen::Vector3d sqrt_term_p = (z2_p.cwiseProduct(z2_p) + eeta_p_).cwiseSqrt();

		Eigen::Vector3d u0_p = dot_alpha_p 
		                    - K2p_.cwiseProduct(z2_p) - term_3_p;

		Eigen::Vector3d rho_p_ = E_hat_p_.cwiseProduct(u0_p.cwiseAbs())  
		                      + (c1_p_*gravity_)/mass_inertia_[0];

		Eigen::Vector3d rho_p(0,0,0);
		rho_p << rho_p_[0]/(1 - E_hat_p_[0]*sigma_p_[0]),
		        rho_p_[1]/(1 - E_hat_p_[1]*sigma_p_[1]),
		          rho_p_[2]/(1 - E_hat_p_[2]*sigma_p_[2]);

		Eigen::Vector3d del_u_p(0,0,0);
		del_u_p << -rho_p[0]*sigma_p_[0]*(z2_p[0]/sqrt_term_p[0]), 
		          -rho_p[1]*sigma_p_[1]*(z2_p[1]/sqrt_term_p[1]),
		            -rho_p[2]*sigma_p_[2]*(z2_p[2]/sqrt_term_p[2]); 

		*forces = mass_inertia_[0]*(gravity_ * e_3 + u0_p + del_u_p);

		last_time = current_time;
	}

	void RsbController::ComputeDesiredMoments(const Eigen::Vector3d& forces,
	                                                     Eigen::Vector3d* moments) {
		assert(moments);

		Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();
		Eigen::Matrix3d R_des;
		desAttFromForces(forces, com_traj_.getYaw(), &R_des);


		Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
		Eigen::Vector3d angle_error;
		vectorFromSkewMatrix(angle_error_matrix, &angle_error);

		Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
		angular_rate_des[2] = com_traj_.getYawRate();
		Eigen:: Vector3d angular_rate_des_B = R_des.transpose() * R * angular_rate_des;

		Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - angular_rate_des_B;
		Eigen::Vector3d xi_q = (angle_error.cwiseProduct(angle_error) + angular_rate_error.cwiseProduct(angular_rate_error)).cwiseSqrt();

		Eigen::Vector3d sqr_term_q = kbq_.cwiseProduct(kbq_)
		                    - angle_error.cwiseProduct(angle_error);

		Eigen::Vector3d alpha_q = -sqr_term_q.cwiseProduct(K1q_.cwiseProduct(angle_error)) 
		                          + angular_rate_des_B;

		Eigen::Vector3d dot_alpha_q(0,0,0);
		dot_alpha_q = (3*angle_error.cwiseProduct(angle_error) 
		              - kbq_.cwiseProduct(kbq_)).cwiseProduct(
		                K1q_.cwiseProduct(angular_rate_error));

		Eigen::Vector3d z2_q = odometry_.angular_velocity - alpha_q;

		Eigen::Vector3d term_3_q(0,0,0);
		term_3_q << angle_error[0]/sqr_term_q[0], angle_error[1]/sqr_term_q[1], angle_error[2]/sqr_term_q[2];

		Eigen::Vector3d sqrt_term_q = (z2_q.cwiseProduct(z2_q) + eeta_q_).cwiseSqrt();

		Eigen::Vector3d u0_q = dot_alpha_q - K2q_.cwiseProduct(z2_q) - term_3_q;

		Eigen::Vector3d rho_q_ = E_hat_q_.cwiseProduct(u0_q.cwiseAbs()) 
		                         + J_inv_.cwiseProduct(c1_q_).cwiseProduct(angular_rate_des_B);

		Eigen::Vector3d rho_q(0,0,0);
		rho_q << rho_q_[0]/(1 - E_hat_q_[0]*sigma_q_[0]),
		          rho_q_[1]/(1 - E_hat_q_[1]*sigma_q_[1]),
		            rho_q_[2]/(1 - E_hat_q_[2]*sigma_q_[2]);



		Eigen::Vector3d del_u_q(0,0,0);
		del_u_q << -rho_q[0]*sigma_q_[0]*(z2_q[0]/sqrt_term_q[0]), 
		            -rho_q[1]*sigma_q_[1]*(z2_q[1]/sqrt_term_q[1]),
		              -rho_q[2]*sigma_q_[2]*(z2_q[2]/sqrt_term_q[2]); 

		*moments = mass_inertia_.tail(3).cwiseProduct(u0_q + del_u_q);
	}

	void RsbController::getAttThrust(Eigen::VectorXd* att_thrust){
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

	void RsbController::setOdometry(const EigenOdometry& odometry){
		odometry_ = odometry;
	}

	void RsbController::setTraj(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
		com_traj_ = command_trajectory;
		controller_active_ = true;
	}


}