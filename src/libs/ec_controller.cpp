#include "drone_controller/ec_controller.h"


namespace drone_controller{
	EcController::EcController()
		: initialized_params_(false),
		  controller_active_(false)	{
	  	initParams();
	}

	EcController::~EcController() {}

	void EcController::initParams(){
		calculateAllocationMatrix(rotor_config_, &(allocation_matrix_));

		thrust_moments_rpms_.resize(rotor_config_.rotors.size(), 4);
		// Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
		// A^{ \dagger} = A^T*(A*A^T)^{-1}
		thrust_moments_rpms_ = allocation_matrix_.transpose()
		  * (allocation_matrix_* allocation_matrix_.transpose()).inverse();

		gravity_ = 9.81;
		initialized_params_ = true;
	}

	void EcController::getRPMs(Eigen::VectorXd* rpms){
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

		rpmConversion(rpms, &thrust_moments_rpms_, moments, thrust);
	}

	void EcController::ComputeDesiredForces(Eigen::Vector3d* forces) {
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
		ros::Time current_time;

		Eigen::Vector3d sp = velocity_error + lam_p_.cwiseProduct(position_error);

		current_time = ros::Time::now();
		double time_from_start = (current_time - start_time).toSec();

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

		// Eigen::Vector3d kap_ep_sq_inv = (kap_sq + position_error_sq).array().inverse();
		// Eigen::Vector3d kbp_ev_sq_inv = (kbp_sq + velocity_error_sq).array().inverse();

		Eigen::Vector3d kap_ep_sq2_inv = (kap_sq - position_error_sq).array().square().inverse();
		Eigen::Vector3d kbp_ev_sq2_inv = (kbp_sq - velocity_error_sq).array().square().inverse();


		

		Eigen::Vector3d term_1_p(0,0,0);
		term_1_p = (kbp_sq + velocity_error_sq).cwiseProduct(kbp_ev_sq2_inv).array().inverse();

		Eigen::Vector3d term_2_p(0,0,0);
		term_2_p = lam_p_.cwiseProduct(kbp_sq + position_error_sq).cwiseProduct(
						kap_ep_sq2_inv) - 2*kbp.cwiseProduct(
						dot_kbp).cwiseProduct(kbp_ev_sq2_inv);

		Eigen::Vector3d term_3_p(0,0,0);
		term_3_p = - 2*lam_p_.cwiseProduct(kap.cwiseProduct(dot_kap)).cwiseProduct(
					kap_ep_sq2_inv);

  
		Eigen::Vector3d zeta_p(0,0,0);
		zeta_p << eHat_p_.cwiseProduct(gravity_ * e_3 + com_traj_.acceleration_W - 
					term_1_p.cwiseProduct(term_2_p).cwiseProduct(
					velocity_error) - term_1_p.cwiseProduct(
					term_3_p).cwiseProduct(position_error)) + eeta_p_;
		        

		Eigen::Vector3d delTau_p(0,0,0);
		delTau_p << zeta_p[0]*signumFn(sp[0], var_pi_p_), 
		          	zeta_p[1]*signumFn(sp[1], var_pi_p_),
		            zeta_p[2]*signumFn(sp[2], var_pi_p_); 

		*forces = mass_inertia_[0]*(gravity_ * e_3 + com_traj_.acceleration_W - 
					term_1_p.cwiseProduct(term_2_p).cwiseProduct(
					velocity_error) - term_1_p.cwiseProduct(
					term_3_p).cwiseProduct(position_error)) + delTau_p;

		// *forces << 0, 0, 0;

		// last_time = current_time;
	}

	void EcController::ComputeDesiredMoments(const Eigen::Vector3d& forces,
	                                                     Eigen::Vector3d* moments) {
		assert(moments);

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
		static ros::Time start_time = ros::Time::now();
		ros::Time current_time;

		Eigen::Vector3d sq = angular_rate_error + lam_q_.cwiseProduct(angle_error);

		current_time = ros::Time::now();
		double time_from_start = (current_time - start_time).toSec();

		Eigen::Vector3d kaq = (rho_0a_q_ - rho_ssa_q_)*std::exp(-alpha_a_q_*time_from_start)
								+ rho_ssa_q_;
		Eigen::Vector3d kbq = (rho_0b_q_ - rho_ssb_q_)*std::exp(-alpha_b_q_*time_from_start)
								+ rho_ssb_q_;

		Eigen::Vector3d dot_kaq = -alpha_a_q_*(rho_0a_q_ - rho_ssa_q_)*std::exp(-alpha_a_q_*time_from_start);
		Eigen::Vector3d dot_kbq = -alpha_b_q_*(rho_0b_q_ - rho_ssb_q_)*std::exp(-alpha_b_q_*time_from_start);

		Eigen::Vector3d angle_error_sq = angle_error.array().square();
		Eigen::Vector3d angular_rate_error_sq = angular_rate_error.array().square();
		Eigen::Vector3d kaq_sq = kaq.array().square();
		Eigen::Vector3d kbq_sq = kbq.array().square();

		// Eigen::Vector3d kap_ep_sq_inv = (kap_sq + position_error_sq).array().inverse();
		// Eigen::Vector3d kbp_ev_sq_inv = (kbp_sq + velocity_error_sq).array().inverse();

		Eigen::Vector3d kaq_ep_sq2_inv = (kaq_sq - angle_error_sq).array().square().inverse();
		Eigen::Vector3d kbq_ev_sq2_inv = (kbq_sq - angular_rate_error_sq).array().square().inverse();


		

		Eigen::Vector3d term_1_q(0,0,0);
		term_1_q = (kbq_sq + angular_rate_error_sq).cwiseProduct(kbq_ev_sq2_inv).array().inverse();

		Eigen::Vector3d term_2_q(0,0,0);
		term_2_q = lam_q_.cwiseProduct(kbq_sq + angle_error_sq).cwiseProduct(
						kaq_ep_sq2_inv) - 2*kbq.cwiseProduct(
						dot_kbq).cwiseProduct(kbq_ev_sq2_inv);

		Eigen::Vector3d term_3_q(0,0,0);
		term_3_q = - 2*lam_q_.cwiseProduct(kaq.cwiseProduct(dot_kaq)).cwiseProduct(
					kaq_ep_sq2_inv);

  
		Eigen::Vector3d zeta_q(0,0,0);
		zeta_q << eHat_q_.cwiseProduct(- term_1_q.cwiseProduct(term_2_q).cwiseProduct(
					angular_rate_error) - term_1_q.cwiseProduct(
					term_3_q).cwiseProduct(angle_error)) + cHat_q_ + eeta_q_;
		        

		Eigen::Vector3d delTau_q(0,0,0);
		delTau_q << zeta_q[0]*signumFn(sq[0], var_pi_q_), 
		          	zeta_q[1]*signumFn(sq[1], var_pi_q_),
		            zeta_q[2]*signumFn(sq[2], var_pi_q_); 

		*moments = mass_inertia_.tail(3).cwiseProduct(- term_1_q.cwiseProduct(term_2_q).cwiseProduct(
					angular_rate_error) - term_1_q.cwiseProduct(
					term_3_q).cwiseProduct(angle_error) + odometry_.angular_velocity.cross(odometry_.angular_velocity))
					+ delTau_q;
	}

	void EcController::getAttThrust(Eigen::VectorXd* att_thrust){
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

	void EcController::setOdometry(const EigenOdometry& odometry){
		odometry_ = odometry;
	}

	void EcController::setTraj(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
		com_traj_ = command_trajectory;
		controller_active_ = true;
	}


}