#include "drone_controller/multi_ecbf_controller.h"
#include <OsqpEigen/OsqpEigen.h>

namespace drone_controller{
	MultiEcbfController::MultiEcbfController()
		: initialized_params_(false),
		  controller_active_(false)	{
	  	initParams();
	}

	MultiEcbfController::~MultiEcbfController() {}

	void MultiEcbfController::initParams(){
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

	void MultiEcbfController::getRPMs_01(Eigen::VectorXd* rpms) const{
		assert(rpms);
		assert(initialized_params_);

		rpms->resize(rotor_config_.rotors.size());

		if(!controller_active_){
			*rpms = Eigen::VectorXd::Zero(rpms->rows());
			return;
		}


		Eigen::VectorXd forces(6);
		ComputeDesiredForces(&forces);

		Eigen::Vector3d forces_01, forces_02;
		forces_01 << forces[0], forces[1], forces[2];
		// forces_02 << forces[3], forces[4], forces[5];
		if(forces_01.norm() < 0.1){
			*rpms << 0.0, 0.0, 0.0, 0.0;
		}
		else{

			Eigen::Vector3d angular_acceleration_01, angular_acceleration_02;
			ComputeDesiredAngularAcc_01(forces_01, &angular_acceleration_01);
			// ComputeDesiredAngularAcc(forces_02, &angular_acceleration_02);

			// ROS_INFO_STREAM("Forces: " << forces);
			// Project thrust onto body z axis.
			double thrust_01 = forces_01.dot(odometry_01.orientation.toRotationMatrix().col(2));
			// double thrust_02 = forces_02.dot(odometry_02.orientation.toRotationMatrix().col(2));
			ROS_INFO_STREAM("Thrust: " << thrust_01);

			rpmConversion(rpms, &ang_acc_rpms_, angular_acceleration_01, thrust_01);
		}
	}

	void MultiEcbfController::getRPMs_02(Eigen::VectorXd* rpms) const{
		assert(rpms);
		assert(initialized_params_);

		rpms->resize(rotor_config_.rotors.size());

		if(!controller_active_){
			*rpms = Eigen::VectorXd::Zero(rpms->rows());
			return;
		}


		Eigen::VectorXd forces(6);
		ComputeDesiredForces(&forces);

		Eigen::Vector3d forces_01, forces_02;
		// forces_01 << forces[0], forces[1], forces[2];
		forces_02 << forces[3], forces[4], forces[5];
		if(forces_02.norm() < 0.1){
			*rpms << 0.0, 0.0, 0.0, 0.0;
		}
		else{


			Eigen::Vector3d angular_acceleration_01, angular_acceleration_02;
			// ComputeDesiredAngularAcc(forces_01, &angular_acceleration_01);
			ComputeDesiredAngularAcc_02(forces_02, &angular_acceleration_02);

			// ROS_INFO_STREAM("Forces: " << forces);
			// Project thrust onto body z axis.
			// double thrust_01 = forces_01.dot(odometry_01.orientation.toRotationMatrix().col(2));
			double thrust_02 = forces_02.dot(odometry_02.orientation.toRotationMatrix().col(2));
			// ROS_INFO_STREAM("Thrust: " << thrust_01);

			rpmConversion(rpms, &ang_acc_rpms_, angular_acceleration_02, thrust_02);
		}
	}

	void MultiEcbfController::ComputeDesiredForces(Eigen::VectorXd* forces) const{
		assert(forces);

		Eigen::Vector3d position_error_01, psuedo_velocity_ref_01, position_error_abs_01;
		Eigen::Vector3d position_error_02, psuedo_velocity_ref_02, position_error_abs_02;
		position_error_01 = odometry_01.position - com_odom_01.position;
		position_error_02 = odometry_02.position - com_odom_02.position;

		Eigen::Vector3d ep_12, ep_21;
		ep_12 = odometry_01.position - com_odom_02.position;
		ep_21 = odometry_02.position - com_odom_01.position;

		// double ae = 1.0;
		// double be = 3.0;



		// static Eigen::Vector3d position_error_integral(0,0,0);
		// if(controller_active_){
		// 	static ros::Time last_time = ros::Time::now();
		// 	ros::Time current_time = ros::Time::now();  
		// 	position_error_integral += position_error * (current_time - last_time).toSec();
		// 	last_time = current_time;
		// }

		position_error_abs_01 = position_error_01.cwiseAbs();
		position_error_abs_02 = position_error_02.cwiseAbs();
		

		// Transform velocity to world frame.
		const Eigen::Matrix3d R_W_I_01 = odometry_01.orientation.toRotationMatrix();
		const Eigen::Matrix3d R_W_I_02 = odometry_02.orientation.toRotationMatrix();
		Eigen::Vector3d velocity_W_01 =  R_W_I_01 * odometry_01.velocity;
		Eigen::Vector3d velocity_W_02 =  R_W_I_02 * odometry_02.velocity;

		Eigen::Vector3d velocity_error_01, velocity_error_02;
		velocity_error_01 = velocity_W_01 - com_odom_01.velocity;
		velocity_error_02 = velocity_W_02 - com_odom_02.velocity;

		psuedo_velocity_ref_01 = -position_error_01.cwiseProduct(Kp_p_) - velocity_error_01.cwiseProduct(Kv_p_);
		psuedo_velocity_ref_02 = -position_error_02.cwiseProduct(Kp_p_) - velocity_error_02.cwiseProduct(Kv_p_);
		// position_error[2] = position_error[2] - 0.1;

		// Eigen::Vector3d position_error_sq = position_error.array().square();
		double x2y2_sqrt_01 = std::sqrt(position_error_01[0]*position_error_01[0] + position_error_01[1]*position_error_01[1]);
		double x2y2_sqrt_02 = std::sqrt(position_error_02[0]*position_error_02[0] + position_error_02[1]*position_error_02[1]);

		if(x2y2_sqrt_01<0.0001){
			x2y2_sqrt_01 = 0.0001;
		}
		if(x2y2_sqrt_02<0.0001){
			x2y2_sqrt_02 = 0.0001;
		}

		// ROS_INFO_STREAM("Sqrt(4): " << std::sqrt(4) << " Exp(-1): " << std::exp(-1));


		constexpr double tolerance = 1e-4;
		Eigen::SparseMatrix<c_float> H_s(6,6);
	    H_s.insert(0,0) = 2;
	    H_s.insert(1,1) = 2;
	    H_s.insert(2,2) = 2;
	    H_s.insert(3,3) = 2;
	    H_s.insert(4,4) = 2;
	    H_s.insert(5,5) = 2;

	    Eigen::SparseMatrix<c_float> A_s(5,6);
	    A_s.insert(0,0) = Kh_scale_*position_error_01[0]*std::exp(-x2y2_sqrt_01)*(x2y2_sqrt_01-1)/x2y2_sqrt_01;
	    A_s.insert(0,1) = Kh_scale_*position_error_01[1]*std::exp(-x2y2_sqrt_01)*(x2y2_sqrt_01-1)/x2y2_sqrt_01;
	    A_s.insert(0,2) = 1;
	    A_s.insert(1,3) = Kh_scale_*position_error_02[0]*std::exp(-x2y2_sqrt_02)*(x2y2_sqrt_02-1)/x2y2_sqrt_02;
	    A_s.insert(1,4) = Kh_scale_*position_error_02[1]*std::exp(-x2y2_sqrt_02)*(x2y2_sqrt_02-1)/x2y2_sqrt_02;
	    A_s.insert(1,5) = 1;
	    A_s.insert(2,0) = 2*(odometry_01.position[0] - odometry_02.position[0]);
	    A_s.insert(2,1) = 2*(odometry_01.position[1] - odometry_02.position[1]);
	    A_s.insert(2,2) = 2*(odometry_01.position[2] - odometry_02.position[2]);
	    A_s.insert(2,3) = 2*(odometry_02.position[0] - odometry_01.position[0]);
	    A_s.insert(2,4) = 2*(odometry_02.position[1] - odometry_01.position[1]);
	    A_s.insert(2,5) = 2*(odometry_02.position[2] - odometry_01.position[2]);
	    A_s.insert(3,0) = 2*ep_12[0]/Kh_ae_;
	    A_s.insert(3,1) = 2*ep_12[1]/Kh_ae_;
	    A_s.insert(3,2) = 2*ep_12[2]/Kh_be_;
	    A_s.insert(4,3) = 2*ep_21[0]/Kh_ae_;
	    A_s.insert(4,4) = 2*ep_21[1]/Kh_ae_;
	    A_s.insert(4,5) = 2*ep_21[2]/Kh_be_;

	    double hl_01 = position_error_01[2] - Kh_scale_*(x2y2_sqrt_01)*std::exp(-x2y2_sqrt_01) - Kh_offset_;
	    double hl_02 = position_error_02[2] - Kh_scale_*(x2y2_sqrt_02)*std::exp(-x2y2_sqrt_02) - Kh_offset_;
	    double hs_12 = (odometry_01.position - odometry_02.position).transpose()*(odometry_01.position - odometry_02.position) - 4*Kh_sph_*Kh_sph_;
	    double he_12 = (ep_12[0]*ep_12[0] + ep_12[1]*ep_12[1])/Kh_ae_ + (ep_12[2]*ep_12[2])/Kh_be_ - 2*Kh_ell_;
	    double he_21 = (ep_21[0]*ep_21[0] + ep_21[1]*ep_21[1])/Kh_ae_ + (ep_21[2]*ep_21[2])/Kh_be_ - 2*Kh_ell_;

	    Eigen::Matrix<c_float, 6, 1> gradient;
	    // gradient.insert(0,0) = -2*psuedo_velocity_ref_01[0];
	    // gradient.insert(1,0) = -2*psuedo_velocity_ref_01[1];
	    // gradient.insert(2,0) = -2*psuedo_velocity_ref_01[2];
	    // gradient.insert(3,1) = -2*psuedo_velocity_ref_02[0];
	    // gradient.insert(4,1) = -2*psuedo_velocity_ref_02[1];
	    // gradient.insert(5,1) = -2*psuedo_velocity_ref_02[2];



	    gradient << -2*psuedo_velocity_ref_01[0], -2*psuedo_velocity_ref_01[1], -2*psuedo_velocity_ref_01[2], 
	    			-2*psuedo_velocity_ref_02[0], -2*psuedo_velocity_ref_02[1], -2*psuedo_velocity_ref_02[2];

	    double dot_x2y2_sqrt_01 = (1-x2y2_sqrt_01)*std::exp(-x2y2_sqrt_01)*(velocity_W_01[0]*position_error_01[0] 
	    						+ velocity_W_01[1]*position_error_01[1])/x2y2_sqrt_01;
	    double dot_x2y2_sqrt_02 = (1-x2y2_sqrt_02)*std::exp(-x2y2_sqrt_02)*(velocity_W_02[0]*position_error_02[0] 
	    						+ velocity_W_02[1]*position_error_02[1])/x2y2_sqrt_02;
	    double dhs12_dt = 2*((odometry_01.position[0] - odometry_02.position[0])*(velocity_W_01[0] - velocity_W_02[0]) 
	    					+ (odometry_01.position[1] - odometry_02.position[1])*(velocity_W_01[1] - velocity_W_02[1]) 
	    					+ (odometry_01.position[2] - odometry_02.position[2])*(velocity_W_01[2] - velocity_W_02[2]));
	    double dhe12_dt = -2*(ep_12[0]*com_odom_02.velocity[0] + ep_12[1]*com_odom_02.velocity[1])/Kh_ae_ - 2*(ep_12[2]*com_odom_02.velocity[2])/Kh_be_;
	    double dhe21_dt = -2*(ep_21[0]*com_odom_01.velocity[0] + ep_21[1]*com_odom_01.velocity[1])/Kh_ae_ - 2*(ep_21[2]*com_odom_01.velocity[2])/Kh_be_;

	    Eigen::Matrix<c_float, 5, 1> lowerBound;
	    lowerBound << -Khl_b_*hl_01 + dot_x2y2_sqrt_01, -Khl_b_*hl_02 + dot_x2y2_sqrt_02, -Khs_b_*hs_12 - dhs12_dt, -Khe_b_*he_12 - dhe12_dt, -Khe_b_*he_21 - dhe21_dt;

	    Eigen::Matrix<c_float, 5, 1> upperBound;
	    upperBound << 50, 50, 50, 50, 50;

	    OsqpEigen::Solver solver;
	    solver.settings()->setVerbosity(true);
	    solver.settings()->setAlpha(1.0);

	    solver.data()->setNumberOfVariables(6);

	    solver.data()->setNumberOfConstraints(5);
	    solver.data()->setHessianMatrix(H_s);
	    solver.data()->setGradient(gradient);
	    solver.data()->setLinearConstraintsMatrix(A_s);
	    solver.data()->setLowerBound(lowerBound);
	    solver.data()->setUpperBound(upperBound);

	    solver.initSolver();
	    solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError;
	    
	    Eigen::Matrix<c_float, 6, 1> expectedSolution;
    	expectedSolution << psuedo_velocity_ref_01[0], psuedo_velocity_ref_01[1], psuedo_velocity_ref_01[2], 
    						psuedo_velocity_ref_02[0], psuedo_velocity_ref_02[1], psuedo_velocity_ref_02[2];

    	Eigen::VectorXd velocity_ref(6);
    	velocity_ref = solver.getSolution();
    	// velocity_ref[0] = 0 ;
    	// velocity_ref[5] = 0 ;

    	// psuedo_velocity_ref = 0.1*Eigen::Vector3d::Ones().cwiseMin(psuedo_velocity_ref.cwiseMax(-0.1*Eigen::Vector3d::Ones()));
    	if (velocity_ref.norm() > max_velocity_){
    		velocity_ref = velocity_ref*(max_velocity_/velocity_ref.norm());
    	}

		Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());

		// static Eigen::Vector3d position_error_integral(0,0,0);
		// if(controller_active_){
		// 	static ros::Time last_time = ros::Time::now();
		// 	ros::Time current_time = ros::Time::now();  
		// 	position_error_integral += position_error * (current_time - last_time).toSec();
		// 	last_time = current_time;
		// }

		Eigen::VectorXd appended_Kv_p_(6);
		appended_Kv_p_[0] = Kv_p_[0];
		appended_Kv_p_[1] = Kv_p_[1];
		appended_Kv_p_[2] = Kv_p_[2];
		appended_Kv_p_[3] = Kv_p_[0];
		appended_Kv_p_[4] = Kv_p_[1];
		appended_Kv_p_[5] = Kv_p_[2];

		Eigen::VectorXd appended_e_3_(6);
		appended_e_3_[0] = 0.0;
		appended_e_3_[1] = 0.0;
		appended_e_3_[2] = 1.0;
		appended_e_3_[3] = 0.0;
		appended_e_3_[4] = 0.0;
		appended_e_3_[5] = 1.0;

		Eigen::VectorXd appened_velocity_error_(6);
		appened_velocity_error_[0] = velocity_error_01[0];
		appened_velocity_error_[1] = velocity_error_01[1];
		appened_velocity_error_[2] = velocity_error_01[2];
		appened_velocity_error_[3] = velocity_error_02[0];
		appened_velocity_error_[4] = velocity_error_02[1];
		appened_velocity_error_[5] = velocity_error_02[2];

		*forces = - (-velocity_ref).cwiseProduct(appended_Kv_p_)
						+ (gravity_ * appended_e_3_) * mass_inertia_[0];

		static bool landing_01 = false;
		static bool landing_02 = false;


		if(landing_01 || ((position_error_abs_01[0] < 0.02) && (position_error_abs_01[1] < 0.02) && (position_error_abs_01[2] < 0.05))){
			forces[0][0] = 0.0;
			forces[0][1] = 0.0;
			forces[0][2] = 0.0;
			landing_01 = true;
		}
		if(landing_02 || ((position_error_abs_02[0] < 0.02) && (position_error_abs_02[1] < 0.02) && (position_error_abs_02[2] < 0.05))){
			forces[0][3] = 0.0;
			forces[0][4] = 0.0;
			forces[0][5] = 0.0;
			landing_02 = true;
		}
	

		// ROS_INFO_STREAM("forces: "<< *forces);
		// ROS_INFO_STREAM("Position Error: "<< position_error_abs_01);


	}

	void MultiEcbfController::ComputeDesiredAngularAcc_01(const Eigen::Vector3d& forces,
	                                                     Eigen::Vector3d* angular_acceleration) const {
		assert(angular_acceleration);

		Eigen::Matrix3d R = odometry_01.orientation.toRotationMatrix();
		Eigen::Matrix3d R_des;
		double yaw;
		// getYawFromOrientation(&yaw, com_odom_01.orientation);
		// ROS_INFO_STREAM("Yaw: " << yaw);
		desAttFromForces(forces, com_odom_01.orientation.z(), &R_des);


		// Angle error according to pid et al.
		Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
		Eigen::Vector3d angle_error;
		vectorFromSkewMatrix(angle_error_matrix, &angle_error);
		angle_error[2] = constrainAngle(angle_error[2]);
		// angle_error[2] = 0.0;

		// TODO(burrimi) include angular rate references at some point.
		Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
		// angular_rate_des[2] = com_odom_01.angular_velocity[2];

		Eigen::Vector3d angular_rate_error = odometry_01.angular_velocity - R_des.transpose() * R * angular_rate_des;



		*angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
		                       - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
	                       + odometry_01.angular_velocity.cross(odometry_01.angular_velocity); // we don't need the inertia matrix here
	  // data_out->angular_acceleration = *angular_acceleration;
	  // data_out->angle_error = angle_error;
	  // data_out->angle_rate_error = angular_rate_error;
	  // tf::vectorEigenToMsg(*angular_acceleration, data_out->angular_acceleration);
	  // tf::vectorEigenToMsg(angle_error, data_out->angle_error);
	  // tf::vectorEigenToMsg(angular_rate_error, data_out->angle_rate_error);
		// ROS_INFO_STREAM("moments: "<< *angular_acceleration);
	}

	void MultiEcbfController::ComputeDesiredAngularAcc_02(const Eigen::Vector3d& forces,
	                                                     Eigen::Vector3d* angular_acceleration) const {
		assert(angular_acceleration);

		Eigen::Matrix3d R = odometry_02.orientation.toRotationMatrix();
		Eigen::Matrix3d R_des;
		double yaw;
		// getYawFromOrientation(&yaw, com_odom_02.orientation);
		// ROS_INFO_STREAM("Yaw2: " << yaw);
		desAttFromForces(forces, com_odom_02.orientation.z(), &R_des);


		// Angle error according to pid et al.
		Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
		Eigen::Vector3d angle_error;
		vectorFromSkewMatrix(angle_error_matrix, &angle_error);
		// angle_error[2] = 0.0;

		// TODO(burrimi) include angular rate references at some point.
		Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
		// angular_rate_des[2] = com_odom_02.angular_velocity[2];

		Eigen::Vector3d angular_rate_error = odometry_02.angular_velocity - R_des.transpose() * R * angular_rate_des;



		*angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
		                       - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
	                       + odometry_02.angular_velocity.cross(odometry_02.angular_velocity); // we don't need the inertia matrix here
	  // data_out->angular_acceleration = *angular_acceleration;
	  // data_out->angle_error = angle_error;
	  // data_out->angle_rate_error = angular_rate_error;
	  // tf::vectorEigenToMsg(*angular_acceleration, data_out->angular_acceleration);
	  // tf::vectorEigenToMsg(angle_error, data_out->angle_error);
	  // tf::vectorEigenToMsg(angular_rate_error, data_out->angle_rate_error);
		// ROS_INFO_STREAM("moments: "<< *angular_acceleration);
	}

	void MultiEcbfController::getAttThrust(Eigen::VectorXd* att_thrust) const{
		assert(att_thrust);

		Eigen::Vector3d forces;
		// ComputeDesiredForces(&forces);
		// double thrust = forces.dot(odometry_01.orientation.toRotationMatrix().col(2));
		// double throttle = std::min(std::max(0.0, thrust/max_thrust_), 1.0);

		// Eigen::Matrix3d R_des;
		// desAttFromForces(forces, com_traj_01.getYaw(), &R_des);

		// Eigen::Quaterniond des_quat(R_des);
		// (*att_thrust)[0] = throttle;
		// (*att_thrust)[1] = des_quat.x();
		// (*att_thrust)[2] = des_quat.y();
		// (*att_thrust)[3] = des_quat.z();
		// (*att_thrust)[4] = des_quat.w();
	}

	void MultiEcbfController::setOdometry_01(const EigenOdometry& odometry){
		odometry_01 = odometry;
	}
	void MultiEcbfController::setOdometry_02(const EigenOdometry& odometry){
		odometry_02 = odometry;
	}

	void MultiEcbfController::setComOdom_01(const EigenOdometry& odometry){
		com_odom_01 = odometry;
		controller_active_ = true;
	}
	void MultiEcbfController::setComOdom_02(const EigenOdometry& odometry){
		com_odom_02 = odometry;
		controller_active_ = true;
	}

	void MultiEcbfController::setTraj_01(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
		com_traj_01 = command_trajectory;
		controller_active_ = true;
	}

	void MultiEcbfController::setTraj_02(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
		com_traj_02 = command_trajectory;
		controller_active_ = true;
	}
	// void MultiEcbfController::setLanding_01(){
	// 	landing_01 = true;
	// }
	// void MultiEcbfController::setLanding_02(){
	// 	landing_02 = true;
	// }


}