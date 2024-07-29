#include "drone_controller/ecbf_controller.h"
#include <OsqpEigen/OsqpEigen.h>

namespace drone_controller{
	EcbfController::EcbfController()
		: initialized_params_(false),
		  controller_active_(false)	{
	  	initParams();
	}

	EcbfController::~EcbfController() {}

	void EcbfController::initParams(){
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

	void EcbfController::getRPMs(Eigen::VectorXd* rpms) const{
		assert(rpms);
		assert(initialized_params_);

		rpms->resize(rotor_config_.rotors.size());

		if(!controller_active_){
			*rpms = Eigen::VectorXd::Zero(rpms->rows());
			return;
		}


		Eigen::Vector3d forces;
		ComputeDesiredForces(&forces);

		if(forces.norm() < 0.01){
			*rpms << 0.0, 0.0, 0.0, 0.0;
		}
		else{

		Eigen::Vector3d angular_acceleration;
		ComputeDesiredAngularAcc(forces, &angular_acceleration);

		// ROS_INFO_STREAM("Forces: " << forces);
		// Project thrust onto body z axis.
		double thrust = forces.dot(odometry_.orientation.toRotationMatrix().col(2));
		ROS_INFO_STREAM("Thrust: " << thrust);

		rpmConversion(rpms, &ang_acc_rpms_, angular_acceleration, thrust);
		}
	}

	void EcbfController::ComputeDesiredForces(Eigen::Vector3d* forces) const{
		assert(forces);

		Eigen::Vector3d position_error, psuedo_velocity_ref, position_error_abs;
		position_error = odometry_.position - com_traj_.position_W;
		static Eigen::Vector3d position_error_integral(0,0,0);
		if(controller_active_){
			static ros::Time last_time = ros::Time::now();
			ros::Time current_time = ros::Time::now();  
			position_error_integral += position_error * (current_time - last_time).toSec();
			last_time = current_time;
		}

		position_error_abs = position_error.cwiseAbs();


		if((position_error_abs[0] < 0.02) && (position_error_abs[1] < 0.02) && (position_error_abs[2] < 0.05)){
			*forces << 0.0, 0.0, 0.0;
		}
		else{

			// Transform velocity to world frame.
			const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
			Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;

			psuedo_velocity_ref = -position_error.cwiseProduct(Kp_p_) - 0.0*position_error_integral;
			// position_error[2] = position_error[2] - 0.1;

			// Eigen::Vector3d position_error_sq = position_error.array().square();
			double x2y2_sqrt = std::sqrt(position_error[0]*position_error[0] + position_error[1]*position_error[1]);

			// ROS_INFO_STREAM("Sqrt(4): " << std::sqrt(4) << " Exp(-1): " << std::exp(-1));


			constexpr double tolerance = 1e-4;
			Eigen::SparseMatrix<c_float> H_s(3,3);
		    H_s.insert(0,0) = 2;
		    H_s.insert(1,1) = 2;
		    H_s.insert(2,2) = 2;

		    Eigen::SparseMatrix<c_float> A_s(1,3);
		    A_s.insert(0,0) = Kh_scale_*position_error[0]*std::exp(-x2y2_sqrt)*(x2y2_sqrt-1)/x2y2_sqrt;
		    A_s.insert(0,1) = Kh_scale_*position_error[1]*std::exp(-x2y2_sqrt)*(x2y2_sqrt-1)/x2y2_sqrt;
		    A_s.insert(0,2) = 1;

		    double h = position_error[2] - Kh_scale_*(x2y2_sqrt)*std::exp(-x2y2_sqrt) - Kh_offset_;

		    Eigen::Matrix<c_float, 3, 1> gradient;
		    gradient << -2*psuedo_velocity_ref[0], -2*psuedo_velocity_ref[1], -2*psuedo_velocity_ref[2];

		    double dot_x2y2_sqrt = (1-x2y2_sqrt)*std::exp(-x2y2_sqrt)*(velocity_W[0]*position_error[0] 
		    						+ velocity_W[1]*position_error[1])/x2y2_sqrt;

		    Eigen::Matrix<c_float, 1, 1> lowerBound;
		    lowerBound << -Kh_b_*h + dot_x2y2_sqrt;

		    Eigen::Matrix<c_float, 1, 1> upperBound;
		    upperBound << 50;

		    OsqpEigen::Solver solver;
		    solver.settings()->setVerbosity(true);
		    solver.settings()->setAlpha(1.0);

		    solver.data()->setHessianMatrix(H_s);
		    solver.data()->setNumberOfVariables(3);

		    solver.data()->setNumberOfConstraints(1);
		    solver.data()->setHessianMatrix(H_s);
		    solver.data()->setGradient(gradient);
		    solver.data()->setLinearConstraintsMatrix(A_s);
		    solver.data()->setLowerBound(lowerBound);
		    solver.data()->setUpperBound(upperBound);

		    solver.initSolver();
		    solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError;
		    
		    Eigen::Matrix<c_float, 3, 1> expectedSolution;
	    	expectedSolution << psuedo_velocity_ref[0], psuedo_velocity_ref[1], psuedo_velocity_ref[2];

	    	Eigen::Vector3d velocity_ref = solver.getSolution();

	    	// psuedo_velocity_ref = 0.1*Eigen::Vector3d::Ones().cwiseMin(psuedo_velocity_ref.cwiseMax(-0.1*Eigen::Vector3d::Ones()));
	    	if (velocity_ref.norm() > max_velocity_){
	    		velocity_ref = velocity_ref*(max_velocity_/velocity_ref.norm());
	    	}
			Eigen::Vector3d velocity_error;
			velocity_error = velocity_W - com_traj_.velocity_W;

			Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());

			static Eigen::Vector3d position_error_integral(0,0,0);
			if(controller_active_){
				static ros::Time last_time = ros::Time::now();
				ros::Time current_time = ros::Time::now();  
				position_error_integral += position_error * (current_time - last_time).toSec();
				last_time = current_time;
			}



			*forces = - (velocity_error-velocity_ref).cwiseProduct(Kv_p_)
							+ (gravity_ * e_3 + com_traj_.acceleration_W) * mass_inertia_[0];
		}

		ROS_INFO_STREAM("forces: "<< *forces);
		// ROS_INFO_STREAM("Position Error: "<< position_error_abs);


	}

	void EcbfController::ComputeDesiredAngularAcc(const Eigen::Vector3d& forces,
	                                                     Eigen::Vector3d* angular_acceleration) const {
		assert(angular_acceleration);

		Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();
		Eigen::Matrix3d R_des;
		desAttFromForces(forces, com_traj_.getYaw(), &R_des);


		// Angle error according to pid et al.
		Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
		Eigen::Vector3d angle_error;
		vectorFromSkewMatrix(angle_error_matrix, &angle_error);
		// angle_error[2] = 0.0;

		// TODO(burrimi) include angular rate references at some point.
		Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
		angular_rate_des[2] = com_traj_.getYawRate();

		Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;



		*angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
		                       - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
	                       + odometry_.angular_velocity.cross(odometry_.angular_velocity); // we don't need the inertia matrix here
	  // data_out->angular_acceleration = *angular_acceleration;
	  // data_out->angle_error = angle_error;
	  // data_out->angle_rate_error = angular_rate_error;
	  // tf::vectorEigenToMsg(*angular_acceleration, data_out->angular_acceleration);
	  // tf::vectorEigenToMsg(angle_error, data_out->angle_error);
	  // tf::vectorEigenToMsg(angular_rate_error, data_out->angle_rate_error);
		// ROS_INFO_STREAM("moments: "<< *angular_acceleration);
	}

	void EcbfController::getAttThrust(Eigen::VectorXd* att_thrust) const{
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

	void EcbfController::setOdometry(const EigenOdometry& odometry){
		odometry_ = odometry;
	}

	void EcbfController::setTraj(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
		com_traj_ = command_trajectory;
		controller_active_ = true;
	}


}