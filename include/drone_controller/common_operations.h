#ifndef DRONE_CONTROLLER_COMMON_H_
#define DRONE_CONTROLLER_COMMON_H_

#include <assert.h>
#include <math.h>
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>


#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>



namespace drone_controller{
	struct Rotor {
		Rotor()
			: angle(0.0),
			  arm_length(0.2),
			  rotor_force_constant(9.5e-8),
			  rotor_moment_constant(0.016),
			  direction(1) {}
		Rotor(double _angle, double _arm_length,
			double _rotor_force_constant, double _rotor_moment_constant,
			int _direction)
			: angle(_angle),
			  arm_length(_arm_length),
			  rotor_force_constant(_rotor_force_constant),
			  rotor_moment_constant(_rotor_moment_constant),
			  direction(_direction) {}
		double angle;
		double arm_length;
		double rotor_force_constant;
		double rotor_moment_constant;
		int direction;
	};

	struct RotorConfiguration {
		RotorConfiguration() {
			// Rotor configuration of Asctec Firefly.
			rotors.push_back(
			  Rotor(0.0, 0.2, 9.5e-8,
			        0.016, 1));
			rotors.push_back(
			  Rotor(1.047, 0.2, 9.5e-8,
			        0.016, -1));
			rotors.push_back(
			  Rotor(2.094, 0.2, 9.5e-8,
			        0.016, 1));
			rotors.push_back(
			  Rotor(3.14, 0.2, 9.5e-8,
			        0.016, -1));
			rotors.push_back(
			  Rotor(4.189, 0.2, 9.5e-8,
			        0.016, 1));
			rotors.push_back(
			  Rotor(5.236, 0.2, 9.5e-8,
			        0.016, -1));
		}
		std::vector<Rotor> rotors;
	};

	struct EigenOdometry {
		EigenOdometry()
		  : position(0.0, 0.0, 0.0),
		    orientation(Eigen::Quaterniond::Identity()),
		    velocity(0.0, 0.0, 0.0),
		    angular_velocity(0.0, 0.0, 0.0),
		    timestamp_ns(0) {};

		EigenOdometry(const Eigen::Vector3d& _position,
		            const Eigen::Quaterniond& _orientation,
		            const Eigen::Vector3d& _velocity,
		            const Eigen::Vector3d& _angular_velocity,
		            const uint64_t _timestamp_ns) {
			position = _position;
			orientation = _orientation;
			velocity = _velocity;
			angular_velocity = _angular_velocity;
			timestamp_ns = _timestamp_ns;
		};

		Eigen::Vector3d position;
		Eigen::Quaterniond orientation;
		Eigen::Vector3d velocity; // Velocity is expressed in the Body frame!
		Eigen::Vector3d angular_velocity;
		uint64_t timestamp_ns;
	};


	template<typename T> inline void GetParameter(const ros::NodeHandle& nh, T* value, 
		const std::string& key, const T& def_value){
		ROS_ASSERT(value != nullptr);
		bool have_parameter = nh.getParam(key, *value);
		if (!have_parameter) {
		ROS_WARN_STREAM("[rosparam]: could not find parameter " << nh.getNamespace()
		                << "/" << key << ", setting to default: " << def_value);
		*value = def_value;
		}
	}

	template<typename T> inline void GetParameterArray(const ros::NodeHandle& nh, T* value, 
		const std::string& key, const T& def_value){
		ROS_ASSERT(value != nullptr);


		std::string str;
		std::vector<std::string> values;
		bool have_parameter = nh.getParam(key, str);
		if (!have_parameter) {
			ROS_WARN_STREAM("[rosparam]: could not find parameter " << nh.getNamespace()
			                << "/" << key << ", setting to default: " << def_value);
			if (def_value.size() == value->size()){
				*value = def_value;
			}
			else{
				*value = Eigen::VectorXd::Zero(value->size());
			}
			return;

		}
		boost::erase_all(str, " ");
		boost::split ( values, str, boost::is_any_of(","));

		for(size_t i = 0; i < values.size(); i++){
			(*value)[i] = std::stod(values[i]);
		}
		for(size_t i = values.size(); i < value->size(); i++){
			if (i < def_value.size()){
				(*value)[i] = def_value[i];
			}
			else{
				(*value)[i] = 0.0;
			}
		}
	}

	inline void GetRotorConfig(const ros::NodeHandle& nh,
	                          RotorConfiguration* rotor_configuration) {
		std::map<std::string, double> single_rotor;
		std::string rotor_configuration_string = "rotor_configuration/";
		unsigned int i = 0;
		while (nh.getParam(rotor_configuration_string + std::to_string(i), single_rotor)) {
			if (i == 0) {
				rotor_configuration->rotors.clear();
			}
			Rotor rotor;
			nh.getParam(rotor_configuration_string + std::to_string(i) + "/angle",
			         rotor.angle);
			nh.getParam(rotor_configuration_string + std::to_string(i) + "/arm_length",
			         rotor.arm_length);
			nh.getParam(rotor_configuration_string + std::to_string(i) + "/rotor_force_constant",
			         rotor.rotor_force_constant);
			nh.getParam(rotor_configuration_string + std::to_string(i) + "/rotor_moment_constant",
			         rotor.rotor_moment_constant);
			nh.getParam(rotor_configuration_string + std::to_string(i) + "/direction",
			         rotor.direction);
			rotor_configuration->rotors.push_back(rotor);
			++i;
			ROS_INFO_STREAM("Rotor: "<< i);
		}
		ROS_INFO_STREAM("rotor_configuration done: " << rotor_configuration->rotors.size());
	}

	inline double signumFn(double s, double var_pi) {
		return ((abs(s) > var_pi) ? s/abs(s) : s/var_pi);
	}

	inline void desAttFromForces(const Eigen::Vector3d forces, double yaw, Eigen::Matrix3d* des_att) {		
		assert(des_att);	

		// Get the desired rotation matrix.
		Eigen::Vector3d b1_des;
		b1_des << cos(yaw), sin(yaw), 0;

		Eigen::Vector3d b3_des;
		b3_des = forces / forces.norm();

		Eigen::Vector3d b2_des;
		b2_des = b3_des.cross(b1_des);
		b2_des.normalize();

		des_att->col(0) = b2_des.cross(b3_des);
		des_att->col(1) = b2_des;
		des_att->col(2) = b3_des;
	}

	inline void genHomePose(const Eigen::Vector3d& point, geometry_msgs::Pose* pose){
		pose->position.x = point[0];
		pose->position.y = point[1];
		pose->position.z = point[2];

		pose->orientation.x = 0;
		pose->orientation.y = 0;
		pose->orientation.z = 0;
		pose->orientation.w = 1;
	}


	template <class T> void waitForPredicate(const T *pred, const std::string &msg, double hz = 2.0) {
		ros::Rate pause(hz);
		ROS_INFO_STREAM(msg);
		while (ros::ok() && !(*pred)) {
			ros::spinOnce();
			pause.sleep();
		}
	}


	inline void eigenOdometryFromMsg(const nav_msgs::OdometryConstPtr& msg,
	                             EigenOdometry* odometry) {
		odometry->position = mav_msgs::vector3FromPointMsg(msg->pose.pose.position);
		odometry->orientation = mav_msgs::quaternionFromMsg(msg->pose.pose.orientation);
		odometry->velocity = mav_msgs::vector3FromMsg(msg->twist.twist.linear);
		odometry->angular_velocity = mav_msgs::vector3FromMsg(msg->twist.twist.angular);
	}

	inline void calculateAllocationMatrix(const RotorConfiguration& rotor_configuration,
	                                  Eigen::Matrix4Xd* allocation_matrix) {
		assert(allocation_matrix != nullptr);
		allocation_matrix->resize(4, rotor_configuration.rotors.size());
		unsigned int i = 0;
		for (const Rotor& rotor : rotor_configuration.rotors) {
			// Set first row of allocation matrix.
			(*allocation_matrix)(0, i) = sin(rotor.angle) * rotor.arm_length
			                             * rotor.rotor_force_constant;
			// Set second row of allocation matrix.
			(*allocation_matrix)(1, i) = -cos(rotor.angle) * rotor.arm_length
			                             * rotor.rotor_force_constant;
			// Set third row of allocation matrix.
			(*allocation_matrix)(2, i) = -rotor.direction * rotor.rotor_force_constant
			                             * rotor.rotor_moment_constant;
			// Set forth row of allocation matrix.
			(*allocation_matrix)(3, i) = rotor.rotor_force_constant;
			++i;
		}
		Eigen::FullPivLU<Eigen::Matrix4Xd> lu(*allocation_matrix);
		// Setting the threshold for when pivots of the rank calculation should be considered nonzero.
		lu.setThreshold(1e-9);
		int rank = lu.rank();
		if (rank < 4) {
			std::cout << "The rank of the allocation matrix is " << lu.rank()
			          << ", it should have rank 4, to have a fully controllable system,"
			          << " check your configuration." << std::endl;
			}

	}

	inline void skewMatrixFromVector(Eigen::Vector3d& vector, Eigen::Matrix3d* skew_matrix) {
		*skew_matrix << 0, -vector.z(), vector.y(),
		              vector.z(), 0, -vector.x(),
		              -vector.y(), vector.x(), 0;
	}

	inline void vectorFromSkewMatrix(Eigen::Matrix3d& skew_matrix, Eigen::Vector3d* vector) {
		*vector << skew_matrix(2, 1), skew_matrix(0,2), skew_matrix(1, 0);
	}

	inline void eigenOdometryFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg,
	                             EigenOdometry* odometry) {
		assert(odometry);

		double dt = (double)(msg->header.stamp.toNSec() - odometry->timestamp_ns) * 1e-9;

		odometry->velocity = (mav_msgs::vector3FromPointMsg(msg->pose.pose.position) - 
		                      odometry->position) / dt;
		Eigen::Matrix3d R_cur = (mav_msgs::quaternionFromMsg(msg->pose.pose.orientation)).toRotationMatrix(); 
		Eigen::Matrix3d R_pre = (odometry->orientation).toRotationMatrix();

		Eigen::Matrix3d R_diff =  0.5 * (R_pre.transpose() * R_cur - R_cur.transpose() * R_pre);
		Eigen::Vector3d angle_diff;
		vectorFromSkewMatrix(R_diff, &angle_diff);
		odometry->angular_velocity = angle_diff / dt;

		odometry->timestamp_ns = msg->header.stamp.toNSec();
		odometry->position = mav_msgs::vector3FromPointMsg(msg->pose.pose.position);
		odometry->orientation = mav_msgs::quaternionFromMsg(msg->pose.pose.orientation);

		// odometry->position = mav_msgs::vector3FromPointMsg(msg->pose.position);
		// odometry->orientation = mav_msgs::quaternionFromMsg(msg->pose.pose.orientation);
		// odometry->velocity = mav_msgs::vector3FromMsg(msg->twist.twist.linear);
		// odometry->angular_velocity = mav_msgs::vector3FromMsg(msg->twist.twist.angular);
	}
	inline void rpmConversion(Eigen::VectorXd* rpms, const Eigen::MatrixX4d* ang_acc_rpms, 
									Eigen::Vector3d angular_acceleration, double thrust){
		Eigen::Vector4d angular_acceleration_thrust;
		angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
		angular_acceleration_thrust(3) = thrust;

		*rpms = *ang_acc_rpms * angular_acceleration_thrust;
		*rpms = rpms->cwiseMax(Eigen::VectorXd::Zero(rpms->rows()));
		*rpms = rpms->cwiseSqrt();
	}

	inline void getYawFromOrientation(double* yaw, const Eigen::Quaterniond& orientation){
		Eigen::Vector3d euler;
		Eigen::Quaterniond orientation_;
		orientation_.w() = orientation.w();
		orientation_.x() = orientation.x();
		orientation_.y() = orientation.z();
		orientation_.z() = orientation.y();
		// orientation_.w() = 1.0;
		// orientation_.x() = 0;
		// orientation_.y() = 0;
		// orientation_.z() = 0;
		euler = orientation.toRotationMatrix().eulerAngles(0,1,2);
		// ROS_INFO_STREAM("euler: " << euler);
		// ROS_INFO_STREAM("orientation: " << orientation[0] << " " << orientation[1] << " " << orientation[2] << " " << orientation[3]);
		*yaw = euler[2];
	}
	inline double constrainAngle(double x){
    x = fmod(x + 180,360);
    if (x < 0){
        x += 360;
    }
    return x - 180;
	}

}
#endif