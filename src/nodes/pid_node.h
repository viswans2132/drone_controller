
#ifndef DRONE_CONTROLLER_PID_NODE_H_
#define DRONE_CONTROLLER_PID_NODE_H_

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/Thrust.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
//#include <apriltag_ros/AprilTagDetectionArray.h>

// #include "rrc_control/common.h"
#include "drone_controller/pid_controller.h"

namespace drone_controller{
	class PidNode{
	public:
		PidNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
		~PidNode();

		void initParams();
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;
		PidController position_controller_;

		bool actuator_enabled_;

		ros::Subscriber odom_sub_;
		ros::Subscriber pose_sub_;
		ros::Subscriber traj_sub_;

		ros::Publisher thr_pub_;
		ros::Publisher att_pub_;
		ros::Publisher rpm_pub_;

		mav_msgs::EigenTrajectoryPointDeque com_;


		std::deque<ros::Duration> com_wt_;
		ros::Timer com_timer_;
		ros::Timer pub_timer_;

		void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
		void OdomCallback(const nav_msgs::OdometryConstPtr& msg);
		void TrajCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);

		void ComCallback(const ros::TimerEvent &e);
		void PubCallback(const ros::TimerEvent &e);
	};
}

#endif
