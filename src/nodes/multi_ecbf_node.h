
#ifndef DRONE_CONTROLLER_MULTI_ECBF_NODE_H_
#define DRONE_CONTROLLER_MULTI_ECBF_NODE_H_

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
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
//#include <apriltag_ros/AprilTagDetectionArray.h>

// #include "rrc_control/common.h"
#include "drone_controller/multi_ecbf_controller.h"

namespace drone_controller{
	class MultiEcbfNode{
	public:
		MultiEcbfNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
		~MultiEcbfNode();

		void initParams();
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;
		MultiEcbfController position_controller_;
		
		bool actuator_enabled_;
  		bool received_home_pose;

		geometry_msgs::PoseStamped home_pose_;

		mavros_msgs::State current_state_;
		mavros_msgs::SetMode offb_set_mode_;
		mavros_msgs::CommandBool arm_cmd_;

		ros::ServiceClient arming_client_;
		ros::ServiceClient set_mode_client_;
		ros::ServiceServer ctrltriggerServ_;
		ros::ServiceServer land_service_;


		ros::Subscriber odom_sub_01;
		ros::Subscriber odom_sub_02;
		ros::Subscriber com_odom_sub_01;
		ros::Subscriber com_odom_sub_02;
		ros::Subscriber pose_sub_;
		ros::Subscriber traj_sub_01;
		ros::Subscriber traj_sub_02;
		ros::Subscriber mavState_sub_;

		ros::Publisher thr_pub_;
		ros::Publisher att_pub_;
		ros::Publisher rpm_pub_01;
		ros::Publisher rpm_pub_02;
		ros::Publisher tgt_pub_;

		mav_msgs::EigenTrajectoryPointDeque com_01;
		mav_msgs::EigenTrajectoryPointDeque com_02;

		std::deque<ros::Duration> com_wt_01;
		std::deque<ros::Duration> com_wt_02;
		ros::Timer com_timer_01;
		ros::Timer com_timer_02;
		ros::Timer pub_timer_;
		ros::Timer mavCom_timer_;
		ros::Timer sts_timer_;

		ros::Time last_request_;
		ros::Time reference_request_now_;
		ros::Time reference_request_last_;

		enum FlightState { WAITING_FOR_HOME_POSE, MISSION_EXECUTION, LANDING, LANDED } node_state_;

		void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
		void OdomCallback_01(const nav_msgs::OdometryConstPtr& msg);
		void OdomCallback_02(const nav_msgs::OdometryConstPtr& msg);
		void TrajCallback_01(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);
		void TrajCallback_02(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);

		void ComCallback_01(const ros::TimerEvent &e);
		void ComCallback_02(const ros::TimerEvent &e);
		void PubCallback(const ros::TimerEvent &e);
		void MavComCallback(const ros::TimerEvent &e);
		void MavStateCallback(const mavros_msgs::State::ConstPtr &msg);
		void StsCallback(const ros::TimerEvent &e);
		void ComOdomCallback_01(const nav_msgs::OdometryConstPtr& msg);
		void ComOdomCallback_02(const nav_msgs::OdometryConstPtr& msg);
	};
}

#endif
