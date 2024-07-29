#include "pid_node.h"

namespace drone_controller{
	PidNode::PidNode(
		const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
	:nh_(nh), private_nh_(private_nh){
		initParams();

		pose_sub_ = nh_.subscribe("pose_msg", 1, 
                            &PidNode::PoseCallback, this);
		odom_sub_ = nh_.subscribe("odom_msg", 1, 
                            &PidNode::OdomCallback, this);
		traj_sub_ = nh_.subscribe("traj_msg", 1, 
                            &PidNode::TrajCallback, this);
		mavState_sub_ = nh_.subscribe("mavros/state", 1, 
							&PidNode::MavStateCallback, this, ros::TransportHints().tcpNoDelay());

        pub_timer_ = nh_.createTimer(ros::Duration(0.02), &PidNode::PubCallback, this,
			false);
        com_timer_ = nh_.createTimer(ros::Duration(0), &PidNode::ComCallback, this,
			true, false);
        sts_timer_ = nh_.createTimer(ros::Duration(0), &PidNode::StsCallback, this, false);
        mavCom_timer_ = nh_.createTimer(ros::Duration(0), &PidNode::MavComCallback, this, false);

        rpm_pub_ = nh_.advertise<mav_msgs::Actuators>(
			mav_msgs::default_topics::COMMAND_ACTUATORS, 1);
        tgt_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
        att_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/attitude_msg",100);
        thr_pub_ = nh_.advertise<mavros_msgs::Thrust>("/thrust_msg", 100);

        genHomePose(Eigen::Vector3d(0,0,0), &home_pose_.pose);
	}

	PidNode::~PidNode() {}

	void PidNode::initParams(){
		private_nh_.getParam("actuator_enabled", actuator_enabled_);
		GetParameterArray(private_nh_, &position_controller_.Kp_p_, "control/Kp_p", Eigen::Vector3d(4.0, 4.0, 4.0));
		GetParameterArray(private_nh_, &position_controller_.Kv_p_, "control/Kv_p", Eigen::Vector3d(2.7, 2.7, 2.7));
		GetParameterArray(private_nh_, &position_controller_.Ki_p_, "control/Ki_p", Eigen::Vector3d(0.2, 0.2, 0.2));
		GetParameterArray(private_nh_, &position_controller_.Kp_q_, "control/Kp_q", Eigen::Vector3d(1.0, 1.0, 0.03));
		GetParameterArray(private_nh_, &position_controller_.Kv_q_, "control/Kv_q", Eigen::Vector3d(0.22, 0.22, 0.01));
		GetParameterArray(private_nh_, &position_controller_.mass_inertia_, "drone/mass_inertia", Eigen::Vector4d(1.0, 0.01, 0.01, 0.02));
		GetParameter(private_nh_, &position_controller_.max_thrust_, "control/max_thrust", 15.0);
		GetRotorConfig(private_nh_, &position_controller_.rotor_config_);

		position_controller_.initParams();
		pub_timer_.start();
		mavCom_timer_.start();
		sts_timer_.start();
	}

	void PidNode::PoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
		EigenOdometry odometry;
		eigenOdometryFromPoseMsg(msg, &odometry);
		position_controller_.setOdometry(odometry);
		home_pose_.pose.position = msg->pose.pose.position;
	}

	void PidNode::OdomCallback(const nav_msgs::OdometryConstPtr& msg){
		EigenOdometry odometry;
		eigenOdometryFromMsg(msg, &odometry);
		position_controller_.setOdometry(odometry);
		if (actuator_enabled_){
			Eigen::VectorXd ref_rpms;
			position_controller_.getRPMs(&ref_rpms);
			mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

			actuator_msg->angular_velocities.clear();
			for (int i = 0; i < ref_rpms.size(); i++)
			  actuator_msg->angular_velocities.push_back(ref_rpms[i]);
			actuator_msg->header.stamp = ros::Time::now();
			// data_out_.header.stamp = ros::Time::now();

			rpm_pub_.publish(actuator_msg);
		}
	}

	void PidNode::TrajCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg){
		ROS_INFO_ONCE("Received first waypoint");
		const size_t no_points = msg->points.size();
		if (no_points<1){
			ROS_WARN_STREAM("Received a trajectory message with no points.");
		}
		else{
			com_timer_.stop();
			com_.clear();
			com_wt_.clear();

			mav_msgs::EigenTrajectoryPoint eigen_ref;
			mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_ref);
			com_.push_front(eigen_ref);

			for(size_t i = 1; i < no_points; i++){
				mav_msgs::eigenTrajectoryPointFromMsg(msg->points[i], &eigen_ref);
				com_.push_back(eigen_ref);
		  		com_wt_.push_back(msg->points[i].time_from_start - msg->points[i-1].time_from_start);
			}

			position_controller_.setTraj(com_.front());
			com_.pop_front();

			if (no_points > 1) {
			  com_timer_.setPeriod(com_wt_.front());
			  com_wt_.pop_front();
			  com_timer_.start();
			}
		}
	}

	void PidNode::ComCallback(const ros::TimerEvent &e){	
		ROS_INFO_STREAM("Timed callback working");
		position_controller_.setTraj(com_.front());
		com_.pop_front();
		com_timer_.stop();

		if(!com_wt_.empty()){
		  com_timer_.setPeriod(com_wt_.front());
		  com_wt_.pop_front();
		  com_timer_.start();
		}

	}

	void PidNode::PubCallback(const ros::TimerEvent &e){
		if (actuator_enabled_){
			Eigen::VectorXd ref_rpms;
			position_controller_.getRPMs(&ref_rpms);
			mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

			actuator_msg->angular_velocities.clear();
			for (int i = 0; i < ref_rpms.size(); i++)
			  actuator_msg->angular_velocities.push_back(ref_rpms[i]);
			actuator_msg->header.stamp = ros::Time::now();
			// data_out_.header.stamp = ros::Time::now();

			// rpm_pub_.publish(actuator_msg);
		}
		else{
			Eigen::VectorXd att_thrust(5);
			position_controller_.getAttThrust(&att_thrust);
			mavros_msgs::Thrust thr_msg;
			thr_msg.thrust = att_thrust[0];

			geometry_msgs::PoseStamped att_msg;
			att_msg.pose.orientation.x = att_thrust[1];
			att_msg.pose.orientation.y = att_thrust[2];
			att_msg.pose.orientation.z = att_thrust[3];
			att_msg.pose.orientation.w = att_thrust[4];
			
			ros::Time now = ros::Time::now();
			thr_msg.header.stamp = now;
			att_msg.header.stamp = now;

			thr_pub_.publish(thr_msg);
			att_pub_.publish(att_msg);
		}
	  	// plot_data_pub_.publish(data_out_);
	}

	void PidNode::StsCallback(const ros::TimerEvent &e){
		// Enable OFFBoard mode and arm automatically
		// This is only run if the vehicle is simulated
		arm_cmd_.request.value = true;
		offb_set_mode_.request.custom_mode = "OFFBOARD";
		if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
			if (set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent) {
				ROS_INFO("Offboard enabled");
			}
			last_request_ = ros::Time::now();
		} 
		else {
			if (!current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
				if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
		  			ROS_INFO("Vehicle armed");
				}
				last_request_ = ros::Time::now();
			}
		}
	}

	void PidNode::MavComCallback(const ros::TimerEvent &e){
		switch (node_state_) {
			case WAITING_FOR_HOME_POSE:
				waitForPredicate(&received_home_pose, "Waiting for home pose...");
				ROS_INFO("Got pose! Drone Ready to be armed.");
				node_state_ = MISSION_EXECUTION;
				break;

			case MISSION_EXECUTION:
				if (!position_controller_.controller_active_){
					position_controller_.controller_active_ = true;
				} 
				break;

			case LANDING:
				home_pose_.header.stamp = ros::Time::now();
				tgt_pub_.publish(home_pose_);
				node_state_ = LANDED;
				ros::spinOnce();
				break;
			
			case LANDED:
				ROS_INFO("Landed. Please set to position control and disarm.");
				mavCom_timer_.stop();
				break;
		}
	}

	void PidNode::MavStateCallback(const mavros_msgs::State::ConstPtr &msg) { 
		current_state_ = *msg;
	}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pid_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  drone_controller::PidNode pid_node(nh, private_nh);

  ros::spin();

  return 0;
}