#include "multi_ecbf_node.h"

namespace drone_controller{
	MultiEcbfNode::MultiEcbfNode(
		const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
	:nh_(nh), private_nh_(private_nh){
		initParams();

		pose_sub_ = nh_.subscribe("pose_msg", 1, 
                            &MultiEcbfNode::PoseCallback, this);
		odom_sub_01 = nh_.subscribe("odom_msg_01", 1, 
                            &MultiEcbfNode::OdomCallback_01, this);
		odom_sub_02 = nh_.subscribe("odom_msg_02", 1, 
                            &MultiEcbfNode::OdomCallback_02, this);
		com_odom_sub_01 = nh_.subscribe("com_odom_msg_01", 1, 
                            &MultiEcbfNode::ComOdomCallback_01, this);
		com_odom_sub_02 = nh_.subscribe("com_odom_msg_02", 1, 
                            &MultiEcbfNode::ComOdomCallback_02, this);
		traj_sub_01 = nh_.subscribe("traj_msg_01", 1, 
                            &MultiEcbfNode::TrajCallback_01, this);
		traj_sub_02 = nh_.subscribe("traj_msg_02", 1, 
                            &MultiEcbfNode::TrajCallback_02, this);
		mavState_sub_ = nh_.subscribe("mavros/state", 1, 
							&MultiEcbfNode::MavStateCallback, this, ros::TransportHints().tcpNoDelay());

        pub_timer_ = nh_.createTimer(ros::Duration(0.02), &MultiEcbfNode::PubCallback, this,
			false);
        com_timer_01 = nh_.createTimer(ros::Duration(0), &MultiEcbfNode::ComCallback_01, this,
			true, false);
        com_timer_01 = nh_.createTimer(ros::Duration(0), &MultiEcbfNode::ComCallback_02, this,
			true, false);
        sts_timer_ = nh_.createTimer(ros::Duration(0), &MultiEcbfNode::StsCallback, this, false);
        mavCom_timer_ = nh_.createTimer(ros::Duration(0), &MultiEcbfNode::MavComCallback, this, false);

        rpm_pub_01 = nh_.advertise<mav_msgs::Actuators>(
			"drone1/command/motor_speed", 1);
        rpm_pub_02 = nh_.advertise<mav_msgs::Actuators>(
			"drone2/command/motor_speed", 1);
        tgt_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
        att_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/attitude_msg",100);
        thr_pub_ = nh_.advertise<mavros_msgs::Thrust>("/thrust_msg", 100);

        genHomePose(Eigen::Vector3d(0,0,0), &home_pose_.pose);
	}

	MultiEcbfNode::~MultiEcbfNode() {}

	void MultiEcbfNode::initParams(){
		private_nh_.getParam("actuator_enabled", actuator_enabled_);
		GetParameterArray(private_nh_, &position_controller_.Kp_p_, "control/Kp_p", Eigen::Vector3d(4.0, 4.0, 4.0));
		GetParameterArray(private_nh_, &position_controller_.Kv_p_, "control/Kv_p", Eigen::Vector3d(2.7, 2.7, 2.7));
		GetParameterArray(private_nh_, &position_controller_.Ki_p_, "control/Ki_p", Eigen::Vector3d(0.2, 0.2, 0.2));
		GetParameterArray(private_nh_, &position_controller_.Kp_q_, "control/Kp_q", Eigen::Vector3d(1.0, 1.0, 0.03));
		GetParameterArray(private_nh_, &position_controller_.Kv_q_, "control/Kv_q", Eigen::Vector3d(0.22, 0.22, 0.01));
		GetParameterArray(private_nh_, &position_controller_.mass_inertia_, "drone/mass_inertia", Eigen::Vector4d(1.0, 0.01, 0.01, 0.02));
		GetParameter(private_nh_, &position_controller_.max_thrust_, "control/max_thrust", 15.0);
		GetParameter(private_nh_, &position_controller_.max_velocity_, "control/max_velocity", 0.5);
		GetParameter(private_nh_, &position_controller_.Kh_scale_, "control/Kh_scale", 1.0);
		GetParameter(private_nh_, &position_controller_.Khl_b_, "control/Khl_b", 1.0);
		GetParameter(private_nh_, &position_controller_.Khs_b_, "control/Khs_b", 1.0);
		GetParameter(private_nh_, &position_controller_.Khe_b_, "control/Khe_b", 1.0);
		GetParameter(private_nh_, &position_controller_.Kh_ae_, "control/Kh_ae", 1.0);
		GetParameter(private_nh_, &position_controller_.Kh_be_, "control/Kh_be", 1.0);
		GetParameter(private_nh_, &position_controller_.Kh_sph_, "control/Kh_sph", 1.0);
		GetParameter(private_nh_, &position_controller_.Kh_ell_, "control/Kh_ell", 1.0);
		GetParameter(private_nh_, &position_controller_.Kh_offset_, "control/Kh_offset", 0.0);
		GetRotorConfig(private_nh_, &position_controller_.rotor_config_);

		position_controller_.initParams();
		pub_timer_.start();
		mavCom_timer_.start();
		sts_timer_.start();
	}

	void MultiEcbfNode::PoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
		EigenOdometry odometry;
		eigenOdometryFromPoseMsg(msg, &odometry);
		position_controller_.setOdometry_01(odometry);
		home_pose_.pose.position = msg->pose.pose.position;
	}

	void MultiEcbfNode::OdomCallback_01(const nav_msgs::OdometryConstPtr& msg){
		EigenOdometry odometry;
		eigenOdometryFromMsg(msg, &odometry);
		position_controller_.setOdometry_01(odometry);
		if (actuator_enabled_){
			Eigen::VectorXd ref_rpms;
			position_controller_.getRPMs_01(&ref_rpms);
			// if(position_controller_.controller_active_ && ref_rpms.norm() < 0.1){
			// 	// position_controller_.setLanding_01();
			// }
			mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

			actuator_msg->angular_velocities.clear();
			for (int i = 0; i < ref_rpms.size(); i++)
			  actuator_msg->angular_velocities.push_back(ref_rpms[i]);
			actuator_msg->header.stamp = ros::Time::now();
			// data_out_.header.stamp = ros::Time::now();

			rpm_pub_01.publish(actuator_msg);
			// ROS_INFO_STREAM("Publishing First Drone");		
		}
	}

	void MultiEcbfNode::OdomCallback_02(const nav_msgs::OdometryConstPtr& msg){
		EigenOdometry odometry;
		eigenOdometryFromMsg(msg, &odometry);
		position_controller_.setOdometry_02(odometry);
		// ROS_INFO_STREAM("Publishing Second Drone");
		if (actuator_enabled_){
			Eigen::VectorXd ref_rpms;
			position_controller_.getRPMs_02(&ref_rpms);
			// if(position_controller_.controller_active_ && ref_rpms.norm() < 0.1){
			// 	// position_controller_.setLanding_02();
			// }
			mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

			actuator_msg->angular_velocities.clear();
			for (int i = 0; i < ref_rpms.size(); i++)
			  actuator_msg->angular_velocities.push_back(ref_rpms[i]);
			actuator_msg->header.stamp = ros::Time::now();
			// data_out_.header.stamp = ros::Time::now();

			rpm_pub_02.publish(actuator_msg);
			// ROS_INFO_STREAM("Publishing Second Drone");
		}
	}

	void MultiEcbfNode::TrajCallback_01(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg){
		ROS_INFO_ONCE("Received first waypoint");
		const size_t no_points = msg->points.size();
		if (no_points<1){
			ROS_WARN_STREAM("Received a trajectory message with no points.");
		}
		else{
			com_timer_01.stop();
			com_01.clear();
			com_wt_01.clear();

			mav_msgs::EigenTrajectoryPoint eigen_ref;
			mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_ref);
			com_01.push_front(eigen_ref);

			for(size_t i = 1; i < no_points; i++){
				mav_msgs::eigenTrajectoryPointFromMsg(msg->points[i], &eigen_ref);
				com_01.push_back(eigen_ref);
		  		com_wt_01.push_back(msg->points[i].time_from_start - msg->points[i-1].time_from_start);
			}

			position_controller_.setTraj_01(com_01.front());
			com_01.pop_front();

			if (no_points > 1) {
			  com_timer_01.setPeriod(com_wt_01.front());
			  com_wt_01.pop_front();
			  com_timer_01.start();
			}
		}
	}

	void MultiEcbfNode::TrajCallback_02(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg){
		ROS_INFO_ONCE("Received first waypoint");
		const size_t no_points = msg->points.size();
		if (no_points<1){
			ROS_WARN_STREAM("Received a trajectory message with no points.");
		}
		else{
			com_timer_02.stop();
			com_02.clear();
			com_wt_02.clear();

			mav_msgs::EigenTrajectoryPoint eigen_ref;
			mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_ref);
			com_02.push_front(eigen_ref);

			for(size_t i = 1; i < no_points; i++){
				mav_msgs::eigenTrajectoryPointFromMsg(msg->points[i], &eigen_ref);
				com_02.push_back(eigen_ref);
		  		com_wt_02.push_back(msg->points[i].time_from_start - msg->points[i-1].time_from_start);
			}

			position_controller_.setTraj_02(com_02.front());
			com_02.pop_front();

			if (no_points > 1) {
			  com_timer_02.setPeriod(com_wt_02.front());
			  com_wt_02.pop_front();
			  com_timer_02.start();
			}
		}
	}

	void MultiEcbfNode::ComOdomCallback_01(const nav_msgs::OdometryConstPtr& msg){
		ROS_INFO_ONCE("Received pose 01");
		EigenOdometry odometry;
		eigenOdometryFromMsg(msg, &odometry);
		position_controller_.setComOdom_01(odometry);
	}
	void MultiEcbfNode::ComOdomCallback_02(const nav_msgs::OdometryConstPtr& msg){
		ROS_INFO_ONCE("Received pose 02");
		EigenOdometry odometry;
		eigenOdometryFromMsg(msg, &odometry);
		position_controller_.setComOdom_02(odometry);
	}

	void MultiEcbfNode::ComCallback_01(const ros::TimerEvent &e){	
		ROS_INFO_STREAM("Timed callback working");
		position_controller_.setTraj_01(com_01.front());
		com_01.pop_front();
		com_timer_01.stop();

		if(!com_wt_01.empty()){
		  com_timer_01.setPeriod(com_wt_01.front());
		  com_wt_01.pop_front();
		  com_timer_01.start();
		}
	}

	void MultiEcbfNode::ComCallback_02(const ros::TimerEvent &e){	
		ROS_INFO_STREAM("Timed callback working");
		position_controller_.setTraj_02(com_02.front());
		com_02.pop_front();
		com_timer_02.stop();

		if(!com_wt_02.empty()){
		  com_timer_02.setPeriod(com_wt_02.front());
		  com_wt_02.pop_front();
		  com_timer_02.start();
		}
	}

	void MultiEcbfNode::PubCallback(const ros::TimerEvent &e){
		if (actuator_enabled_){
			Eigen::VectorXd ref_rpms;
			position_controller_.getRPMs_01(&ref_rpms);
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

	void MultiEcbfNode::StsCallback(const ros::TimerEvent &e){
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

	void MultiEcbfNode::MavComCallback(const ros::TimerEvent &e){
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

	void MultiEcbfNode::MavStateCallback(const mavros_msgs::State::ConstPtr &msg) { 
		current_state_ = *msg;
	}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "multiecbf_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  drone_controller::MultiEcbfNode multiecbf_node(nh, private_nh);

  ros::spin();

  return 0;
}