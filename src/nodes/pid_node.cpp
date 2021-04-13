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
        rpm_pub_ = nh_.advertise<mav_msgs::Actuators>(
			mav_msgs::default_topics::COMMAND_ACTUATORS, 1);
        pub_timer_ = nh_.createTimer(ros::Duration(0.01), &PidNode::PubCallback, this,
			false);
        com_timer_ = nh_.createTimer(ros::Duration(0), &PidNode::ComCallback, this,
			true, false);
        att_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude",100);
        thr_pub_ = nh_.advertise<mavros_msgs::Thrust>("/mavros/setpoint_attitude/thrust", 100);
	}

	PidNode::~PidNode() {}

	void PidNode::initParams(){
		private_nh_.getParam("actuator_enabled", actuator_enabled_);
		GetParameterArray(private_nh_, &position_controller_.Kp_p_, "control/Kp_p", Eigen::Vector3d(4.0, 4.0, 4.0));
		GetParameterArray(private_nh_, &position_controller_.Kv_p_, "control/Kv_p", Eigen::Vector3d(2.7, 2.7, 2.7));
		GetParameterArray(private_nh_, &position_controller_.Ki_p_, "control/Ki_p", Eigen::Vector3d(0.2, 0.2, 0.2));
		GetParameterArray(private_nh_, &position_controller_.Kp_q_, "control/Kp_q", Eigen::Vector3d(1.0, 1.0, 0.03));
		GetParameterArray(private_nh_, &position_controller_.Kv_q_, "control/Kv_q", Eigen::Vector3d(0.22, 0.22, 0.01));
		GetParameterArray(private_nh_, &position_controller_.mass_inertia_, "drone/mass", Eigen::Vector4d(1.0, 0.01, 0.01, 0.02));
		GetRotorConfig(private_nh_, &position_controller_.rotor_config_);

		position_controller_.initParams();
		pub_timer_.start();
	}

	void PidNode::PoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
		EigenOdometry odometry;
		eigenOdometryFromPoseMsg(msg, &odometry);
		position_controller_.setOdometry(odometry);
	}

	void PidNode::OdomCallback(const nav_msgs::OdometryConstPtr& msg){
		EigenOdometry odometry;
		eigenOdometryFromMsg(msg, &odometry);
		position_controller_.setOdometry(odometry);
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

			rpm_pub_.publish(actuator_msg);
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

}
int main(int argc, char** argv) {
  ros::init(argc, argv, "pid_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  drone_controller::PidNode pid_node(nh, private_nh);

  ros::spin();

  return 0;
}