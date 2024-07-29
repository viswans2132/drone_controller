#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>
#include <thread>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "mavros_msgs/Thrust.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"



