import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Transform




def odom_cb(msg):
	traj_pub = rospy.Publisher('/pelican/command/trajectory', MultiDOFJointTrajectory, queue_size=10)
	# rate = rospy.Rate(10)
	traj_msg = MultiDOFJointTrajectory()
	traj_msg.header.stamp = rospy.Time.now()

	transform_msg = Transform()
	velocities_msg = Twist()
	points_msg = MultiDOFJointTrajectoryPoint()

	# transform_msg.translation = msg.pose.pose.position
	transform_msg.translation.x = msg.pose.pose.position.x + 0.0*msg.twist.twist.linear.x
	transform_msg.translation.y = msg.pose.pose.position.y + 0.0*msg.twist.twist.linear.y
	transform_msg.translation.z = 0.4
	transform_msg.rotation = msg.pose.pose.orientation
	velocities_msg.linear = msg.twist.twist.linear
	points_msg.transforms.append(transform_msg)
	points_msg.velocities.append(velocities_msg)

	traj_msg.points.append(points_msg)


	# traj_msg.points[0].transforms[0].translation = msg.pose.pose.position
	# traj_msg.points[0].velocities[0].linear = msg.twist.twist.linear

	traj_pub.publish(traj_msg)
	# rate.sleep()

def spin():
	rospy.init_node('turtle_bot_odom_publisher', anonymous=True)
	pub = rospy.Publisher('/pelican/cmd_vel', Twist, queue_size=10)

	rate = rospy.Rate(30)

	cmdvel_msg = Twist()
	cmdvel_msg.linear.x = 0.0
	cmdvel_msg.angular.z = 0.0

	rospy.sleep(1)

	pub.publish(cmdvel_msg)
	rate.sleep()

	sub = rospy.Subscriber('/pelican/odom', Odometry, odom_cb)
	rospy.spin()




if __name__ == '__main__':
	try:
		spin()
	except rospy.ROSInterruptionException:
		pass

