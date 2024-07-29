import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Transform
from scipy.spatial.transform import Rotation
import numpy as np

t = 0.0

def quaternion_to_euler_angle_vectorized1(q):
	w = q[0]
	x = q[1]
	y = q[2]
	z = q[3]
	ysqr = y * y

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = np.arctan2(t0, t1)

	t2 = +2.0 * (w * y - z * x)
	t2 = np.where(t2>+1.0,+1.0,t2)
	#t2 = +1.0 if t2 > +1.0 else t2

	t2 = np.where(t2<-1.0, -1.0, t2)
	#t2 = -1.0 if t2 < -1.0 else t2
	Y = np.arcsin(t2)

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = np.arctan2(t3, t4)

	return X, Y, Z 

def odom_cb_2(msg):
	global t
	traj_pub = rospy.Publisher('/com_odom_msg_02', Odometry, queue_size=10)
	# rate = rospy.Rate(10)
	# traj_msg = MultiDOFJointTrajectory()
	# traj_msg.header.stamp = rospy.Time.now()

	# transform_msg = Transform()
	# velocities_msg = Twist()
	# points_msg = MultiDOFJointTrajectoryPoint()

	# # transform_msg.translation = msg.pose.pose.position
	# transform_msg.translation.x = msg.pose.pose.position.x + 0.0*msg.twist.twist.linear.x
	# transform_msg.translation.y = msg.pose.pose.position.y + 0.0*msg.twist.twist.linear.y
	# transform_msg.translation.z = 0.4
	# transform_msg.rotation = msg.pose.pose.orientation
	# velocities_msg.linear = msg.twist.twist.linear
	# points_msg.transforms.append(transform_msg)
	# points_msg.velocities.append(velocities_msg)

	# traj_msg.points.append(points_msg)


	# # traj_msg.points[0].transforms[0].translation = msg.pose.pose.position
	# # traj_msg.points[0].velocities[0].linear = msg.twist.twist.linear

	# traj_pub.publish(traj_msg)

	qf = np.array([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z])
	Eu = quaternion_to_euler_angle_vectorized1(qf)
	odom_msg = Odometry()
	# odom_msg = msg
	# odom_msg.pose.pose.orientation.w = 0.7071;
	# odom_msg.pose.pose.orientation.x = 0.0;
	odom_msg.pose.pose.position.x = msg.pose.pose.position.x + np.cos(Eu[2])*(-0.05) - np.sin(Eu[2])*(0.0)
	odom_msg.pose.pose.position.y = msg.pose.pose.position.y + np.sin(Eu[2])*(-0.05) + np.cos(Eu[2])*(0.0)
	odom_msg.pose.pose.position.z = 0.31
	odom_msg.pose.pose.orientation.y = msg.pose.pose.orientation.z;
	# odom_msg.pose.pose.orientation.z = Eu[2];
	odom_msg.pose.pose.orientation.z = 0.0
	# odom_msg.twist.twist.rotation.z = 
	odom_msg.header.stamp = rospy.Time.now()

	if(t>400):
		traj_pub.publish(odom_msg)

def odom_cb_1(msg):
	global t
	traj_pub = rospy.Publisher('/com_odom_msg_01', Odometry, queue_size=10)
	# rate = rospy.Rate(10)
	# traj_msg = MultiDOFJointTrajectory()
	# traj_msg.header.stamp = rospy.Time.now()

	# transform_msg = Transform()
	# velocities_msg = Twist()
	# points_msg = MultiDOFJointTrajectoryPoint()

	# # transform_msg.translation = msg.pose.pose.position
	# transform_msg.translation.x = msg.pose.pose.position.x + 0.0*msg.twist.twist.linear.x
	# transform_msg.translation.y = msg.pose.pose.position.y + 0.0*msg.twist.twist.linear.y
	# transform_msg.translation.z = 0.4
	# transform_msg.rotation = msg.pose.pose.orientation
	# velocities_msg.linear = msg.twist.twist.linear
	# points_msg.transforms.append(transform_msg)
	# points_msg.velocities.append(velocities_msg)

	# traj_msg.points.append(points_msg)


	# # traj_msg.points[0].transforms[0].translation = msg.pose.pose.position
	# # traj_msg.points[0].velocities[0].linear = msg.twist.twist.linear

	# traj_pub.publish(traj_msg)
	qf = np.array([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z])
	Eu = quaternion_to_euler_angle_vectorized1(qf)

	odom_msg = Odometry()
	# odom_msg = msg
	# odom_msg.pose.pose.orientation.w = msg.pose;
	# odom_msg.pose.pose.orientation.x = 0.0;
	odom_msg.pose.pose.position.x = msg.pose.pose.position.x + np.cos(Eu[2])*(-0.05) - np.sin(Eu[2])*(0.0)
	odom_msg.pose.pose.position.y = msg.pose.pose.position.y + np.sin(Eu[2])*(-0.05) + np.cos(Eu[2])*(0.0)
	odom_msg.pose.pose.position.z = 0.31
	odom_msg.pose.pose.orientation.y = msg.pose.pose.orientation.z;
	# odom_msg.pose.pose.orientation.z = Eu[2];
	odom_msg.pose.pose.orientation.z = 0.0;
	odom_msg.header.stamp = rospy.Time.now()

	# rot = Rotation.from_quat(qf)
	# rot_euler = rot.as_euler('xyz', degrees=True)
	# euler_df = pd.DataFrame(data=rot_euler, columns=['x', 'y', 'z'])
	
	# print(Eu)

	if(t>400):
		traj_pub.publish(odom_msg)




def odom_cb(msg):
	traj_pub = rospy.Publisher('/drone1/command/trajectory', MultiDOFJointTrajectory, queue_size=10)
	# rate = rospy.Rate(10)
	traj_msg = MultiDOFJointTrajectory()
	traj_msg.header.stamp = rospy.Time.now()

	transform_msg = Transform()
	velocities_msg = Twist()
	points_msg = MultiDOFJointTrajectoryPoint()

	# transform_msg.translation = msg.pose.pose.position
	transform_msg.translation.x = msg.pose.pose.position.x + 0.0*msg.twist.twist.linear.x
	transform_msg.translation.y = msg.pose.pose.position.y + 0.0*msg.twist.twist.linear.y
	transform_msg.translation.z = 0.31
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
	global t
	rospy.init_node('turtle_bot_odom_publisher', anonymous=True)
	pub1 = rospy.Publisher('/usv1/cmd_vel', Twist, queue_size=10)
	pub2 = rospy.Publisher('/usv2/cmd_vel', Twist, queue_size=10)

	sub1 = rospy.Subscriber('/usv1/odom', Odometry, odom_cb_1)
	sub2 = rospy.Subscriber('/usv2/odom', Odometry, odom_cb_2)
	rospy.sleep(1)	

	rate = rospy.Rate(30)

	while not rospy.is_shutdown():

		cmdvel_msg_1 = Twist()
		cmdvel_msg_2 = Twist()
		cmdvel_msg_1.linear.x = 0.5
		cmdvel_msg_2.linear.x = 0.5
		cmdvel_msg_1.angular.z = 0.5*np.cos(0.01*t)
		cmdvel_msg_2.angular.z = -0.5*np.cos(0.01*t)


		pub1.publish(cmdvel_msg_1)
		pub2.publish(cmdvel_msg_2)
		rate.sleep()

		t = t + 1

	rospy.spin()




if __name__ == '__main__':
	try:
		spin()
	except rospy.ROSInterruptionException:
		pass

