#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/tf.h>

bool premierTour = true;
float firstx, firsty;
float currentx, currenty;
float dist;
float distAParcourir, AngleAFaire;
double roll, pitch, yaw;
geometry_msgs::Twist msg_Twist;

void chatterCallback(const nav_msgs::Odometry msg_Odom)
{
	if (premierTour)
	{
		firstx = msg_Odom.pose.pose.position.x;
		firsty = msg_Odom.pose.pose.position.y;
		premierTour = false;
	}
	currentx = msg_Odom.pose.pose.position.x;
	currenty = msg_Odom.pose.pose.position.y;
	dist = sqrt(pow(msg_Odom.pose.pose.position.x - firstx, 2) + pow(msg_Odom.pose.pose.position.y - firsty, 2));
	ROS_INFO("Start : (%.2f, %.2f),"
			 " Current : (%.2f, %.2f),"
			 " Distance = %.2f",
			 firstx,
			 firsty,
			 msg_Odom.pose.pose.position.x,
			 msg_Odom.pose.pose.position.y,
			 dist);

	tf::Quaternion q(msg_Odom.pose.pose.orientation.x,
					 msg_Odom.pose.pose.orientation.y,
					 msg_Odom.pose.pose.orientation.z,
					 msg_Odom.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);

	m.getRPY(roll, pitch, yaw);
	ROS_INFO("Yaw = %f", yaw);
}

void avancerCallback(const std_msgs::Float32 distance)
{
	distAParcourir = distance.data;
}

void tournerCallback(const std_msgs::Float32 angle)
{
	AngleAFaire = yaw + angle.data * M_PI / 180;
	if (AngleAFaire < -M_PI)
		AngleAFaire += 2 * M_PI;
	if (AngleAFaire > M_PI)
		AngleAFaire -= 2 * M_PI;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle node;

	ros::Subscriber sub = node.subscribe("/odom", 1, chatterCallback);
	ros::Subscriber sub1 = node.subscribe("/avancer", 1, avancerCallback);
	ros::Subscriber sub2 = node.subscribe("/tourner", 1, tournerCallback);

	ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	ros::Rate rate(100);
	while (ros::ok())
	{
		// if(dist < 1) //meters
		// {
		// 	msg_Twist.linear.x = 0.1;
		// 	pub.publish(msg_Twist);
		// }
		// else
		// {
		// 	msg_Twist.linear.x = 0;
		// 	ROS_INFO("%lf",yaw);
		// 	if(yaw < 1.57)
		// 	{
		// 		msg_Twist.angular.z = 0.3;
		// 		pub.publish(msg_Twist);
		// 	}else
		// 	{
		// 		msg_Twist.angular.z = 0;
		// 		pub.publish(msg_Twist);
		// 	}
		// }
		if (dist < distAParcourir)
		{
			msg_Twist.linear.x = 0.1;
			// pub.publish(msg_Twist);
		}
		else
		{
			firstx = currentx;
			firsty = currenty;
			distAParcourir = 0;
			msg_Twist.linear.x = 0;
			// pub.publish(msg_Twist);
		}
		if (yaw < AngleAFaire - 0.1)
		{
			msg_Twist.angular.z = 0.3;
			if (yaw < AngleAFaire - 0.01)
				msg_Twist.angular.z = 0.1;
		}
		else if (yaw > AngleAFaire + 0.1)
		{
			msg_Twist.angular.z = -0.3;
			if (yaw < AngleAFaire + 0.01)
				msg_Twist.angular.z = -0.01;
		}
		else
		{
			msg_Twist.angular.z = 0;
			// AngleAFaire = 0;
		}
		pub.publish(msg_Twist);
		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
