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
float xGoal,yGoal;
float distGoal, thetaGoal;

geometry_msgs::Twist msg_Twist;
bool turn = false;
bool avancer = false;

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
	avancer = true;
	distAParcourir = distance.data;
}

void tournerCallback(const std_msgs::Float32 angle)
{
	turn = true;
	AngleAFaire = yaw + angle.data * M_PI / 180;
	if (AngleAFaire < -M_PI)
		AngleAFaire += 2 * M_PI;
	if (AngleAFaire > M_PI)
		AngleAFaire -= 2 * M_PI;
}

void goCallback(const geometry_msgs::PoseStamped msg){
	xGoal = msg.pose.position.x;
	yGoal = msg.pose.position.y;
	thetaGoal = atan2(yGoal,xGoal);
	distGoal = sqrt(pow(xGoal - currentx,2) + pow(yGoal - currenty, 2));
	std_msgs::Float32 tourner;
	tourner.data = 45;
	std_msgs::Float32 avancer;
	avancer.data = distGoal;
	tournerCallback(tourner);
	avancerCallback(avancer);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle node;

	ros::Subscriber sub = node.subscribe("/odom", 1, chatterCallback);
	ros::Subscriber sub1 = node.subscribe("/avancer", 1, avancerCallback);
	ros::Subscriber sub2 = node.subscribe("/tourner", 1, tournerCallback);
	ros::Subscriber sub3 = node.subscribe("/move_base_simple/goal", 1, goCallback);

	ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	ros::Rate rate(50);
	while (ros::ok())
	{
		if(avancer)
		{
			if (dist < distAParcourir)
			{
				msg_Twist.linear.x = 0.1;
			}
			else if (dist > distAParcourir)
			{
				msg_Twist.linear.x = - 0.1;
			}
			else
			{
				firstx = currentx;
				firsty = currenty;
				distAParcourir = 0;
				msg_Twist.linear.x = 0;
				avancer = false;
			}

		}

		if (turn)
		{
			if (yaw < AngleAFaire - 0.2 || yaw > AngleAFaire + 0.2)
			{

				if (yaw - AngleAFaire < 0)
				{
					msg_Twist.angular.z = 0.2;
				}
				else if (yaw - AngleAFaire > 0)
				{
					msg_Twist.angular.z = -0.2;
				}
			}
			else
			{
				msg_Twist.angular.z = 0;
				AngleAFaire = 0;
				turn = false;
			}
		}
		pub.publish(msg_Twist);
		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
