#include <ros/ros.h>
#include <std_msgs/float64.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

const float DISTANCE_LRF_TO_CENTER = 0.00;	//[m]

ros::Publisher pub;
geometry_msgs::PoseStamped pose_msg;

float yaw = 0.0f;
float yawfirst = 0.0f;
float old_x = 0;
float old_y = 0;

void encXCallback(const std_msgs::float64& msg)
{
	pose_msg.pose.position.x += (msg.data - old_x) * cos(yaw);
	pose_msg.pose.position.x += (msg.data - old_x) * cos(M_PI/2 + yaw);
	old_x = msg.data;
	pub.publish(pose_msg);
}

void encYCallback(const std_msgs::float64& msg)
{
	pose_msg.pose.position.y += (msg.data - old_y) * cos(yaw);
	pose_msg.pose.position.x += (msg.data - old_y) * cos(M_PI/2 + yaw);
	old_x = msg.data;
	pub.publish(pose_msg);
}

void imuCallback(const sensor_msgs::Imu& msg)
{
	yaw = msg.orientation.z;
	pose_msg.pose.orientation.w = msg.orientation.w;
	pose_msg.pose.orientation.x = msg.orientation.x;
	pose_msg.pose.orientation.y = msg.orientation.y;
	pose_msg.pose.orientation.z = msg.orientation.z;
	pub.publish(pose_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "deadreckoning");

	ros::NodeHandle nh;

	ros::Subscriber subEncX = nh.subscribe("/enc_distance_X", 1000, encXCallback);
	ros::Subscriber subEncY = nh.subscribe("/enc_distance_Y", 1000, encYCallback);
	ros::Subscriber subIMU = nh.subscribe("/imu/data_raw", 1000, imuCallback);

	pose_msg.header.frame_id = "/map";
	pose_msg.pose.position.x = pose_msg.pose.position.y = pose_msg.pose.position.z = 0.0f;
	pose_msg.pose.orientation.x = pose_msg.pose.orientation.y = pose_msg.pose.orientation.z = pose_msg.pose.orientation.w = 0.0f;

	pub = nh.advertise<geometry_msgs::PoseStamped>("/robot/pose", 5);
	ros::spin();

	return 0;
}
