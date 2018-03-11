#include <ros/ros.h>
#include <deadreckoning/enc.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

const float DISTANCE_LRF_TO_CENTER = 0.00;	//[m]

ros::Publisher pub;
geometry_msgs::PoseStamped pose_msg;
sensor_msgs::Imu& imu_msg;

float yaw = 0.0f;
float yawfirst = 0.0f;

void encCallback(const deadreckoning::enc& msg)
{
	pose_msg.pose.position.x += msg.distance_X * cos(yaw);
	pose_msg.pose.position.y += msg.distance_X * sin(yaw);
	pose_msg.pose.position.x += msg.distance_Y * cos(M_PI/2 + yaw);
	pose_msg.pose.position.y += msg.distance_Y * sin(M_PI/2 + yaw);
	pub.publish(pose_msg);
}

void imuCallback(const sensor_msgs::Imu& msg)
{
	yaw = msg.orientation.x;
	pose_msg.pose.orientation.w = msg.orientation.w;
	pose_msg.pose.orientation.x = msg.orientation.x;
	pose_msg.pose.orientation.y = msg.orientation.y;
	pose_msg.pose.orientation.z = msg.orientation.z;
	pub.publish(pose_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robotPose");

	ros::NodeHandle nh;

	ros::Subscriber subEnc = nh.subscribe("/enc", 1000, encCallback);
	ros::Subscriber subIMU = nh.subscribe("/imu/data_raw", 1000, imuCallback);

	pose_msg.header.frame_id = "/map";
	pose_msg.pose.position.x = pose_msg.pose.position.y = pose_msg.pose.position.z = 0.0f;
	pose_msg.pose.orientation.x = pose_msg.pose.orientation.y = pose_msg.pose.orientation.z = pose_msg.pose.orientation.w = 0.0f;

	pub = nh.advertise<geometry_msgs::PoseStamped>("/robot/pose", 5);
	ros::spin();

	return 0;
}
