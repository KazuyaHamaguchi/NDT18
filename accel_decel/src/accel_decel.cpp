#include <ros/ros.h>
#include <accel_decel/param.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

ros::Publisher pub;
geometry_msgs::PoseStamped pose_msg;

ros::time current_time, last_time, time;

float Vs = 0.0f;
float Vmax = 0.0f;
float Ve = 0.0f;
float Amax = 0.0f;
float Xall = 0.0f

bool cb_flag = false;
bool first;

void param_cb(const accel_decel::param& msg)
{
	if(!cb_flag)
	{
		Vs = msg.Vs;
		Vmax = msg.Vmax;
		Ve = msg.Ve;
		Amax = msg.Amax;
		Xall = msg.Xall;

		cb_flag = true;
	}
	else
	{
		if(end)
		{
			cb_flag = false;
		}
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "accel_decel");
	ros::NodeHandle nh;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	ros::Rate loop_rate(30);

	ros::Subscriber sub = nh.subscribe("/accel_decel", 1000, param_cb);

	pub = nh.advertise<nemcon::motor>("motor", 100);

	while(ros::ok())
	{
		if(first)
		{
			time = 0.0f;
		}
		current_time = ros::Time::now();
		t = (current_time - last_time).toSec();

		last_time = current_time;
		time += current_time;

		ROS_INFO("Vs");

		loop_rate.sleep();
		ros::spinOnce();
	}
}

