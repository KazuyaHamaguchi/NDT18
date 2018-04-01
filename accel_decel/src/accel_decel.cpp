#include <ros/ros.h>
#include <accel_decel/param.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

ros::Publisher pub;

ros::Time current_time, last_time;

float Vs = 0.0f;
float Vmax = 0.0f;
float Ve = 0.0f;
float Amax = 0.0f;
float Xall = 0.0f;
float t = 0.0f;
float now_t = 0.0f;

bool cb_flag = false;
bool first = false;
bool end = false;

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
		first = true;
		t = 0.0f;

	}
	else
	{
		if(end)
		{
			first = false;
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

	while(ros::ok())
	{
		if(first)
		{
			while(t < 1.0)
			{
				current_time = ros::Time::now();
				t += (current_time - last_time).toSec();
				ROS_INFO("time: %f\t Vs: %f\n", t, Vs);
				last_time = current_time;
			}

			first = false;
			end = true;
			cb_flag = false;

		}

		loop_rate.sleep();
		ros::spinOnce();
	}
}
