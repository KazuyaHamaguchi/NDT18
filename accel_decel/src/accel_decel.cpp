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
float t1 = 0.0f;
float X1 = 0.0f;
float t2 = 0.0f;
float x2 = 0.0f;
float t3 = 0.0f;
float X3 = 0.0f;
float V = 0.0f;

bool cb_flag = false;
bool first = false;
bool end = false;

float accel(float naw_t)
{
	return ((Vmax - Vs) * (1 - cos(((2 * Amax) / (Vmax - Vs)) * naw_t)) / 2) + Vs;
}

void param_cb(const accel_decel::param& msg)
{
	if(!cb_flag)
	{
		Vs = msg.Vs;
		Vmax = msg.Vmax;
		Ve = msg.Ve;
		Amax = msg.Amax;
		Xall = msg.Xall;

		if(Xall > (M_PI * ((Vmax * Vmax) - ((Vs * Vs) - (Ve * Ve) / 2)) / 2 * Amax))
		{
			Vmax = ((2 * Amax) / M_PI) + ((Vs * Vs) + (Ve * Ve));
		}

		t1 = (M_PI * (Vmax - Vs)) / (2 * Amax);
		x1 = (((Vmax * Vmax) - (Vs * Vs)) * M_PI) / (4 * Amax);

		t3 = (M_PI * (Vmax - Ve)) / (2 * Amax);
		x3 = (((Vmax * Vmax) - (Ve * Ve)) * M_PI) / (4 * Amax);

		ROS_INFO("Vs: %f\t Vmax: %f\t Ve: %f\t Amax: %f\t Xall: %f\t t1: %f\t X1: %f\t t3: %f\t X3: %f\n", Vs, Vmax, Ve, Amax, Xall, t1, X1, t3, X3);

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
		current_time = ros::Time::now();

		if(first)
		{
			while(t1 < t)
			{
				t += (current_time - last_time).toSec();

				ROS_INFO("time: %f\t V: %f", t, accel(t));
			}

			first = false;
			end = true;
			cb_flag = false;

		}

		last_time = current_time;
		loop_rate.sleep();
		ros::spinOnce();
	}
}
