#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <signal.h>

#include <mpu9250/motor.h>

#include <stdio.h>
#include <stdlib.h>
#include <cmath>	// http://faithandbrave.hateblo.jp/entry/2016/12/12/181115

using namespace std;

static float speed = 10;
int front = 1;	//前：1，右：2，後：3，左：4


static float P = 20.00;
static float I = 2.00;
static float D = 0.05;

static float delta_t = 0.01;
float speedFR = 0, speedRL = 0, speedFL = 0, speedRR = 0;
float turn_acc = 0;

ros::Publisher pub;
mpu9250::motor msg_m;

float clamp(float input, float min, float max)
{
	float output = 0.00000f;
	if(input <= min)
	{
		output = min;
	}
	if(min < input && input < max)
	{
		output = input;
	}
	if(input >= max)
	{
		output = max;
	}
	if(1 <= input && input < 3)
	{
		output = 2;
	}
	if(-3 < input && input <= -1)
	{
		output = -2;
	}
	return_acc output;
}

void pid_acc(const sensor_msgs::Imu& msg)
{
	float lasterror = 0, integral = 0, error = 0;
	error = msg.orientation.z - 0.0000;

	integral += (error + lasterror) / 2.0 * delta_t;

	turn_acc = P * error + I * integral + D * (error - lasterror) / delta_t;

	lasterror = error;
}

void pid_enc(const sensor_msgs::Imu& msg)
{
	float lasterror = 0, integral = 0, error = 0;
	error =  - 0.0000;

	integral += (error + lasterror) / 2.0 * delta_t;

	turn_acc = P * error + I * integral + D * (error - lasterror) / delta_t;

	lasterror = error;
}


void mySigintHandler(int sig)
{
	msg_m.motor_FR = 0;
	msg_m.motor_FL = 0;
	msg_m.motor_RR = 0;
	msg_m.motor_RL = 0;
	pub.publish(msg_m);
	ros::shutdown();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pid_control", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;
	ros::NodeHandle local_nh("~");

	/*if(!local_nh.hasParam(""))*/

	ros::Subscriber sub_imu = nh.subscribe("/imu/data_raw", 1000, pid_acc);
	ros::Subscriber sub_enc = nh.subscribe("/enc", 1000, pid_enc);

	pub = nh.advertise<mpu9250::motor>("motor", 1);

	signal(SIGINT, mySigintHandler);

	while(ros::ok())
	{
		if(speed == 0)
		{
			speedFR = clamp(nearbyint(speed - turn_acc), -20, 20);
			speedFL = clamp(nearbyint(speed + turn_acc), -20, 20);
			speedRL = clamp(nearbyint(speed + turn_acc), -20, 20);
			speedRR = clamp(nearbyint(speed - turn_acc), -20, 20);
		}
		if(speed > 0)
		{
			switch(front)
			{
				case 1:	//前
					speedFR = clamp(nearbyint( speed - turn_acc), 0, 20);
					speedFL = clamp(nearbyint( speed + turn_acc), 0, 20);
					speedRL = clamp(nearbyint( speed + turn_acc), 0, 20);
					speedRR = clamp(nearbyint( speed - turn_acc), 0, 20);
					break;

				case 2:	//右
					speedFR = clamp(nearbyint(-(speed + turn_acc)), -20, 0);
					speedFL = clamp(nearbyint( speed + turn_acc), 0, 20);
					speedRL = clamp(nearbyint(-(speed - turn_acc)), -20, 0);
					speedRR = clamp(nearbyint( speed - turn_acc), 0, 20);
					break;

				case 3:	//後
					speedFR = clamp(nearbyint( -(speed + turn_acc)), -20, 0);
					speedFL = clamp(nearbyint( -(speed - turn_acc + 4)), -20, 0);
					speedRL = clamp(nearbyint( -(speed - turn_acc + 4)), -20, 0);
					speedRR = clamp(nearbyint( -(speed + turn_acc)), -20, 0);
					break;

				case 4:	//左
					speedFR = clamp(nearbyint( speed - turn_acc + 2 ), 0, 20);
					speedFL = clamp(nearbyint(-(speed - turn_acc)), -20, 0);
					speedRL = clamp(nearbyint( speed + turn_acc + 2), 0, 20);
					speedRR = clamp(nearbyint(-(speed + turn_acc)), -20, 0);
					break;

				default:
					speedFR = 0;
					speedFL = 0;
					speedRL = 0;
					speedRR = 0;
					break;
			}
		}

		msg_m.motor_FR = speedFR;
		msg_m.motor_FL = speedFL;
		msg_m.motor_RR = speedRR;
		msg_m.motor_RL = speedRL;

		printf("%f\t %f\t %f\t %f\n", sub_imu.orientation.z, turn_acc, speedFR, speedRL);

		pub.publish(msg_m);
		ros::spinOnce();
	}
}
