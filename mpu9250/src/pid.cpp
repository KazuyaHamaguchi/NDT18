#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <signal.h>

#include <mpu9250/motor.h>

#include <stdio.h>
#include <stdlib.h>
#include <cmath>	// http://faithandbrave.hateblo.jp/entry/2016/12/12/181115

using namespace std;

static float speed = 0;
int front = 2;	//前：1，右：2，後：3，左：4


static float P = 20.00;
static float I = 2.00;
static float D = 0.05;

static float delta_t = 0.01;

static float lasterror = 0, integral = 0, error = 0, turn = 0;
float speedFR = 0, speedRL = 0, speedFL = 0, speedRR = 0;

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
	return output;
}

void pid_control(const sensor_msgs::Imu& msg)
{
	static float lasterror = 0, integral = 0, error = 0, turn = 0;
	error = msg.orientation.z - 0.0000;

	integral += (error + lasterror) / 2.0 * delta_t;

	turn = P * error + I * integral + D * (error - lasterror) / delta_t;

	lasterror = error;
	if(speed == 0)
	{
		speedFR = clamp(nearbyint(speed - turn), -20, 20);
		speedFL = clamp(nearbyint(speed + turn), -20, 20);
		speedRL = clamp(nearbyint(speed + turn), -20, 20);
		speedRR = clamp(nearbyint(speed - turn), -20, 20);
	}
	if(speed > 0)
	{
		switch(front)
		{
			case 1:	//前
				speedFR = clamp(nearbyint( speed - turn), 0, 20);
				speedFL = clamp(nearbyint( speed + turn), 0, 20);
				speedRL = clamp(nearbyint( speed + turn), 0, 20);
				speedRR = clamp(nearbyint( speed - turn), 0, 20);
				break;

			case 2:	//右
				speedFR = clamp(nearbyint(-(speed + turn)), -20, 0);
				speedFL = clamp(nearbyint( speed + turn), 0, 20);
				speedRL = clamp(nearbyint(-(speed - turn)), -20, 0);
				speedRR = clamp(nearbyint( speed - turn), 0, 20);
				break;

			case 3:	//後
				speedFR = clamp(nearbyint( -(speed + turn)), -20, 0);
				speedFL = clamp(nearbyint( -(speed - turn + 4)), -20, 0);
				speedRL = clamp(nearbyint( -(speed - turn + 4)), -20, 0);
				speedRR = clamp(nearbyint( -(speed + turn)), -20, 0);
				break;

			case 4:	//左
				speedFR = clamp(nearbyint( speed - turn + 2 ), 0, 20);
				speedFL = clamp(nearbyint(-(speed - turn)), -20, 0);
				speedRL = clamp(nearbyint( speed + turn + 2), 0, 20);
				speedRR = clamp(nearbyint(-(speed + turn)), -20, 0);
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

	printf("%f\t %f\t %f\t %f\n", msg.orientation.z, turn, speedFR, speedRL);

	pub.publish(msg_m);
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

	ros::Subscriber sub = nh.subscribe("/imu/data_raw", 1000, pid_control);

	pub = nh.advertise<mpu9250::motor>("motor", 1);

	signal(SIGINT, mySigintHandler);

	ros::spin();
}
