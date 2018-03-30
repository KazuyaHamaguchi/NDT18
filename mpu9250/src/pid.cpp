#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include <signal.h>

#include <mpu9250/motor.h>
#include <deadreckoning/enc.h>

#include <stdio.h>
#include <stdlib.h>
#include <cmath>	// http://faithandbrave.hateblo.jp/entry/2016/12/12/181115

using namespace std;

static float speed = 10;
int front = 1;	//前：1，右：2，後：3，左：4


static float acc_P = 2.00;
static float acc_I = 0.00;
static float acc_D = 0.00;

static float enc_P = 2.00;
static float enc_I = 0.00;
static float enc_D = 0.00;


static float delta_t = 0.01;
float speedFR = 0, speedRL = 0, speedFL = 0, speedRR = 0;
float turn_acc = 0, turn_enc_x = 0, turn_enc_y = 0;

ros::Publisher pub;
mpu9250::motor msg_m;
sensor_msgs::Imu msg_acc;

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

void pid_acc(const sensor_msgs::Imu& msg)
{
	float lasterror = 0, integral = 0, error = 0;
	error = msg.orientation.z - 0.0000;

	integral += (error + lasterror) / 2.0 * delta_t;

	turn_acc = acc_P * error + acc_I * integral + acc_D * (error - lasterror) / delta_t;

	lasterror = error;
}

void pid_enc(const geometry_msgs::PoseStamped& msg)
{
	float lasterror_x = 0, lasterror_y = 0, integral_x = 0, integral_y = 0, error_x = 0, error_y = 0;

	error_x = msg.pose.position.x - 0.0000;
	error_y = msg.pose.position.y - 0.0000;

	integral_x += (error_x + lasterror_x) / 2.0 * delta_t;
	integral_y += (error_y + lasterror_y) / 2.0 * delta_t;

	turn_enc_x = enc_P * error_x + enc_I * integral_x + enc_D * (error_x - lasterror_x) / delta_t;
	turn_enc_y = enc_P * error_y + enc_I * integral_y + enc_D * (error_y - lasterror_y) / delta_t;

	lasterror_x = error_x;
	lasterror_y = error_y;
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
	ros::Rate loop_rate(10);
	ros::NodeHandle local_nh("~");


	/*if(!local_nh.hasParam(""))*/

	ros::Subscriber sub_imu = nh.subscribe("/imu/data_raw", 1000, pid_acc);
	ros::Subscriber sub_enc = nh.subscribe("/robot/pose", 1000, pid_enc);

	pub = nh.advertise<mpu9250::motor>("motor", 100);



	while(ros::ok())
	{
		signal(SIGINT, mySigintHandler);
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
					speedFR = clamp(nearbyint( speed - turn_acc - turn_enc_x), 0, 20);
					speedFL = clamp(nearbyint( speed + turn_acc + turn_enc_x), 0, 20);
					speedRL = clamp(nearbyint( speed + turn_acc - turn_enc_x), 0, 20);
					speedRR = clamp(nearbyint( speed - turn_acc + turn_enc_x), 0, 20);
					break;

				case 2:	//右
					speedFR = clamp(nearbyint(-(speed + turn_acc)), -20, 0);
					speedFL = clamp(nearbyint( speed + turn_acc), 0, 20);
					speedRL = clamp(nearbyint(-(speed - turn_acc)), -20, 0);
					speedRR = clamp(nearbyint( speed - turn_acc), 0, 20);
					break;

				case 3:	//後
					speedFR = clamp(nearbyint( -(speed + turn_acc + turn_enc_x)), -20, 0);
					speedFL = clamp(nearbyint( -(speed - turn_acc - turn_enc_x)), -20, 0);
					speedRL = clamp(nearbyint( -(speed - turn_acc + turn_enc_x)), -20, 0);
					speedRR = clamp(nearbyint( -(speed + turn_acc - turn_enc_x)), -20, 0);
					break;

				case 4:	//左
					speedFR = clamp(nearbyint( speed - turn_acc - turn_enc_y ), 0, 20);
					speedFL = clamp(nearbyint(-(speed - turn_acc + turn_enc_y)), -20, 0);
					speedRL = clamp(nearbyint( speed + turn_acc - turn_enc_y), 0, 20);
					speedRR = clamp(nearbyint(-(speed + turn_acc + turn_enc_y)), -20, 0);
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

		printf("%f\t %f\t %f\t %f\n", msg_acc.orientation.z, turn_acc, speedFR, speedRL);

		pub.publish(msg_m);
		loop_rate.sleep();
		ros::spinOnce();
	}
}
