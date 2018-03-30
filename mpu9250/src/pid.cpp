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

int speed;
int front;	//前：1，右：2，後：3，左：4


static float imu_P = 20.00;
static float imu_I = 2.00;
static float imu_D = 0.05;

static float enc_P = 2.00;
static float enc_I = 0.00;
static float enc_D = 0.00;


static float delta_t = 0.01;
float speedFR = 0, speedRL = 0, speedFL = 0, speedRR = 0;
float turn_imu = 0, turn_enc_x = 0, turn_enc_y = 0;

ros::Time current_imu_time , last_imu_time, current_enc_time, last_enc_time;

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
	if(1 <= input && input < 3 && min >= 0)
	{
		output = 2;
	}
	if(-3 < input && input <= -1 && min <= 0)
	{
		output = -2;
	}
	return output;
}

void pid_acc(const sensor_msgs::Imu& msg)
{
	float lasterror = 0, integral = 0, error = 0;
	current_imu_time = ros::Time::now();

	double dt = (current_imu_time - last_imu_time).toSec();

	error = msg.orientation.z - 0.0000;
	printf("%f\t %f\n", dt, error);

	integral += (error + lasterror) / 2.0 * dt;

	turn_imu = imu_P * error + imu_I * integral + imu_D * (error - lasterror) / dt;

	lasterror = error;
	last_imu_time = current_imu_time;
}

void pid_enc(const geometry_msgs::PoseStamped& msg)
{
	float lasterror_x = 0, lasterror_y = 0, integral_x = 0, integral_y = 0, error_x = 0, error_y = 0;
	current_enc_time = ros::Time::now();

	double dt = (current_enc_time - last_enc_time).toSec();

	error_x = msg.pose.position.x - 0.0000;
	error_y = msg.pose.position.y - 0.0000;

	integral_x += (error_x + lasterror_x) / 2.0 * dt;
	integral_y += (error_y + lasterror_y) / 2.0 * dt;

	turn_enc_x = enc_P * error_x + enc_I * integral_x + enc_D * (error_x - lasterror_x) / dt;
	turn_enc_y = enc_P * error_y + enc_I * integral_y + enc_D * (error_y - lasterror_y) / dt;

	lasterror_x = error_x;
	lasterror_y = error_y;
	last_enc_time = current_enc_time;
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
	current_imu_time = ros::Time::now();
	last_imu_time = ros::Time::now();
	current_enc_time = ros::Time::now();
	last_enc_time = ros::Time::now();
	ros::Rate loop_rate(20);
	ros::NodeHandle local_nh("~");


	if(!local_nh.hasParam("speed"))
	{
		ROS_INFO("Parameter speed is not defind. Now, it is set default value.");
		local_nh.setParam("speed", 0);
	}

	if(!local_nh.getParam("speed", speed))
	{
		ROS_ERROR("parameter speed is invalid.");
		return -1;
	}
	ROS_INFO("speed: %d", speed);

	if(!local_nh.hasParam("front"))
	{
		ROS_INFO("Parameter front is not defind. Now, it is set default value.");
		local_nh.setParam("front", 10);
	}

	if(!local_nh.getParam("front", front))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("front: %d", front);

	ros::Subscriber sub_imu = nh.subscribe("/imu/data_raw", 1, pid_acc);
	ros::Subscriber sub_enc = nh.subscribe("/robot/pose", 1, pid_enc);

	pub = nh.advertise<mpu9250::motor>("motor", 100);



	while(ros::ok())
	{
		signal(SIGINT, mySigintHandler);
		if(speed == 0)
		{
			speedFR = clamp(nearbyint(speed - turn_imu), -20, 20);
			speedFL = clamp(nearbyint(speed + turn_imu), -20, 20);
			speedRL = clamp(nearbyint(speed + turn_imu), -20, 20);
			speedRR = clamp(nearbyint(speed - turn_imu), -20, 20);
		}
		if(speed > 0)
		{
			switch(front)
			{
				case 1:	//前
					speedFR = clamp(nearbyint( speed - turn_imu - turn_enc_x), 0, 20);
					speedFL = clamp(nearbyint( speed + turn_imu + turn_enc_x), 0, 20);
					speedRL = clamp(nearbyint( speed + turn_imu - turn_enc_x), 0, 20);
					speedRR = clamp(nearbyint( speed - turn_imu + turn_enc_x), 0, 20);
					break;

				case 2:	//右
					speedFR = clamp(nearbyint(-(speed + turn_imu + turn_enc_y )), -20, 0);
					speedFL = clamp(nearbyint( speed + turn_imu - turn_enc_y), 0, 20);
					speedRL = clamp(nearbyint(-(speed - turn_imu + turn_enc_y)), -20, 0);
					speedRR = clamp(nearbyint( speed - turn_imu - turn_enc_y), 0, 20);
					break;

				case 3:	//後
					speedFR = clamp(nearbyint( -(speed + turn_imu - turn_enc_x)), -20, 0);
					speedFL = clamp(nearbyint( -(speed - turn_imu + turn_enc_x)), -20, 0);
					speedRL = clamp(nearbyint( -(speed - turn_imu - turn_enc_x)), -20, 0);
					speedRR = clamp(nearbyint( -(speed + turn_imu + turn_enc_x)), -20, 0);
					break;

				case 4:	//左
					speedFR = clamp(nearbyint( speed - turn_imu + turn_enc_y ), 0, 20);
					speedFL = clamp(nearbyint(-(speed - turn_imu - turn_enc_y)), -20, 0);
					speedRL = clamp(nearbyint( speed + turn_imu + turn_enc_y), 0, 20);
					speedRR = clamp(nearbyint(-(speed + turn_imu - turn_enc_y)), -20, 0);
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

		pub.publish(msg_m);
		loop_rate.sleep();
		ros::spinOnce();
	}
}
