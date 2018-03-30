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


static float imu_Kp = 0.20;
static float imu_Ki = 0.90;
static float imu_Kd = 0.05;

static float enc_Kp = 2.00;
static float enc_Ki = 0.00;
static float enc_Kd = 0.00;

float imu_P = 0, imu_I = 0, imu_D = 0, pre_imu_P = 0;
float speedFR = 0, speedRL = 0, speedFL = 0, speedRR = 0;
float turn_imu = 0, turn_enc_x = 0, turn_enc_y = 0;

ros::Time current_time , last_time;
double dt;

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

void imu_cb(const sensor_msgs::Imu& msg)
{
	imu_P = msg.orientation.z - 0.00;
}

void pid_enc(const geometry_msgs::PoseStamped& msg)
{
	/*float lasterror_x = 0, lasterror_y = 0, integral_x = 0, integral_y = 0, error_x = 0, error_y = 0;

	dt = (current_enc_time - last_enc_time).toSec();

	error_x = msg.pose.position.x - 0.0000;
	error_y = msg.pose.position.y - 0.0000;

	integral_x += (error_x + lasterror_x) / 2.0 * dt;
	integral_y += (error_y + lasterror_y) / 2.0 * dt;

	turn_enc_x = enc_P * error_x + enc_I * integral_x + enc_D * (error_x - lasterror_x) / dt;
	turn_enc_y = enc_P * error_y + enc_I * integral_y + enc_D * (error_y - lasterror_y) / dt;

	lasterror_x = error_x;
	lasterror_y = error_y;

	last_enc_time = current_enc_time;

	//printf("enc:%f\n", dt);*/
}

void imu_pid()
{
	imu_I += (imu_P + pre_imu_P) / 2.0 * dt;
	imu_D = (imu_P - pre_imu_P) /dt
	turn_imu = imu_Kp * imu_P + imu_Ki * imu_I + imu_Kd * imu_D;

	pre_imu_P = imu_P;
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
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	ros::Rate loop_rate(10);
	ros::NodeHandle local_nh("~");


	/*if(!local_nh.hasParam(""))*/

	ros::Subscriber sub_imu = nh.subscribe("/imu/data_raw", 1000, imu_cb);
	ros::Subscriber sub_enc = nh.subscribe("/robot/pose", 1000, enc_cb);

	pub = nh.advertise<mpu9250::motor>("motor", 100);



	while(ros::ok())
	{
		signal(SIGINT, mySigintHandler);

		current_time = ros::Time::now();
		dt = (current_imu_time - last_imu_time).toSec();

		imu_pid();


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

		printf("%f\t %f\t %f\t %f\t %f\n",dt, turn_imu, turn_enc_x, speedFR, speedRL);

		pub.publish(msg_m);
		last_time = current_time;
		loop_rate.sleep();
		ros::spinOnce();
	}
}
