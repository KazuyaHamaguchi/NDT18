#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>

#include <mpu9250/motor.h>

#include <stdio.h>
#include <stdlib.h>

using namespace std;

static float P = 8.00;
static float I = 0.00;
static float D = 0.00;

static float delta_t = 0.01;

static float lasterror = 0, integral = 0, error = 0, turn = 0;

ros::Publisher pub;
mpu9250::motor msg_m;

void pid_control(const sensor_msgs::Imu& msg)
{
	static float lasterror = 0, integral = 0, error = 0, turn = 0;
	error = msg.orientation.z - 0.00;

	integral += (error + lasterror) / 2.0 * delta_t;

	turn = P * error + I * integral + D * (error - lasterror) / delta_t;

	lasterror = error;

	printf("%f\t %f\n", msg.orientation.z, turn);

	msg_m.motor_RL = turn;
	pub.publish(msg_m);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pid_control");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/imu/data_raw", 1000, pid_control);

	pub = nh.advertise<mpu9250::motor>("motor", 100);

	ros::spin();

	return 0;
}
