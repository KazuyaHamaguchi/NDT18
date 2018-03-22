#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <mpu9250/motor.h>

#include <pigpiod_if2.h>

int pi = pigpio_start(0, 0);

ros::Publisher pub;
mpu9250::motor msg_m;

#define e_pin 15;
int motor, accel;
int flag = 0;

void callback_m(const mpu9250::motor& msg)
{
	if(msg.motor_FR == 0 && msg.motor_FL == 0 && msg.motor_RL == 0 && msg.motor_RR == 0)
	{
		ROS_INFO("M_OK");
		motor = 1;
	}
	else
	{
		motor = 0;
	}
}

void callback_a(const sensor_msgs::Imu& msg)
{
	if(msg.linear_acceleration.x > 2 || msg.linear_acceleration.x < -2 || msg.linear_acceleration.y > 2 || msg.linear_acceleration.y < -2)
	{
		ROS_INFO("A_OK");
		accel = 1;
	}
	else
	{
		accel = 0;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "emergency_stop");
	ros::NodeHandle nh;
	ros::Subscriber sub_motor = nh.subscribe("motor", 1000, callback_m);
	ros::Subscriber sub_imu = nh.subscribe("/imu/data_raw", 1000, callback_a);
	pub = nh.advertise<mpu9250::motor>("motor", 1000);
	set_mode(pi, e_pin, PI_OUTPUT);

	while(ros::ok())
	{
		if(motor == 1 && accel == 1)
		{
			if(flag == 0)
			{
				flag = 1;

				gpio_write(pi, e_pin, 1);

				msg_m.motor_FR = 0;
				msg_m.motor_FL = 0;
				msg_m.motor_RR = 0;
				msg_m.motor_RL = 0;

				pub.publish(msg_m);

				ROS_INFO("OK");
			}
			else;
		}
		else
		{
			if(flag == 1)
			{
				flag = 0;
				gpio_write(pi, e_pin, 0);
				ROS_INFO("NG");
			}
			else;
		}
		ros::spinOnce();
	}
	return 0;
}
