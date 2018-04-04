#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nemcon/motor.h>

#include <pigpiod_if2.h>

int pi = pigpio_start(0, 0);

ros::Publisher pub;
mpu9250::motor msg_m;

#define e_pin 22
bool motor = false;
bool accel = false;
bool flag = false;

void callback_m(const nemcon::motor& msg)
{
	if(msg.motor_FR == 0 && msg.motor_FL == 0 && msg.motor_RL == 0 && msg.motor_RR == 0)
	{
		ROS_INFO("M_OK");
		motor = true;
	}
	else
	{
		motor = false;
	}
}

void callback_a(const sensor_msgs::Imu& msg)
{
	if(msg.linear_acceleration.x > 2 || msg.linear_acceleration.x < -2 || msg.linear_acceleration.y > 2 || msg.linear_acceleration.y < -2)
	{
		ROS_INFO("A_OK");
		accel = true;
	}
	else
	{
		accel = false;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "emergency_stop");
	ros::NodeHandle nh;
	ros::Subscriber sub_motor = nh.subscribe("motor", 1000, callback_m);
	ros::Subscriber sub_imu = nh.subscribe("/imu/data_raw", 1000, callback_a);
	pub = nh.advertise<nemcon::motor>("motor", 1000);
	set_mode(pi, e_pin, PI_OUTPUT);

	while(ros::ok())
	{
		if(motor && accel)
		{
			if(!flag)
			{
				flag = true;

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
			if(flag)
			{
				flag = false;
				gpio_write(pi, e_pin, 0);
				ROS_INFO("NG");
			}
			else;
		}
		ros::spinOnce();
	}
	return 0;
}
