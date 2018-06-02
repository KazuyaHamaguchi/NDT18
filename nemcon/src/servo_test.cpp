#include <ros/ros.h>

#include <pigpiod_if2.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "servo_test");
	ros::NodeHandle nh;
	int pi = pigpio_start(0, 0);
	set_mode(pi, 24, PI_OUTPUT);
	set_servo_pulsewidth(pi, 24, 1520);	//0度


	while(ros::ok())
	{
		set_servo_pulsewidth(pi, 24, 1520);
		ros::Duration(1).sleep();
		set_servo_pulsewidth(pi, 24, 1300/*920*/);
		ros::Duration(1).sleep();
	}
	return 0;
}
