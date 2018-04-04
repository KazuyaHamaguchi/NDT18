#include <ros/ros.h>

#include <pigpiod_if2.h>

int pi = pigpio_start(0, 0);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "servo_test");
	ros::NodeHandle nh;

	gpioSetMode(25, PI_OUTPUT);

	while(ros::ok())
	{
		set_servo_pulsewidth(pi, 25, 0);
		/*ros::Duration(2).sleep();
		set_servo_pulsewidth(pi, 25, 2000);
		ros::Duration(2).sleep();*/
		ros::spinOnce();
	}
	return 0;
}
