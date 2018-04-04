#include <ros/ros.h>

#include <pigpiod_if2.h>

int pi = pigpio_start(0, 0);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "servo_test");
	ros::NodeHandle nh;

	set_mode(pi, 24, PI_OUTPUT);
	ros::Rate loop_rate(1);

	while(ros::ok())
	{
		/*set_PWM_frequency(pi, 24, 400);
		set_PWM_range(pi, 24, 0);

		set_PWM_dutycycle(pi, 24, 500);*/


		set_servo_pulsewidth(pi, 24, 0);
		set_servo_pulsewidth(pi, 24, 1500);
		/*ros::Duration(2).sleep();
		set_servo_pulsewidth(pi, 24, 2000);
		ros::Duration(2).sleep();*/
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
