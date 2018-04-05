#include <ros/ros.h>
#include <nemcon/switch_in.h>

#include <pigpiod_if2.h>

static const int pin_blue = 16;
static const int pin_orange = 12;
static const int pin_servo = 24;

int pi = pigpio_start(0, 0);

void switch_cb(const nemcon::switch_in& msg)
{
	if(msg.START && msg.TZ1 && msg.TZ2 && msg.TZ3 && !msg.SC)
	{
		gpio_write(pi, pin_blue, 1);
		gpio_write(pi, pin_orange, 1);
		ROS_INFO("OK");
	}
	else
	{
		gpio_write(pi, pin_blue, 0);
		gpio_write(pi, pin_orange, 0);
		ROS_INFO("NO");
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "servo_test");
	ros::NodeHandle nh;

	set_mode(pi, pin_blue, PI_OUTPUT);
	set_mode(pi, pin_orange, PI_OUTPUT);
	set_servo_pulsewidth(pi, pin_servo, 1520);	//0度

	ros::Subscriber subSwitch = nh.subscribe("/switch", 1000, switch_cb);

	/*ros::Rate loop_rate(1);

	while(ros::ok())
	{
		set_servo_pulsewidth(pi, pin_servo, 2450);
		time_sleep(1);
		set_servo_pulsewidth(pi, pin_servo, 650);
		time_sleep(1);
	}
	return 0;*/
	ros::spin();
}
