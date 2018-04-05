#include <ros/ros.h>
#include <nemcon/switch_in.h>
#include <nemcon/tar_dis.h>
#include <accel_decel/param.h>

#include <pigpiod_if2.h>

static const int pin_blue = 16;
static const int pin_yellow = 12;
static const int pin_servo = 24;

int pi = pigpio_start(0, 0);

void led_flash(int num, float time, int color);	//color：blue = 0, yellow = 1

void switch_cb(const nemcon::switch_in& msg)
{
	if(msg.START && !msg.START && msg.TZ1 && !msg.TZ2 && !msg.TZ3 && !msg.SC)
	{
		led_flash(3, 0.5, 1);
	}
	else
	{
		gpio_write(pi, pin_blue, 0);
		gpio_write(pi, pin_yellow, 0);
		ROS_INFO("NO");
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "servo_test");
	ros::NodeHandle nh;

	set_mode(pi, pin_blue, PI_OUTPUT);
	set_mode(pi, pin_yellow, PI_OUTPUT);
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

void led_flash(int num, float time, int color)
{
	for(int i = 0; i < num; i++)
	{
		if(color == 0)
		{
			gpio_write(pi, pin_blue, 1);
			ros::Duration(time).sleep();
			gpio_write(pi, pin_blue, 0);
			ros::Duration(time).sleep();
		}
		if(color == 0)
		{
			gpio_write(pi, pin_yellow, 1);
			ros::Duration(time).sleep();
			gpio_write(pi, pin_yellow, 0);
			ros::Duration(time).sleep();
		}
	}
}
