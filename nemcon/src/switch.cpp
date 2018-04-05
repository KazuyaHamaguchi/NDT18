#include <ros/ros.h>
#include <nemcon/switch.h>

#include <pigpiod_if2.h>

static const int pin_START = 29;
static const int pin_SZ = 35;
static const int pin_TZ1 = 37;
static const int pin_TZ2 = 29;
static const int pin_TZ3 = 29;
static const int pin_SC = 29;

bool flag_START = false;
bool flag_SZ = false;
bool flag_TZ1 = false;
bool flag_TZ2 = false;
bool flag_TZ3 = false;
bool flag_SC = false;

nemcon::switch msg;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "switch");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<nemcon::switch>("switch", 1000);

	int pi = pigpio_start(0, 0);
	set_mode(pi, pin_START, PI_INPUT);
	set_mode(pi, pin_SZ, PI_INPUT);
	set_mode(pi, pin_TZ1, PI_INPUT);
	set_mode(pi, pin_TZ2, PI_INPUT);
	set_mode(pi, pin_TZ3, PI_INPUT);
	set_mode(pi, pin_SC, PI_INPUT);

	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		if(gpio_read(pi, pin_START) == 1 && !flag_START)
		{
			flag_START = true;
			msg.START = true;
			pub.publish(msg);
		}
		else
		{
			msg.START = false;
			pub.publish(msg);
			if(flag_START)
			{
				flag_START = false;
			}
		}

		if(gpio_read(pi, pin_SZ) == 1 && !flag_SZ)
		{
			flag_SZ = true;
			msg.SZ = true;
			pub.publish(msg);
		}
		else
		{
			msg.SZ = false;
			pub.publish(msg);
			if(flag_SZ)
			{
				flag_SZ = false;
			}
		}

		if(gpio_read(pi, pin_TZ1) == 1 && !flag_TZ1)
		{
			flag_TZ1 = true;
			msg.TZ1 = true;
			pub.publish(msg);
		}
		else
		{
			msg.TZ1 = false;
			pub.publish(msg);
			if(flag_TZ1)
			{
				flag_TZ1 = false;
			}
		}

		if(gpio_read(pi, pin_TZ2) == 1 && !flag_TZ2)
		{
			flag_TZ2 = true;
			msg.TZ2 = true;
			pub.publish(msg);
		}
		else
		{
			msg.TZ2 = false;
			pub.publish(msg);
			if(flag_TZ2)
			{
				flag_TZ2 = false;
			}
		}

		if(gpio_read(pi, pin_TZ3) == 1 && !flag_TZ3)
		{
			flag_TZ3 = true;
			msg.TZ3 = true;
			pub.publish(msg);
		}
		else
		{
			msg.TZ3 = false;
			pub.publish(msg);
			if(flag_TZ3)
			{
				flag_TZ3 = false;
			}
		}

		if(gpio_read(pi, pin_SC) == 1 && !flag_SC)
		{
			flag_SC = true;
			msg.SC = true;
			pub.publish(msg);
		}
		else
		{
			msg.SC = false;
			pub.publish(msg);
			if(flag_SC)
			{
				flag_SC = false;
			}
		}

		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
