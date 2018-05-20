#include <ros/ros.h>
#include <nemcon/switch_in.h>

#include <pigpiod_if2.h>

static const int pin_START = 5;
static const int pin_SZ = 19;
static const int pin_TZ1 = 26;
static const int pin_TZ2 = 21;
static const int pin_TZ3 = 20;
static const int pin_SC = 13;

bool flag_START = false;
bool flag_SZ = false;
bool flag_TZ1 = false;
bool flag_TZ2 = false;
bool flag_TZ3 = false;
bool flag_SC = false;

int pin_START_count = 0;

nemcon::switch_in msg;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "switch");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<nemcon::switch_in>("switch", 1000);

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
		/*if(gpio_read(pi, pin_START) == 1)
		{
			if(!flag_START)
			{
				flag_START = true;
				pin_START_count ++;
			}
		}
		else
		{
			if(flag_START)
			{
				flag_START = false;
			}
		}
		if(pin_START_count % 2 == 1)
		{
			msg.START = true;
			pub.publish(msg);
		}
		else
		{
			msg.START = false;
			pub.publish(msg);
		}

		if(gpio_read(pi, pin_SZ) == 1)
		{
			if(!flag_SZ)
			{
				flag_SZ = true;
				msg.SZ = true;
				pub.publish(msg);
			}
		}
		else
		{
			if(flag_SZ)
			{
				msg.SZ = false;
				pub.publish(msg);
				flag_SZ = false;
			}
		}*/

		if(gpio_read(pi, pin_START) == 1)
		{
			if(!flag_START)
			{
				flag_START = true;
				msg.START = true;
				pub.publish(msg);
			}
		}
		else
		{
			if(flag_START)
			{
				msg.START = false;
				pub.publish(msg);
				flag_START = false;
			}
		}

		if(gpio_read(pi, pin_TZ1) == 1)
		{
			if(!flag_TZ1)
			{
				flag_TZ1 = true;
				msg.TZ1 = true;
				pub.publish(msg);
			}
		}
		else
		{
			if(flag_TZ1)
			{
				msg.TZ1 = false;
				pub.publish(msg);
				flag_TZ1 = false;
			}
		}

		if(gpio_read(pi, pin_TZ2) == 1)
		{
			if(!flag_TZ2)
			{
				flag_TZ2 = true;
				msg.TZ2 = true;
				pub.publish(msg);
			}
		}
		else
		{
			if(flag_TZ2)
			{
				msg.TZ2 = false;
				pub.publish(msg);
				flag_TZ2 = false;
			}
		}

		if(gpio_read(pi, pin_TZ3) == 1)
		{
			if(!flag_TZ3)
			{
				flag_TZ3 = true;
				msg.TZ3 = true;
				pub.publish(msg);
			}
		}
		else
		{
			if(flag_TZ3)
			{
				msg.TZ3 = false;
				pub.publish(msg);
				flag_TZ3 = false;
			}
		}

		if(gpio_read(pi, pin_SC) == 1)
		{
			if(!flag_SC)
			{
				flag_SC = true;
				msg.SC = true;
				pub.publish(msg);
			}
		}
		else
		{
			if(flag_SC)
			{
				msg.SC = false;
				pub.publish(msg);
				flag_SC = false;
			}
		}

		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
