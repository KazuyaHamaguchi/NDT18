#include <ros/ros.h>
#include <nemcon/switch_in.h>
#include <nemcon/pid_param.h>
#include <accel_decel/param.h>

#include <pigpiod_if2.h>

static const int pin_blue = 16;
static const int pin_yellow = 12;
static const int pin_servo = 24;

int pi = pigpio_start(0, 0);
bool cb_flag = false;
bool end = false;

void led_flash(int num, float time, int color);	//color：blue = 0, yellow = 1
void movement(float Vs, float Vmax, float Ve, float Amax, float Xall, float tar_x, float tar_y, int front); //front：1前 2右 3後 4左

nemcon::pid_param msg_pid_param;
accel_decel::param msg_acc_param;
ros::Publisher pub_tar_dis;
ros::Publisher pub_move_param;


void switch_cb(const nemcon::switch_in& msg)
{
	if(msg.START && !msg.SZ && !msg.TZ1 && !msg.TZ2 && !msg.TZ3 && !msg.SC && !cb_flag)
	{
		led_flash(3, 0.5, 0);
		led_flash(0, 0, 0);

		movement(0, 1, 0, 0.5, 0.56, 0, 0, 4);
		ROS_INFO("OK");
		led_flash(0, 0, 1);
		ros::Duration(2).sleep();

		cb_flag = true;
		end = true;

	}
	else
	{
		led_flash(5, 0.25, 1);
		led_flash(0, 0, 1);
		if(end)
		{
			//cb_flag = false;
			ROS_INFO("NG");
		}
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

	pub_tar_dis = nh.advertise<nemcon::pid_param>("pid_param", 1000);
	pub_move_param = nh.advertise<accel_decel::param>("accel_decel/param", 1000);

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
	if(num > 0)
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
			if(color == 1)
			{
				gpio_write(pi, pin_yellow, 1);
				ros::Duration(time).sleep();
				gpio_write(pi, pin_yellow, 0);
				ros::Duration(time).sleep();
			}
		}
	}
	if(num == 0)
	{
		gpio_write(pi, pin_blue, 1);
	}
}


void movement(float Vs, float Vmax, float Ve, float Amax, float Xall, float tar_x, float tar_y, int front)
{
	msg_acc_param.Vs = Vs;
	msg_acc_param.Vmax = Vmax;
	msg_acc_param.Ve = Ve;
	msg_acc_param.Amax = Amax;
	msg_acc_param.Xall = Xall;
	msg_pid_param.tar_x = tar_x;
	msg_pid_param.tar_y = tar_y;
 	msg_pid_param.front = front;

	pub_move_param.publish(msg_acc_param);
	pub_tar_dis.publish(msg_pid_param);
}
