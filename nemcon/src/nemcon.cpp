#include <ros/ros.h>
#include <nemcon/switch_in.h>
#include <nemcon/pid_param.h>
#include <accel_decel/param.h>
#include <accel_decel/result.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <nemcon/lrf_flag.h>
#include <nemcon/TZ_judg.h>

#include <pigpiod_if2.h>

static const int pin_blue = 16;
static const int pin_yellow = 12;
static const int pin_servo = 24;

int pi = pigpio_start(0, 0);
bool cb_flag = false;
bool enc_flag = false;
bool receive_flag = false;
bool end = false;
bool first = false;

int TZ = 0;

float acc_t = 0.0f;

void acc_t_cb(const accel_decel::result& msg);
void receive_cb(const std_msgs::Int8& msg);
void judg_cb(const nemcon::TZ_judg& msg);

void led_flash(int num, float time, int color);	//color：blue = 0, yellow = 1
void acc_move(float Vs, float Vmax, float Ve, float Amax, float Xall, float tar_x, float tar_y, int front); //front：1前 2右 3後 4左

nemcon::pid_param msg_pid_param;
accel_decel::param msg_acc_param;
std_msgs::Int8 msg_throw;
nemcon::lrf_flag msg_lrf;
ros::Publisher pub_tar_dis;
ros::Publisher pub_move_param;
ros::Publisher pub_throw;
ros::Publisher pub_lrf;


void switch_cb(const nemcon::switch_in& msg)
{
	if(msg.START)
	{
		if(msg.SZ && !msg.TZ1 && !msg.TZ2 && !msg.TZ3 && !msg.SC && !cb_flag)
		{
			msg_throw.data = 30;
			pub_throw.publish(msg_throw);

			led_flash(0, 0, 2);
			led_flash(3, 0.1, 0);
			led_flash(-1, 0, 0);

			acc_move(0, 1, 0, 0.5, 1.05, 0, 0, 4);	//SZ横
			ros::Duration(3.632449 + 0.05).sleep();
			acc_move(0, 1, 0, 0.5, 4.8, -1.1, 0, 1);	//TZ1横
			ros::Duration(7.941593 + 0.05).sleep();
			acc_move(0, 1, 0, 0.5, 1, -1.15, 4.5, 4);	//TZ1受け渡しポイント
			ros::Duration(3.544907 + 0.05).sleep();

			msg_throw.data = 40;	//受け取り待機
			pub_throw.publish(msg_throw);

			cb_flag = true;
			end = true;
		}
		if(!msg.SZ && msg.TZ1 && !msg.TZ2 && !msg.TZ3 && !msg.SC && !cb_flag)
		{
			led_flash(3, 0.25, 1);

			cb_flag = true;
		}
	}
	else
	{
		led_flash(0, 0, 0);
		led_flash(-1, 0, 1);
		cb_flag = false;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "nemcon");
	ros::NodeHandle nh;

	set_mode(pi, pin_blue, PI_OUTPUT);
	set_mode(pi, pin_yellow, PI_OUTPUT);
	set_servo_pulsewidth(pi, pin_servo, 1520);	//0度

	ros::Subscriber subSwitch = nh.subscribe("/switch", 1000, switch_cb);
	ros::Subscriber sub_accel = nh.subscribe("/accel_decel/result", 1000, acc_t_cb);
	ros::Subscriber sub_receive = nh.subscribe("Throw_on", 1000, receive_cb);
	ros::Subscriber sub_judg = nh.subscribe("TZ_judg", 1000, judg_cb);

	pub_tar_dis = nh.advertise<nemcon::pid_param>("pid_param", 1000);
	pub_move_param = nh.advertise<accel_decel::param>("accel_decel/param", 1000);
	pub_throw = nh.advertise<std_msgs::Int8>("Throw_on_1", 1000);
	pub_lrf = nh.advertise<nemcon::lrf_flag>("lrf_flag", 1000);

	ros::spin();
}

void led_flash(int num, float time, int color)
{
	if(num > 0)		//点滅
	{
		for(int i = 0; i < num; i++)
		{
			if(color == 0)
			{
				gpio_write(pi, pin_blue, 1);
				ros::Duration(time / 2).sleep();
				gpio_write(pi, pin_blue, 0);
				ros::Duration(time / 2).sleep();
			}
			if(color == 1)
			{
				gpio_write(pi, pin_yellow, 1);
				ros::Duration(time / 2).sleep();
				gpio_write(pi, pin_yellow, 0);
				ros::Duration(time / 2).sleep();
			}
			if(color == 2)
			{
				gpio_write(pi, pin_yellow, 1);
				gpio_write(pi, pin_blue, 1);
				ros::Duration(time / 2).sleep();
				gpio_write(pi, pin_yellow, 0);
				gpio_write(pi, pin_blue, 0);
				ros::Duration(time / 2).sleep();
			}
		}
	}
	if(num == 0)	//消灯
	{
		if(color == 0)
		{
			gpio_write(pi, pin_blue, 0);
			ros::Duration(time).sleep();
		}
		if(color == 1)
		{
			gpio_write(pi, pin_yellow, 0);
			ros::Duration(time).sleep();
		}
		if(color == 2)
		{
			gpio_write(pi, pin_blue, 0);
			gpio_write(pi, pin_yellow, 0);
			ros::Duration(time).sleep();
		}
	}
	if(num == -1)	//点灯
	{
		if(color == 0)
		{
			gpio_write(pi, pin_blue, 1);
			ros::Duration(time).sleep();
		}
		if(color == 1)
		{
			gpio_write(pi, pin_yellow, 1);
			ros::Duration(time).sleep();
		}
		if(color == 2)
		{
			gpio_write(pi, pin_blue, 1);
			gpio_write(pi, pin_yellow, 1);
			ros::Duration(time).sleep();
		}
	}
}


void acc_move(float Vs, float Vmax, float Ve, float Amax, float Xall, float tar_x, float tar_y, int front)
{
	msg_acc_param.Vs = Vs;
	msg_acc_param.Vmax = Vmax;
	msg_acc_param.Ve = Ve;
	msg_acc_param.Amax = Amax;
	msg_acc_param.Xall = Xall;
	msg_pid_param.tar_x = tar_x;
	msg_pid_param.tar_y = tar_y;
 	msg_pid_param.front = front;
 	msg_pid_param.pattern = 0;

	pub_move_param.publish(msg_acc_param);
	pub_tar_dis.publish(msg_pid_param);
}

void receive_cb(const std_msgs::Int8& msg)
{
	if(msg.data == -40)	//CRからの受け取りに成功
	{
		msg_throw.data = 50;
		pub_throw.publish(msg_throw);
	}
	if(msg.data == -10)
	{
	}
	else;
}
void judg_cb(const nemcon::TZ_judg& msg)
{
	if(msg.leave)	//1回目にCRが離れた
	{
		msg_throw.data = 10;
		pub_throw.publish(msg_throw);
		if(!first)	//1回だけTZ1
		{
			acc_move(0, 1, 0, 0.5, 1.3, -1.15, 4.4, 4);
			ros::Duration(4.194392 + 0.05).sleep();
			msg_lrf.flag = true;
			msg_lrf.TZ = 1;
			pub_lrf.publish(msg_lrf);
			first = true;
		}
		else
		{
			msg_throw.data = 52;
			pub_throw.publish(msg_throw);
		}
	}
	if(msg.TZ1)	//TZ1と判断
	{
		TZ = 1;
		msg_throw.data = 51;
		pub_throw.publish(msg_throw);
	}
	if(msg.TZ2)	//TZ2と判断
	{
		TZ = 2;
		msg_throw.data = 51;
		pub_throw.publish(msg_throw);
	}
	if(msg.TZ3)	//TZ3と判断
	{
		TZ = 3;
		msg_throw.data = 51;
		pub_throw.publish(msg_throw);
	}
	if(msg.leave2 && TZ == 1)	//2回目にCRが離れてTZ1だった時
	{
		acc_move(0, 1, 0, 0.5, 1.3, -1.15, 4.4, 4);
		ros::Duration(4.194392 + 0.05).sleep();
		msg_lrf.flag = true;
		msg_lrf.TZ = 2;
		pub_lrf.publish(msg_lrf);

	}
	if(msg.leave2 && TZ == 2)	//2回目にCRが離れてTZ2だった時
	{
		acc_move(0, 1, 0, 0.5, 1.3, -1.15, 4.4, 4);
		ros::Duration(4.194392 + 0.05).sleep();
		msg_lrf.flag = true;
		msg_lrf.TZ = 2;
		pub_lrf.publish(msg_lrf);
	}
	if(msg.leave2 && TZ == 3)	//2回目にCRが離れてTZ3だった時
	{
		acc_move(0, 1, 0, 0.5, 1.3, -1.15, 4.4, 4);
		ros::Duration(4.194392 + 0.05).sleep();
		msg_lrf.flag = true;
		msg_lrf.TZ = 3;
		pub_lrf.publish(msg_lrf);
	}
}
void acc_t_cb(const accel_decel::result& msg)
{
	acc_t = msg.t;
}
