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
static const int pin_RESET = 6;

int pi = pigpio_start(0, 0);
bool cb_flag = false;
bool enc_flag = false;
bool end = false;
bool first = false;
bool lrf = false;
bool throw_on = false;
bool TZ_3 = false;

bool flag_RESET = false;

int TZ = 0;
int pre_TZ = 0;

float acc_t = 0.0f;

void switch_cb(const nemcon::switch_in& msg);
void receive_cb(const std_msgs::Int8& msg);
void judg_cb(const std_msgs::Int8& msg);
void lrf_cb(const std_msgs::Int8& msg);
void Reset(pi, pin_RESET, unsigned level, uint32_t tick);

void led_flash(int num, float time, int color);	//color：blue = 0, yellow = 1
void acc_move(float Vs, float Vmax, float Ve, float Amax, float Xall, float tar_x, float tar_y, int front); //front：1前 2右 3後 4左

nemcon::pid_param msg_pid_param;
accel_decel::param msg_acc_param;
std_msgs::Int8 msg_throw;
nemcon::lrf_flag msg_lrf;
std_msgs::Int8 msg_judg;
ros::Publisher pub_tar_dis;
ros::Publisher pub_move_param;
ros::Publisher pub_throw;
ros::Publisher pub_lrf;
ros::Publisher pub_judg;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "nemcon");
	ros::NodeHandle nh;

	set_mode(pi, pin_blue, PI_OUTPUT);
	set_mode(pi, pin_yellow, PI_OUTPUT);
	set_servo_pulsewidth(pi, pin_servo, 1520);	//0度
	callback(pi, pin_RESET, 2, Reset);

	ros::Subscriber subSwitch = nh.subscribe("/switch", 1000, switch_cb);
	ros::Subscriber sub_receive = nh.subscribe("Throw_on", 1000, receive_cb);
	ros::Subscriber sub_judg = nh.subscribe("TZ_judg", 1000, judg_cb);
	ros::Subscriber sub_lrf = nh.subscribe("lrf", 1000, lrf_cb);

	pub_tar_dis = nh.advertise<nemcon::pid_param>("pid_param", 1000);
	pub_move_param = nh.advertise<accel_decel::param>("accel_decel/param", 1000);
	pub_throw = nh.advertise<std_msgs::Int8>("Throw_on_1", 1000);
	pub_lrf = nh.advertise<nemcon::lrf_flag>("lrf_flag", 1000);
	pub_judg = nh.advertise<std_msgs::Int8>("judg_call", 1000);

	ros::spin();
}


void switch_cb(const nemcon::switch_in& msg)
{
	if(msg.START)
	{
		if(msg.SZ && !msg.TZ1 && !msg.TZ2 && !msg.TZ3 && !msg.SC && !cb_flag)
		{
			msg_throw.data = 3;
			pub_throw.publish(msg_throw);
			msg_throw.data = 43;
			pub_throw.publish(msg_throw);
			msg_throw.data = 4;
			pub_throw.publish(msg_throw);

			led_flash(0, 0, 2);
			led_flash(3, 0.1, 0);
			led_flash(-1, 0, 0);

			//acc_move(0, 1, 0, 0.5, 1.05, 0, 0, 4);	//SZ横
			//ros::Duration(3.632449 + 0.05).sleep();
			acc_move(0, 3, 0, 2, 4.5, -1.15, 0, 1);	//TZ1横
			ros::Duration(3.759942 + 0.1).sleep();
			msg_throw.data = 30;
			pub_throw.publish(msg_throw);
			acc_move(0, 3, 0, 2, 0.95, -1.15, 4.42, 4);	//TZ1受け渡しポイント
			ros::Duration(1.772454 + 0.1).sleep();

			msg_lrf.flag = true;
			msg_lrf.type = 1;
			pub_lrf.publish(msg_lrf);

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

void lrf_cb(const std_msgs::Int8& msg)
{
	if(msg.data == -50)
	{
		msg_pid_param.pattern = 3;
		pub_tar_dis.publish(msg_pid_param);
		msg_lrf.flag = false;
		pub_lrf.publish(msg_lrf);
	}
}

void receive_cb(const std_msgs::Int8& msg)
{
	if(msg.data == -40)	//CRからの受け取りに成功
	{
		if(!first)
		{
			msg_judg.data = 50;
			pub_judg.publish(msg_judg);
		}
		else
		{
			msg_judg.data = 52;
			pub_judg.publish(msg_judg);
		}
	}

	if(msg.data == -41)
	{
		set_servo_pulsewidth(pi, pin_servo, 950);	//90度
		msg_lrf.flag = false;
		pub_lrf.publish(msg_lrf);
		ros::Duration(1).sleep();
		if(TZ == 1)
		{
			msg_throw.data = 1;
			pub_throw.publish(msg_throw);
		}
		if(TZ == 2)
		{
			msg_throw.data = 11;
			pub_throw.publish(msg_throw);
		}
		if(TZ == 3)
		{
			msg_throw.data = 111;
			pub_throw.publish(msg_throw);
		}
	}

	if(msg.data == -1)
	{
		set_servo_pulsewidth(pi, pin_servo, 1520);
		acc_move(0, 3, 0, 2, 1.2, -1, 4.42, 2);
		ros::Duration(1.941626 + 0.1).sleep();
		msg_lrf.flag = true;
		msg_lrf.type = 1;
		pub_lrf.publish(msg_lrf);
		pre_TZ = TZ;
	}
	if(msg.data == -11)
	{
		set_servo_pulsewidth(pi, pin_servo, 1520);
		acc_move(0, 3, 0, 2, 1.3, -1, 6.5, 2);
		ros::Duration(2.020908 + 0.1).sleep();
		msg_lrf.flag = true;
		msg_lrf.type = 1;
		pub_lrf.publish(msg_lrf);
		pre_TZ = TZ;
	}
	if(msg.data == -111)
	{
		set_servo_pulsewidth(pi, pin_servo, 1520);
		acc_move(0, 3, 0, 2, 4.8, -1, 6.5, 2);
		ros::Duration(3.883252 + 0.1).sleep();
		msg_lrf.flag = true;
		msg_lrf.type = 1;
		pub_lrf.publish(msg_lrf);
		pre_TZ = TZ;
		TZ_3 = true;
	}

	if(msg.data == -100)
	{
		msg_throw.data = 40;
		pub_throw.publish(msg_throw);
	}
	if(msg.data == -44)
	{
		msg_judg.data = 52;
		pub_judg.publish(msg_judg);
	}
}

void judg_cb(const std_msgs::Int8& msg)
{
	if(msg.data == 1)	//CRが離れた
	{
		if(!first)	//1回だけTZ1
		{
			TZ = 1;
			msg_throw.data = 41;
			pub_throw.publish(msg_throw);
			acc_move(0, 3, 0, 2, 1.4, -1.15, 4.42, 4);
			ros::Duration(2.1 + 0.1).sleep();
			msg_lrf.flag = true;
			msg_lrf.type = 0;
			msg_lrf.TZ = 1;
			pub_lrf.publish(msg_lrf);
			first = true;
		}

		else if(TZ == 1 && pre_TZ == 1)
		{
			set_servo_pulsewidth(pi, pin_servo, 1520);
			msg_throw.data = 41;
			pub_throw.publish(msg_throw);
			ROS_INFO("TZ1 OK!");
			acc_move(0, 3, 0, 2, 1.3, -1.15, 4.42, 4);
			ros::Duration(2.020908 + 0.1).sleep();
			msg_lrf.flag = true;
			msg_lrf.type = 0;
			msg_lrf.TZ = 1;
			pub_lrf.publish(msg_lrf);
		}
		else if(TZ == 2 && pre_TZ == 1)
		{
			set_servo_pulsewidth(pi, pin_servo, 1520);
			ROS_INFO("TZ2 OK!");
			acc_move(0, 3, 0, 2, 1, -1.1, 4.4, 2);
			ros::Duration(1.772454 + 0.05).sleep();
			acc_move(0, 3, 0, 2, 2.2, -1.2, 4.4, 1);
			ros::Duration(2.628974 + 0.1).sleep();
			acc_move(0, 3, 0, 2, 2.25, -1.1, 6.4, 4);
			ros::Duration(2.65868 + 0.1).sleep();
			msg_lrf.flag = true;
			msg_lrf.type = 0;
			msg_lrf.TZ = 2;
			pub_lrf.publish(msg_lrf);
			msg_throw.data = 41;
			pub_throw.publish(msg_throw);
		}
		else if(TZ == 3 && pre_TZ == 2)
		{
			set_servo_pulsewidth(pi, pin_servo, 1520);
			ROS_INFO("TZ3 OK!");
			acc_move(0, 3, 0, 2, 4.8, -1.15, 6.4, 4);
			ros::Duration(3.883252 + 0.1).sleep();
			msg_throw.data = 41;
			pub_throw.publish(msg_throw);
			msg_lrf.flag = true;
			msg_lrf.type = 0;
			msg_lrf.TZ = 3;
			pub_lrf.publish(msg_lrf);
		}
		else if(TZ_3)
		{
			TZ = 3;
			msg_throw.data = 41;
			pub_throw.publish(msg_throw);
			ROS_INFO("TZ3 OK!");
			acc_move(0, 3, 0, 2, 4.8, -1.15, 6.4, 4);
			ros::Duration(3.883252 + 0.1).sleep();
			msg_lrf.flag = true;
			msg_lrf.type = 0;
			msg_lrf.TZ = 3;
			pub_lrf.publish(msg_lrf);
		}
		else
		{
			set_servo_pulsewidth(pi, pin_servo, 1700);
			msg_judg.data = 52;
			pub_judg.publish(msg_judg);

		}
	}

	/*if(TZ_3)
	{
		TZ = 3;
		ROS_INFO("TZ3 OK!");
		acc_move(0, 3, 0, 2, 4.8, -1.15, 6.4, 4);
		ros::Duration(3.883252 + 0.1).sleep();
		msg_throw.data = 41;
		pub_throw.publish(msg_throw);
		msg_lrf.flag = true;
		msg_lrf.type = 0;
		msg_lrf.TZ = 3;
		pub_lrf.publish(msg_lrf);
	}
	else
	{
		TZ_3 = false;
	}*/

	if(msg.data == 2)	//true 投射成功
	{
		set_servo_pulsewidth(pi, pin_servo, 1450);
		ROS_INFO("true");
		switch(pre_TZ)
		{
			case 1:
				TZ = 2;
				break;

			case 2:
				TZ = 3;
				break;
		}
		msg_judg.data = 50;
		pub_judg.publish(msg_judg);
	}
	if(msg.data == 3)	//false 投射失敗
	{
		set_servo_pulsewidth(pi, pin_servo, 1450);
		ROS_INFO("false");
		TZ = pre_TZ;
		msg_judg.data = 50;
		pub_judg.publish(msg_judg);
	}

	ROS_INFO("pre_TZ: %d, TZ: %d", pre_TZ, TZ);
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

void Reset(pi, pin_RESET, unsigned level, uint32_t tick)
{
	if(!flag_RESET)
	{
		flag_RESET = true;
		ROS_INFO("RESET!");
	}
	else
	{
		if(flag_RESET)
		{
			flag_RESET = false;
		}
	}
}
