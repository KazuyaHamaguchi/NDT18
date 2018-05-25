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
bool RESET = false;
bool ERROR = false;
bool cb_flag = false;
bool enc_flag = false;
bool end = false;
bool first = false;
bool lrf = false;
bool throw_on = false;
bool TZ_3 = false;
bool TZ_3_receive = false;


bool flag_RESET = false;

int TZ = 0;
int pre_TZ = 0;

float acc_t = 0.0f;

void switch_cb(const nemcon::switch_in& msg);
void receive_cb(const std_msgs::Int8& msg);
void judg_cb(const std_msgs::Int8& msg);
void lrf_cb(const std_msgs::Int8& msg);

void reset();

void led_flash(int num, float time, int color);	//color：blue = 0, yellow = 1
void acc_move(float Vs, float Vmax, float Ve, float Amax, float Xall, float tar_x, float tar_y, int front); //front：1前 2右 3後 4左

nemcon::pid_param msg_pid_param;
accel_decel::param msg_acc_param;
std_msgs::Int8 msg_throw;
nemcon::lrf_flag msg_lrf;
std_msgs::Int8 msg_judg;
nemcon::switch_in msg_switch;
std_msgs::Int8 msg_throw_on;
std_msgs::Int8 msg_TZ_judg;
std_msgs::Int8 msg_lrf2;

ros::Publisher pub_tar_dis;
ros::Publisher pub_move_param;
ros::Publisher pub_throw;
ros::Publisher pub_lrf;
ros::Publisher pub_judg;
ros::Publisher pub_switch;
ros::Publisher pub_receive;
ros::Publisher pub_judg2;
ros::Publisher pub_lrf2;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "nemcon2");
	ros::NodeHandle nh;

	set_mode(pi, pin_blue, PI_OUTPUT);
	set_mode(pi, pin_yellow, PI_OUTPUT);
	set_servo_pulsewidth(pi, pin_servo, 1520);	//0度

	ros::Subscriber sub_Switch = nh.subscribe("switch", 1000, switch_cb);
	ros::Subscriber sub_receive = nh.subscribe("Throw_on", 1000, receive_cb);
	ros::Subscriber sub_judg = nh.subscribe("TZ_judg", 1000, judg_cb);
	ros::Subscriber sub_lrf = nh.subscribe("lrf", 1000, lrf_cb);

	pub_tar_dis = nh.advertise<nemcon::pid_param>("pid_param", 1000);
	pub_move_param = nh.advertise<accel_decel::param>("accel_decel/param", 1000);
	pub_throw = nh.advertise<std_msgs::Int8>("Throw_on_1", 1000);
	pub_lrf = nh.advertise<nemcon::lrf_flag>("lrf_flag", 1000);
	pub_judg = nh.advertise<std_msgs::Int8>("judg_call", 1000);
	pub_switch = nh.advertise<nemcon::switch_in>("switch", 1000);
	pub_receive = nh.advertise<std_msgs::Int8>("Throw_on", 1000);
	pub_judg2 = nh.advertise<std_msgs::Int8>("TZ_judg", 1000);
	pub_lrf2 = nh.advertise<std_msgs::Int8>("lrf", 1000);

	ros::Rate loop_rate(20);

	while(ros::ok())
	{
		if(!end && RESET)
		{
			ROS_INFO("end: %d", end);
			reset();
		}
		else;

		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}


void switch_cb(const nemcon::switch_in& msg)
{
	if(msg.RESET)
	{
		RESET = true;
	}

	if(msg.START)
	{
		end = false;
		RESET = false;
		if(msg.SZ && !msg.TZ1 && !msg.TZ2 && !msg.TZ3 && !msg.SC && !cb_flag)	//SZから通常通り
		{
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
			acc_move(0, 3, 0, 2, 0.95, -1.15, 4.42, 4);	//TZ1受け渡しポイント
			ros::Duration(1.772454 + 0.1).sleep();

			msg_throw.data = 30;
			pub_throw.publish(msg_throw);

			msg_throw.data = 3;
			pub_throw.publish(msg_throw);

			msg_lrf.flag = true;
			msg_lrf.type = 1;
			pub_lrf.publish(msg_lrf);

			msg_throw.data = 40;	//受け取り待機
			pub_throw.publish(msg_throw);

			cb_flag = true;
		}
		if(!msg.SZ && msg.TZ1 && !msg.TZ2 && !msg.TZ3 && !msg.SC && !cb_flag)
		{
			led_flash(3, 0.25, 1);
			cb_flag = true;
		}
	}
	end = true;
}

void lrf_cb(const std_msgs::Int8& msg)
{
	end = false;
	if(msg.data == -50)	//lrf終了時
	{
		msg_pid_param.pattern = 3;	//ブレーキかける
		pub_tar_dis.publish(msg_pid_param);
		if(TZ == 3 && pre_TZ == 2)	//TZ2 → TZ3時に投射へ
		{
			set_servo_pulsewidth(pi, pin_servo, 950);	//90度
			ros::Duration(1).sleep();
			msg_throw.data = 111;
			pub_throw.publish(msg_throw);
		}
		else;
	}
	else
	{
		msg_pid_param.pattern = 99;
		pub_tar_dis.publish(msg_pid_param);
		if(msg.data == 99)
		{
			end = true;
		}
		else
		{
			end - false;
		}
		ROS_INFO("lrf_2 OK");
	}
	ROS_INFO("lrf_cb: %d", msg.data);
}

void receive_cb(const std_msgs::Int8& msg)
{
	end = false;
	if(msg.data == -40)	//CRからの受け取りに成功
	{
		if(!first || TZ == 3)
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

	else if(msg.data == -41)	//TR内での受け渡しに成功
	{

		if(TZ == 1)			//TZ1投射指示
		{
			set_servo_pulsewidth(pi, pin_servo, 950);	//90度
			msg_lrf.flag = false;
			pub_lrf.publish(msg_lrf);
			ros::Duration(1).sleep();
			msg_throw.data = 1;
			pub_throw.publish(msg_throw);
		}
		else if(TZ == 2)	//TZ2投射指示
		{
			set_servo_pulsewidth(pi, pin_servo, 950);	//90度
			msg_lrf.flag = false;
			pub_lrf.publish(msg_lrf);
			ros::Duration(1).sleep();
			msg_throw.data = 11;
			pub_throw.publish(msg_throw);
		}
		else if(TZ == 3)	//TZ3投射指示
		{
			if(!TZ_3_receive)	//
			{
				msg_throw.data = 45;
				pub_throw.publish(msg_throw);
				TZ_3_receive = true;
			}
			else
			{
				ROS_INFO("TZ_3_receive: %d", TZ_3_receive);
				set_servo_pulsewidth(pi, pin_servo, 950);	//90度
				msg_lrf.flag = false;
				pub_lrf.publish(msg_lrf);
				ros::Duration(1).sleep();
				msg_throw.data = 111;
				pub_throw.publish(msg_throw);
			}
		}
	}

	else if(msg.data == -45)
	{
		msg_throw.data = 40;	//受け取り待機
		pub_throw.publish(msg_throw);
	}

	else if(msg.data == -1)
	{
		set_servo_pulsewidth(pi, pin_servo, 1520);
		acc_move(0, 3, 0, 2, 1.2, -1, 4.42, 2);
		ros::Duration(1.941626 + 0.1).sleep();
		msg_lrf.flag = true;
		msg_lrf.type = 1;
		pub_lrf.publish(msg_lrf);
		pre_TZ = TZ;
	}
	else if(msg.data == -11)
	{
		set_servo_pulsewidth(pi, pin_servo, 1520);
		acc_move(0, 3, 0, 2, 1.3, -1, 6.4, 2);
		ros::Duration(2.020908 + 0.1).sleep();
		msg_lrf.flag = true;
		msg_lrf.type = 1;
		pub_lrf.publish(msg_lrf);
		pre_TZ = TZ;
	}
	else if(msg.data == -111)
	{
		set_servo_pulsewidth(pi, pin_servo, 1520);
		acc_move(0, 2.5, 0, 2, 4.8, -1, 6.4, 2);
		ros::Duration(3.769912 + 0.1).sleep();
		msg_lrf.flag = true;
		msg_lrf.type = 1;
		pub_lrf.publish(msg_lrf);
		pre_TZ = TZ;
		TZ_3 = true;
	}

	else if(msg.data == -100)
	{
		if(TZ == 3 && pre_TZ == 2)
		{
			msg_throw.data = 41;
			pub_throw.publish(msg_throw);
			msg_lrf.flag = true;
			msg_lrf.type = 0;
			msg_lrf.TZ = 3;
			pub_lrf.publish(msg_lrf);
			pre_TZ = TZ;
		}
		else
		{
			msg_throw.data = 40;
			pub_throw.publish(msg_throw);
		}
	}
	else if(msg.data == -44)
	{
		msg_judg.data = 52;
		pub_judg.publish(msg_judg);
	}
	else
	{
		if(msg.data == 99)
		{
			end = true;
		}
		else
		{
			end = false;
		}
		ROS_INFO("Receive OK");
	}
	ROS_INFO("receive_cb: %d", msg.data);
}

void judg_cb(const std_msgs::Int8& msg)
{
	end = false;
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

		else if(TZ == 1 && pre_TZ == 1)	//TZ1 → TZ1
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
		else if(TZ == 2 && pre_TZ == 1)	//TZ1 → TZ2
		{
			set_servo_pulsewidth(pi, pin_servo, 1520);
			ROS_INFO("TZ2 OK!");
			acc_move(0, 3, 0, 2, 1.25, -1.1, 4.4, 2);
			ros::Duration(1.981664 + 0.1).sleep();
			acc_move(0, 3, 0, 2, 2.2, -1.0, 4.4, 1);
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
		else if(TZ == 2 && pre_TZ == 2)	//TZ2 → TZ2
		{
			set_servo_pulsewidth(pi, pin_servo, 1520);
			ROS_INFO("TZ2 OK!");
			acc_move(0, 3, 0, 2, 1.3, -1, 6.4, 4);
			ros::Duration(2.65868 + 0.1).sleep();
			msg_lrf.flag = true;
			msg_lrf.type = 0;
			msg_lrf.TZ = 2;
			pub_lrf.publish(msg_lrf);
			msg_throw.data = 41;
			pub_throw.publish(msg_throw);
		}
		else if(TZ == 3 && pre_TZ == 2)	//TZ2 → TZ3
		{
			if(!TZ_3_receive)
			{
				set_servo_pulsewidth(pi, pin_servo, 1520);
				ROS_INFO("TZ3 OK!");
				msg_throw.data = 41;
				pub_throw.publish(msg_throw);
			}
			else
			{
				acc_move(0, 2.4, 0, 2, 4.8, -1.15, 6.4, 4);
				ros::Duration(3.769912 + 0.1).sleep();
				msg_lrf.flag = true;
				msg_lrf.type = 0;
				msg_lrf.TZ = 3;
				pub_lrf.publish(msg_lrf);
			}
			/*set_servo_pulsewidth(pi, pin_servo, 1520);
			ROS_INFO("TZ3 OK!");
			acc_move(0, 2.4, 0, 2, 4.8, -1.15, 6.4, 4);
			ros::Duration(3.769912 + 0.1).sleep();
			msg_lrf.flag = true;
			msg_lrf.type = 0;
			msg_lrf.TZ = 3;
			pub_lrf.publish(msg_lrf);
			msg_throw.data = 41;
			pub_throw.publish(msg_throw);*/
		}
		else if(TZ_3)	//TZ3 → TZ3
		{
			TZ = 3;
			ROS_INFO("TZ3 OK!");
			acc_move(0, 2.4, 0, 2, 4.8, -1.15, 6.4, 4);
			ros::Duration(3.769912 + 0.1).sleep();
			msg_lrf.flag = true;
			msg_lrf.type = 0;
			msg_lrf.TZ = 3;
			pub_lrf.publish(msg_lrf);
			msg_throw.data = 41;
			pub_throw.publish(msg_throw);
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

	else if(msg.data == 2)	//true 投射成功
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
	else if(msg.data == 3)	//false 投射失敗
	{
		set_servo_pulsewidth(pi, pin_servo, 1700);
		ROS_INFO("false");
		TZ = pre_TZ;
		msg_judg.data = 50;
		pub_judg.publish(msg_judg);
	}
	else
	{
		if(msg.data == 99)
		{
			//ROS_INFO("%d", msg.data);
			end = true;
		}
		else
		{
			end = false;
		}
		ROS_INFO("judg ok");
	}
	ROS_INFO("judg_cb: %d", msg.data);

	ROS_INFO("pre_TZ: %d, TZ: %d", pre_TZ, TZ);
}

void reset()
{
	set_servo_pulsewidth(pi, pin_servo, 1520);

	msg_switch.RESET = true;
	pub_switch.publish(msg_switch);

	msg_lrf.flag = false;
	msg_lrf.type = 99;
	pub_lrf.publish(msg_lrf);

	msg_acc_param.flag = false;
	pub_move_param.publish(msg_acc_param);

	msg_pid_param.pattern = 99;
	pub_tar_dis.publish(msg_pid_param);

	msg_TZ_judg.data = 99;
	pub_judg2.publish(msg_TZ_judg);

	msg_judg.data = 99;
	pub_judg.publish(msg_judg);

	msg_throw_on.data = 99;
	pub_receive.publish(msg_throw_on);

	//ros::Duration(2).sleep();

	msg_lrf2.data = 99;
	pub_lrf2.publish(msg_lrf2);

	//ros::Duration(2).sleep();

	msg_throw.data = 99;
	pub_throw.publish(msg_throw);

	cb_flag = false;
	enc_flag = false;
	end = false;
	first = false;
	lrf = false;
	throw_on = false;
	TZ_3 = false;
	TZ_3_receive = false;

	flag_RESET = false;

	TZ = 0;
	pre_TZ = 0;

	acc_t = 0.0f;

	//ros::Duration(2).sleep();

	msg_switch.RESET = true;
	pub_switch.publish(msg_switch);

	RESET = false;
	end = false;
}

void led_flash(int num, float time, int color)
{
	if(num > 0)		//点滅
	{
		for(int i = 0; i < num; i++)
		{
			if(color == 0)
			{
				gpio_write(pi, pin_yellow, 0);
				gpio_write(pi, pin_blue, 1);
				ros::Duration(time / 2).sleep();
				gpio_write(pi, pin_blue, 0);
				ros::Duration(time / 2).sleep();
			}
			if(color == 1)
			{
				gpio_write(pi, pin_blue, 0);
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
			gpio_write(pi, pin_yellow, 0);
			ros::Duration(time).sleep();
		}
		if(color == 1)
		{
			gpio_write(pi, pin_blue, 0);
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
	msg_acc_param.flag = true;
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
