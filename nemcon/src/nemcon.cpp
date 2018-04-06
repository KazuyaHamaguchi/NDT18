#include <ros/ros.h>
#include <nemcon/switch_in.h>
#include <nemcon/pid_param.h>
#include <accel_decel/param.h>
#include <accel_decel/result.h>

#include <pigpiod_if2.h>

static const int pin_blue = 16;
static const int pin_yellow = 12;
static const int pin_servo = 24;

int pi = pigpio_start(0, 0);
bool cb_flag = false;
bool enc_flag = false;
bool end = false;

float acc_t = 0.0f;
float lrf_x = 0.0f;
float lrf_y = 0.0f;
float lrf_z = 0.0f;

void acc_t_cb(const accel_decel::result& msg);
void lrf_cb(const geometry_msgs::PoseStamped);

void led_flash(int num, float time, int color);	//color：blue = 0, yellow = 1
void acc_move(float Vs, float Vmax, float Ve, float Amax, float Xall, float tar_x, float tar_y, int front); //front：1前 2右 3後 4左
void lrf_move(float V, float tar_x, float tar_y);

nemcon::pid_param msg_pid_param;
accel_decel::param msg_acc_param;
ros::Publisher pub_tar_dis;
ros::Publisher pub_move_param;


void switch_cb(const nemcon::switch_in& msg)
{
	if(msg.START)
	{
		if(msg.SZ && !msg.TZ1 && !msg.TZ2 && !msg.TZ3 && !msg.SC && !cb_flag)
		{
			led_flash(0, 0, 2);
			led_flash(3, 0.1, 0);
			led_flash(-1, 0, 0);

			acc_move(0, 1, 0, 0.5, 1.05, 0, 0, 4);	//SZ横
			ros::Duration(acc_t).sleep();
			acc_move(0, 1, 0, 0.5, 4.8, -1.15, 0, 1);	//TZ1横
			ros::Duration(acc_t).sleep();
			acc_move(0, 1, 0, 0.3, 1, -1.15, 4.5, 4);	//TZ1受け渡しポイント
			ros::Duration(acc_t).sleep();
			lrf_move(3);


			led_flash(3, 0.25, 1);

			cb_flag = true;
			end = true;
		}
		if(!msg.SZ && msg.TZ1 && !msg.TZ2 && !msg.TZ3 && !msg.SC && !cb_flag)
		{
			lrf_move(3);
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
	ros::init(argc, argv, "servo_test");
	ros::NodeHandle nh;

	set_mode(pi, pin_blue, PI_OUTPUT);
	set_mode(pi, pin_yellow, PI_OUTPUT);
	set_servo_pulsewidth(pi, pin_servo, 1520);	//0度

	ros::Subscriber subSwitch = nh.subscribe("/switch", 1000, switch_cb);
	ros::Subscriber sub_accel = nh.subscribe("/accel_decel/result", 1000, acc_t_cb);
	ros::Subscriber sub_lrf = nh.subscribe("/lrf_pose", 1000, lrf_cb);


	pub_tar_dis = nh.advertise<nemcon::pid_param>("pid_param", 1000);
	pub_move_param = nh.advertise<accel_decel::param>("accel_decel/param", 1000);

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

void lrf_move(float V)
{
	bool flag_x = false;
	bool flag_y = false;
	bool flag_z = false;

	msg_pid_param.pattern = 1;
	msg_pid_param.speed = V;

	while(1)
	{
		while(1)
		{
			if(lrf_x > 0)
			{
				msg_pid_param.front = 4;
				pub_tar_dis.publish(msg_pid_param);
			}
			if(lrf_x < 0)
			{
				msg_pid_param.front = 2;
				pub_tar_dis.publish(msg_pid_param);
			}
			if(-0.01 < lrf_x && lrf_x < 0.01)
			{
				msg_pid_param.speed = 0;
				pub_tar_dis.publish(msg_pid_param);
				flag_x = true;
				break;
			}
		}
		while(1)
		{
			if(lrf_y > 0)
			{
				msg_pid_param.front = 3;
				pub_tar_dis.publish(msg_pid_param);
			}
			if(lrf_y < 0)
			{
				msg_pid_param.front = 1;
				pub_tar_dis.publish(msg_pid_param);
			}
			if(-0.01 < lrf_y && lrf_y < 0.01)
			{
				msg_pid_param.speed = 0;
				pub_tar_dis.publish(msg_pid_param);
				flag_y = true;
				break;
			}
		}
		while(1)
		{
			msg_pid_param.speed = -1;
			pub_tar_dis.publish(msg_pid_param);
			if(-0.01 < lrf_z && lrf_z < 0.01)
			{
				msg_pid_param.speed = 0;
				pub_tar_dis.publish(msg_pid_param);
				flag_z = true;
				break;
			}
		}
		if(flag_x && flag_y && flag_z)
		{
			break;
		}
	}
}

void acc_t_cb(const accel_decel::result& msg)
{
	acc_t = msg.t;
}

void lrf_cb(const geometry_msgs::PoseStamped& msg)
{
	lrf_x = msg.pose.position.x;
	lrf_y = msg.pose.position.y;
	lrf_y = msg.pose.pose.orientation.z;
}
