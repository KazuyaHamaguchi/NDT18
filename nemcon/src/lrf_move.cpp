#include <ros/ros.h>
#include <nemcon/pid_param.h>
#include <geometry_msgs/PoseStamped.h>
#include <accel_decel/result.h>
#include <nemcon/lrf_flag.h>
#include <std_msgs/Int8.h>

void pose_cb(const geometry_msgs::PoseStamped& msg);
void lrf_cb(const geometry_msgs::PoseStamped& msg);
void flag_cb(const nemcon::lrf_flag& msg);

int type = 0;

bool flag = false;
bool flag_x = false;
bool flag_y = false;
bool flag_z = false;

float x = 0.0f;
float y = 0.0f;

float enc_x = 0.0f;
float enc_y = 0.0f;

float lrf_x = 0.0f;
float lrf_y = 0.0f;
float lrf_z = 0.0f;

float offset_x = 0.0f;
float offset_y = 0.0f;

float t = 0.0f;

bool first = false;

ros::Time current_time, last_time;

accel_decel::result msg_acc;
nemcon::pid_param msg_pid_param;
std_msgs::Int8 msg_lrf;
ros::Publisher pub_tar_dis;
ros::Publisher pub_acc;
ros::Publisher pub_lrf;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lrf_move");
	ros::NodeHandle nh;

	ros::Rate loop_rate(40);

	ros::Subscriber sub_enc = nh.subscribe("/robot/pose", 1000, pose_cb);
	ros::Subscriber sub_lrf = nh.subscribe("/lrf_pose", 1000, lrf_cb);
	ros::Subscriber sub_flag = nh.subscribe("lrf_flag", 1000, flag_cb);

	pub_tar_dis = nh.advertise<nemcon::pid_param>("pid_param", 1000);
	pub_acc = nh.advertise<accel_decel::result>("accel_decel/result", 1000);
	pub_lrf = nh.advertise<std_msgs::Int8>("lrf", 1000);

	msg_pid_param.speed = 1;
	msg_acc.Vmax = false;

	while(ros::ok())
	{
		current_time = ros::Time::now();
		t += (current_time - last_time).toSec();

	if(type == 0)
	{
		x = lrf_x;
		y = lrf_y;
		msg_pid_param.pattern = 2;
	}
	else if(type == 1)
	{
		x = enc_x;
		y = enc_y;
		msg_pid_param.pattern = 1;
	}

		if(flag)
		{
				if(!flag_x && !flag_y)
				{
					if(0.1 + offset_x < x)
					{
						ROS_INFO("lrf_x:%f", x);
						msg_acc.V = 0.2;
						msg_pid_param.front = 4;
						pub_tar_dis.publish(msg_pid_param);
						pub_acc.publish(msg_acc);
						first = false;
					}
					if(0.01 + offset_x < x && x <= 0.1 + offset_x)
					{
						ROS_INFO("lrf_x2:%f", x);
						msg_acc.V = 0.05;
						msg_pid_param.front = 4;
						pub_tar_dis.publish(msg_pid_param);
						pub_acc.publish(msg_acc);
						first = false;
					}
					if(x < -0.1 + offset_x)
					{
						ROS_INFO("-lrf_x:%f", x);
						msg_acc.V = 0.2;
						msg_pid_param.front = 2;
						pub_tar_dis.publish(msg_pid_param);
						pub_acc.publish(msg_acc);
						first = false;
					}
					if(-0.1 + offset_x <= x && x < -0.01 + offset_x)
					{
						ROS_INFO("-lrf_x2:%f", x);
						msg_acc.V = 0.05;
						msg_pid_param.front = 2;
						pub_tar_dis.publish(msg_pid_param);
						pub_acc.publish(msg_acc);
						first = false;
					}
					if(-0.01 + offset_x <= x && x <= 0.01 + offset_x /*&& first_x*/)
					{
						ROS_INFO("lrf_x OK");
						msg_acc.V = 0;
						pub_tar_dis.publish(msg_pid_param);
						pub_acc.publish(msg_acc);
						flag_x = true;
					}
					else
					{
						flag_x = false;
						first = false;
					}
				}


				if(flag_x && !flag_y)
				{
					if(0.1 + offset_y < y)
					{
						ROS_INFO("lrf_y:%f", y);
						msg_acc.V = 0.2;
						msg_pid_param.front = 3;
						pub_tar_dis.publish(msg_pid_param);
						pub_acc.publish(msg_acc);
						flag_y = false;
						first = false;
					}
					if(0.01 + offset_y < y && y <= 0.1 + offset_y)
					{
						ROS_INFO("lrf_y2:%f", y);
						msg_acc.V = 0.05;
						msg_pid_param.front = 3;
						pub_tar_dis.publish(msg_pid_param);
						pub_acc.publish(msg_acc);
						flag_y = false;
						first = false;
					}
					if(y < -0.1 + offset_y)
					{
						ROS_INFO("-lrf_y:%f", y);
						msg_acc.V = 0.2;
						msg_pid_param.front = 1;
						pub_tar_dis.publish(msg_pid_param);
						pub_acc.publish(msg_acc);
						flag_y = false;
						first = false;
					}
					if(-0.1 + offset_y <= y && y <= -0.01 + offset_y)
					{
						ROS_INFO("-lrf_y2:%f", lrf_y);
						msg_acc.V = 0.05;
						msg_pid_param.front = 1;
						pub_tar_dis.publish(msg_pid_param);
						pub_acc.publish(msg_acc);
						flag_y = false;
						first = false;
					}
					if(-0.01 + offset_y <= y && y <= 0.01 + offset_y)
					{
						ROS_INFO("lrf_y OK");
						msg_acc.V = 0;
						pub_tar_dis.publish(msg_pid_param);
						pub_acc.publish(msg_acc);
						if(-0.01 + offset_x <= x && x <= 0.01 + offset_x && t >= 0.2)
						{
							ROS_INFO("lrf OK");
							msg_lrf.data = -50;
							pub_lrf.publish(msg_lrf);
							flag = false;
							flag_x = false;
							flag_y = false;
							first = true;
							t = 0.0f;
						}
						else
						{
							flag_x = false;
						}
					}
					else
					{
						flag_y = false;
						first = false;
						t = 0.0f;
					}
				}
		}
		else
		{
			flag_x = false;
			flag_y = false;
			first = false;
		}

		last_time = current_time;

		loop_rate.sleep();
		ros::spinOnce();
	}
}

void pose_cb(const geometry_msgs::PoseStamped& msg)
{
	enc_x = msg.pose.position.x;
	enc_y = msg.pose.position.y;
}

void lrf_cb(const geometry_msgs::PoseStamped& msg)
{
	lrf_x = msg.pose.position.x;
	lrf_y = msg.pose.position.y;
	lrf_z = msg.pose.orientation.z;
	//ROS_INFO("%f\t %f\t %f\t", lrf_x, lrf_y, lrf_z);
}

void flag_cb(const nemcon::lrf_flag& msg)
{
	flag = msg.flag;
	type = msg.type;
	if(!flag)
	{
		if(!first)
		{
			ROS_INFO("lrf_move first!");
			msg_pid_param.pattern = 100;
			pub_tar_dis.publish(msg_pid_param);
			msg_lrf.data = -50;
			pub_lrf.publish(msg_lrf);
			flag_x = false;
			flag_y = false;
			first = true;
		}
		else;
	}
	else
	{
		first = false;
	}

	if(type == 0)
	{
		switch(msg.TZ)
		{
			case 1: case 3:
				offset_x = 0.0f;
				offset_y = 0.0f;
				break;

			case 2:
				offset_x = 3.27503521586;
				offset_y = 0.0f;
				break;

			default:
				offset_x = 0.0f;
				offset_y = 0.0f;
				break;
		}
	}
	if(type == 1)
	{
		offset_x = -2.00f;
		offset_y = enc_y;
	}
}
